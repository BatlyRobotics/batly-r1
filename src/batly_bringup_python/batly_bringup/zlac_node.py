#!/usr/bin/env python3
"""
batly_bringup/zlac_node.py

ROS 2 Python driver for ZLAC8015D / ZLAC8030D over RS485 Modbus RTU.

Subscribes : /cmd_vel               (geometry_msgs/Twist)
Publishes  : /raw_odom              (nav_msgs/Odometry)
TF         : odom → base_footprint  (if publish_tf)

Services (all under ~/):
  Essential
    enable_motor      (std_srvs/SetBool)   true=enable(0x08) false=shutdown(0x07)
    emergency_stop    (std_srvs/Trigger)   write control word 0x05
    clear_fault       (std_srvs/Trigger)   write control word 0x06
    reset_odometry    (std_srvs/Trigger)   zero x, y, yaw in software

  Recommended
    get_status        (std_srvs/Trigger)   decode status word 0x20A2
    get_fault_code    (std_srvs/Trigger)   decode fault code 0x20A5/0x20A6
    get_temperature   (std_srvs/Trigger)   read motor + driver temperature
    set_parking_mode  (std_srvs/SetBool)   write 0x200C (limit current to 3 A)

  Nice-to-have
    reset_encoder     (std_srvs/Trigger)   write 0x2005 = 0x0003 (clear L+R)
    set_max_speed     (std_srvs/SetBool)   toggles between conservative/max
                                            (also see: ROS param `max_rpm`)
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from std_srvs.srv import SetBool, Trigger

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException


# ── Modbus register / value constants (ZLAC8015D / 8030D datasheet) ──────────

REG_CONTROL_WORD  = 0x200E
REG_PARKING_MODE  = 0x200C
REG_RESET_FB_POS  = 0x2005
REG_RPM_START     = 0x2088
REG_STATUS_WORD   = 0x20A2
REG_MOTOR_TEMP    = 0x20A4
REG_FAULT_L       = 0x20A5
REG_FAULT_R       = 0x20A6
REG_POS_L_HI      = 0x20A7
REG_POS_R_HI      = 0x20A9
REG_VEL_FEEDBACK  = 0x20AB
REG_DRIVER_TEMP   = 0x20B0

CW_EMERGENCY_STOP = 0x0005
CW_CLEAR_FAULT    = 0x0006
CW_SHUTDOWN       = 0x0007
CW_ENABLE         = 0x0008

SIGMA_VX     = 0.02      # m/s
SIGMA_VYAW   = 0.061     # rad/s
SIGMA_XY     = 0.05      # m    - pose is not fused; for downstream consumers
SIGMA_YAW    = 0.05      # rad  - likewise
SIGMA_UNUSED = 1.0e3     # large: planar platform, these DOF are meaninglessLE         = 0x0008

FAULT_CODES = {
    0x0000: "No error",
    0x0001: "Over-voltage",
    0x0002: "Under-voltage",
    0x0004: "Over-current",
    0x0008: "Overload",
    0x0020: "Encoder out-of-tolerance",
    0x0080: "Reference voltage error",
    0x0100: "EEPROM read/write error",
    0x0200: "Hall sensor error",
    0x0400: "Motor over-temperature",
    0x0800: "Encoder error",
    0x1000: "Driver over-temperature",
    0x2000: "Speed setting exceeds rated speed",
}


def _s16_to_u16(v: int) -> int:
    return v & 0xFFFF


def _u16_to_s16(v: int) -> int:
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def _i32_from_u32(u: int) -> int:
    if u & 0x80000000:
        return -((~u + 1) & 0xFFFFFFFF)
    return u & 0x7FFFFFFF


def _decode_fault(code: int) -> str:
    if code == 0:
        return "No error"
    parts = [msg for bit, msg in FAULT_CODES.items() if bit != 0 and (code & bit)]
    return " | ".join(parts) if parts else f"Unknown 0x{code:04X}"


# ── Main node ─────────────────────────────────────────────────────────────────

class ZLACNode(Node):
    def __init__(self) -> None:
        super().__init__("zlac8015d_node")

        # ── Hardware ──────────────────────────────────────────────────
        self._port = self.declare_parameter(
            "port", "/dev/rs485"
        ).get_parameter_value().string_value
        self._baudrate = self.declare_parameter(
            "baudrate", 115200
        ).get_parameter_value().integer_value
        self._slave = self.declare_parameter(
            "slave_id", 1
        ).get_parameter_value().integer_value

        # ── Geometry ──────────────────────────────────────────────────
        self._wheel_radius = self.declare_parameter(
            "wheel_radius", 0.0535
        ).get_parameter_value().double_value
        self._wheel_base = self.declare_parameter(
            "wheel_base", 0.23
        ).get_parameter_value().double_value
        self._ticks_per_rev = self.declare_parameter(
            "ticks_per_rev", 4096.0
        ).get_parameter_value().double_value
        self._gear_ratio = self.declare_parameter(
            "gear_ratio", 1.0
        ).get_parameter_value().double_value

        # ── Timing ────────────────────────────────────────────────────
        self._cmd_rate_hz = self.declare_parameter(
            "cmd_rate_hz", 20.0
        ).get_parameter_value().double_value
        self._odom_rate_hz = self.declare_parameter(
            "odom_rate_hz", 50.0
        ).get_parameter_value().double_value
        self._cmd_timeout = self.declare_parameter(
            "cmd_timeout", 0.5
        ).get_parameter_value().double_value

        # ── Limits ────────────────────────────────────────────────────
        self._max_rpm = self.declare_parameter(
            "max_rpm", 200
        ).get_parameter_value().integer_value
        # for set_max_speed service toggle
        self._max_rpm_high = self._max_rpm
        self._max_rpm_low  = max(1, self._max_rpm // 4)   # 25% of max

        # ── Flip flags (match zlac_node.cpp) ──────────────────────────
        self._flip_cmd_left = self.declare_parameter(
            "flip_cmd_left", False
        ).get_parameter_value().bool_value
        self._flip_left = self.declare_parameter(
            "flip_left", False
        ).get_parameter_value().bool_value
        self._flip_right = self.declare_parameter(
            "flip_right", True
        ).get_parameter_value().bool_value

        # ── Frames ────────────────────────────────────────────────────
        self._frame_odom = self.declare_parameter(
            "frame_odom", "odom"
        ).get_parameter_value().string_value
        self._frame_base = self.declare_parameter(
            "frame_base", "base_footprint"
        ).get_parameter_value().string_value
        self._publish_tf = self.declare_parameter(
            "publish_tf", False
        ).get_parameter_value().bool_value

        # ── Misc ──────────────────────────────────────────────────────
        self._init_enable_only = self.declare_parameter(
            "init_enable_only", True
        ).get_parameter_value().bool_value
        self._rpm_write_retries = self.declare_parameter(
            "rpm_write_retries", 2
        ).get_parameter_value().integer_value
        self._mb_timeout_ms = self.declare_parameter(
            "mb_timeout_ms", 150
        ).get_parameter_value().integer_value

        # ── pymodbus client ───────────────────────────────────────────
        self._client = ModbusSerialClient(
            port=self._port,
            baudrate=self._baudrate,
            bytesize=8, parity="N", stopbits=1,
            timeout=self._mb_timeout_ms / 1000.0,
        )
        if self._client.connect():
            self.get_logger().info(
                f"Serial OK: {self._port} @ {self._baudrate} baud (pymodbus)"
            )
        else:
            self.get_logger().error(f"Serial open FAILED: {self._port}")

        # ── state ─────────────────────────────────────────────────────
        self._v_cmd = 0.0
        self._w_cmd = 0.0
        self._last_cmd_stamp = self.get_clock().now()
        self._last_odom_stamp = self.get_clock().now()

        self._enc_init = False
        self._prev_l = 0
        self._prev_r = 0
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # ── Enable motors ─────────────────────────────────────────────
        if self._client.connected and self._init_enable_only:
            ok = False
            for i in range(5):
                if self._write_control_word(CW_ENABLE):
                    ok = True
                    self.get_logger().info(
                        f"Motor ENABLE OK "
                        f"(reg=0x{REG_CONTROL_WORD:04X} val=0x{CW_ENABLE:04X})"
                    )
                    break
                self.get_logger().warn(f"Motor ENABLE retry {i + 1}/5 ...")
                time.sleep(0.2)
            if not ok:
                self.get_logger().error(
                    "Motor ENABLE FAILED — motors will NOT move"
                )

        # ── ROS interfaces ────────────────────────────────────────────
        self._sub_cmd = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10
        )
        self._pub_odom = self.create_publisher(Odometry, "/raw_odom", 10)
        self._tf_pub = TransformBroadcaster(self)

        self._timer_cmd = self.create_timer(
            1.0 / self._cmd_rate_hz, self._loop_send_rpm
        )
        self._timer_odom = self.create_timer(
            1.0 / self._odom_rate_hz, self._loop_read_odom
        )

        # ── Services (10 total) ───────────────────────────────────────
        self._srv_enable = self.create_service(
            SetBool,  "~/enable_motor",     self._srv_enable_motor
        )
        self._srv_estop = self.create_service(
            Trigger,  "~/emergency_stop",   self._srv_emergency_stop
        )
        self._srv_clear = self.create_service(
            Trigger,  "~/clear_fault",      self._srv_clear_fault
        )
        self._srv_reset_odom = self.create_service(
            Trigger,  "~/reset_odometry",   self._srv_reset_odometry
        )
        self._srv_status = self.create_service(
            Trigger,  "~/get_status",       self._srv_get_status
        )
        self._srv_fault = self.create_service(
            Trigger,  "~/get_fault_code",   self._srv_get_fault_code
        )
        self._srv_temp = self.create_service(
            Trigger,  "~/get_temperature",  self._srv_get_temperature
        )
        self._srv_parking = self.create_service(
            SetBool,  "~/set_parking_mode", self._srv_set_parking_mode
        )
        self._srv_reset_enc = self.create_service(
            Trigger,  "~/reset_encoder",    self._srv_reset_encoder
        )
        self._srv_max_speed = self.create_service(
            SetBool,  "~/set_max_speed",    self._srv_set_max_speed
        )

        self.get_logger().info(
            f"READY | slave={self._slave} | "
            f"flip_cmd_left={'T' if self._flip_cmd_left else 'F'} | "
            f"flip_enc(L={'T' if self._flip_left else 'F'} "
            f"R={'T' if self._flip_right else 'F'}) | "
            f"wheel_r={self._wheel_radius:.4f} "
            f"wheel_base={self._wheel_base:.4f} | "
            f"max_rpm={self._max_rpm}"
        )
        self.get_logger().info(
            "Services available under ~/  "
            "(enable_motor, emergency_stop, clear_fault, reset_odometry, "
            "get_status, get_fault_code, get_temperature, set_parking_mode, "
            "reset_encoder, set_max_speed)"
        )

    # ── low-level helpers ─────────────────────────────────────────────

    def _write_control_word(self, value: int) -> bool:
        try:
            rr = self._client.write_register(
                address=REG_CONTROL_WORD, value=value, slave=self._slave
            )
            return not rr.isError()
        except ModbusException:
            return False

    def _write_register(self, reg: int, value: int) -> bool:
        try:
            rr = self._client.write_register(
                address=reg, value=value, slave=self._slave
            )
            return not rr.isError()
        except ModbusException:
            return False

    def _read_registers(self, reg: int, count: int):
        try:
            rr = self._client.read_holding_registers(
                address=reg, count=count, slave=self._slave
            )
            if rr.isError():
                return None
            return rr.registers
        except ModbusException:
            return None

    def _reconnect(self) -> bool:
        try:
            self._client.close()
        except Exception:
            pass
        return self._client.connect()

    # ── /cmd_vel callback ─────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._v_cmd = msg.linear.x
        self._w_cmd = msg.angular.z
        self._last_cmd_stamp = self.get_clock().now()

    # ── Kinematics ────────────────────────────────────────────────────

    def _cmd_to_rpm(self, v: float, w: float) -> tuple[int, int]:
        vL = v - (self._wheel_base / 2.0) * w
        vR = v + (self._wheel_base / 2.0) * w

        def to_rpm(v_ms: float) -> float:
            return (v_ms / self._wheel_radius) * (60.0 / (2.0 * math.pi))

        rpmL = max(-self._max_rpm, min(self._max_rpm, int(round(to_rpm(vL)))))
        rpmR = max(-self._max_rpm, min(self._max_rpm, int(round(to_rpm(vR)))))

        if self._flip_cmd_left:
            rpmL = -rpmL
        rpmR = -rpmR
        return rpmL, rpmR

    def _loop_send_rpm(self) -> None:
        if not self._client.connected:
            self._reconnect()
            return

        elapsed = (self.get_clock().now() - self._last_cmd_stamp).nanoseconds * 1e-9
        v = 0.0 if elapsed > self._cmd_timeout else self._v_cmd
        w = 0.0 if elapsed > self._cmd_timeout else self._w_cmd

        rpmL, rpmR = self._cmd_to_rpm(v, w)
        values = [_s16_to_u16(rpmL), _s16_to_u16(rpmR)]

        ok = False
        for _ in range(self._rpm_write_retries + 1):
            try:
                rr = self._client.write_registers(
                    address=REG_RPM_START, values=values, slave=self._slave
                )
                if not rr.isError():
                    ok = True
                    break
            except ModbusException:
                pass
            self._write_control_word(CW_ENABLE)
            time.sleep(0.010)

        if not ok:
            self.get_logger().warning(
                f"Write RPM failed (L={rpmL} R={rpmR}) — reconnecting",
                throttle_duration_sec=2.0,
            )
            self._reconnect()

    # ── Odometry ──────────────────────────────────────────────────────

    def _loop_read_odom(self) -> None:
        if not self._client.connected:
            return

        regs_l = self._read_registers(REG_POS_L_HI, 2)
        if regs_l is None:
            return
        regs_r = self._read_registers(REG_POS_R_HI, 2)
        if regs_r is None:
            return

        posL = _i32_from_u32((regs_l[0] << 16) | regs_l[1])
        posR = _i32_from_u32((regs_r[0] << 16) | regs_r[1])

        if self._flip_left:
            posL = -posL
        if self._flip_right:
            posR = -posR

        t_now = self.get_clock().now()
        dt = (t_now - self._last_odom_stamp).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self._last_odom_stamp = t_now

        if not self._enc_init:
            self._prev_l = posL
            self._prev_r = posR
            self._enc_init = True
            self.get_logger().info(f"Encoder initialized: L={posL} R={posR}")
            return

        mpt = (2.0 * math.pi * self._wheel_radius) / (
            self._ticks_per_rev * self._gear_ratio
        )
        dl = (posL - self._prev_l) * mpt
        dr = (posR - self._prev_r) * mpt
        self._prev_l = posL
        self._prev_r = posR

        d = (dl + dr) * 0.5
        dtheta = (dr - dl) / self._wheel_base

        self._x += d * math.cos(self._yaw + dtheta * 0.5)
        self._y += d * math.sin(self._yaw + dtheta * 0.5)
        self._yaw += dtheta

        self._publish_odom(t_now, d / dt, dtheta / dt)

    def _publish_odom(self, t_now, vx: float, wz: float) -> None:
        o = Odometry()
        o.header.stamp = t_now.to_msg()
        o.header.frame_id = self._frame_odom
        o.child_frame_id = self._frame_base

        o.pose.pose.position.x = self._x
        o.pose.pose.position.y = self._y
        o.pose.pose.orientation.w = math.cos(self._yaw * 0.5)
        o.pose.pose.orientation.z = math.sin(self._yaw * 0.5)

        o.twist.twist.linear.x = vx
        o.twist.twist.angular.z = wz

        # 6x6 row-major covariance; diagonal index = 7 * i for i in 0..5
        # order: x, y, z, roll, pitch, yaw  /  vx, vy, vz, vroll, vpitch, vyaw
        o.pose.covariance[0]   = SIGMA_XY      ** 2   # x
        o.pose.covariance[7]   = SIGMA_XY      ** 2   # y
        o.pose.covariance[14]  = SIGMA_UNUSED  ** 2   # z
        o.pose.covariance[21]  = SIGMA_UNUSED  ** 2   # roll
        o.pose.covariance[28]  = SIGMA_UNUSED  ** 2   # pitch
        o.pose.covariance[35]  = SIGMA_YAW     ** 2   # yaw

        o.twist.covariance[0]  = SIGMA_VX      ** 2   # vx
        o.twist.covariance[7]  = SIGMA_UNUSED  ** 2   # vy  (no sideways motion)
        o.twist.covariance[14] = SIGMA_UNUSED  ** 2   # vz
        o.twist.covariance[21] = SIGMA_UNUSED  ** 2   # vroll
        o.twist.covariance[28] = SIGMA_UNUSED  ** 2   # vpitch
        o.twist.covariance[35] = SIGMA_VYAW    ** 2   # vyaw

        self._pub_odom.publish(o)

        if self._publish_tf:
            tf = TransformStamped()
            tf.header = o.header
            tf.child_frame_id = self._frame_base
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation = o.pose.pose.orientation
            self._tf_pub.sendTransform(tf)

    # ── SERVICES ──────────────────────────────────────────────────────

    # 1. Enable / Shutdown motor
    def _srv_enable_motor(self, req: SetBool.Request, res: SetBool.Response):
        cw = CW_ENABLE if req.data else CW_SHUTDOWN
        label = "ENABLE" if req.data else "SHUTDOWN"
        ok = self._write_control_word(cw)
        res.success = ok
        res.message = (
            f"Motor {label} OK (0x{cw:04X})" if ok
            else f"Motor {label} FAILED"
        )
        self.get_logger().info(res.message)
        return res

    # 2. Emergency stop
    def _srv_emergency_stop(self, _req, res: Trigger.Response):
        ok = self._write_control_word(CW_EMERGENCY_STOP)
        res.success = ok
        res.message = (
            "EMERGENCY STOP triggered (0x0005) — call enable_motor to resume"
            if ok else "EMERGENCY STOP FAILED"
        )
        self.get_logger().warn(res.message)
        return res

    # 3. Clear fault
    def _srv_clear_fault(self, _req, res: Trigger.Response):
        ok = self._write_control_word(CW_CLEAR_FAULT)
        res.success = ok
        res.message = "Fault cleared (0x0006)" if ok else "Clear fault FAILED"
        self.get_logger().info(res.message)
        return res

    # 4. Reset software odometry
    def _srv_reset_odometry(self, _req, res: Trigger.Response):
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        # NOTE: don't touch _prev_l / _prev_r — we still want dead-reckoning
        # to continue smoothly from the current encoder value, just from
        # a re-zeroed pose frame.
        res.success = True
        res.message = "Software odometry reset (x=y=yaw=0)"
        self.get_logger().info(res.message)
        return res

    # 5. Read status word (0x20A2)
    def _srv_get_status(self, _req, res: Trigger.Response):
        regs = self._read_registers(REG_STATUS_WORD, 1)
        if regs is None:
            res.success = False
            res.message = "Failed to read status word"
            return res
        sw = regs[0]

        # left = high byte, right = low byte
        left_hi  = (sw >> 8) & 0xFF   # bits 15..8
        right_lo = sw & 0xFF          # bits 7..0

        def shaft_state(byte_val: int) -> str:
            top2 = byte_val & 0xC0
            if   top2 == 0x00: return "release"
            elif top2 == 0x40: return "lock"
            elif top2 == 0x80: return "e-stop"
            else:              return "alarm"

        l_running = "running" if (left_hi & 0x01) else "stopped"
        r_running = "running" if (right_lo & 0x01) else "stopped"
        l_shaft   = shaft_state(left_hi)
        r_shaft   = shaft_state(right_lo)

        res.success = True
        res.message = (
            f"raw=0x{sw:04X}  |  "
            f"L: {l_running}, shaft {l_shaft}  |  "
            f"R: {r_running}, shaft {r_shaft}"
        )
        self.get_logger().info(res.message)
        return res

    # 6. Read fault code (0x20A5 left, 0x20A6 right)
    def _srv_get_fault_code(self, _req, res: Trigger.Response):
        regs = self._read_registers(REG_FAULT_L, 2)  # read both contiguously
        if regs is None:
            res.success = False
            res.message = "Failed to read fault code"
            return res
        code_L, code_R = regs[0], regs[1]
        res.success = True
        res.message = (
            f"L: 0x{code_L:04X} ({_decode_fault(code_L)}) | "
            f"R: 0x{code_R:04X} ({_decode_fault(code_R)})"
        )
        self.get_logger().info(res.message)
        return res

    # 7. Read temperatures
    def _srv_get_temperature(self, _req, res: Trigger.Response):
        # motor temperature: 0x20A4 — high 8 bits = left, low 8 bits = right, unit 1°C signed
        # driver temperature: 0x20B0 — signed I16, unit 0.1°C
        regs = self._read_registers(REG_MOTOR_TEMP, 1)
        if regs is None:
            res.success = False
            res.message = "Failed to read motor temp"
            return res
        motor_word = regs[0]
        t_L = (motor_word >> 8) & 0xFF
        t_R = motor_word & 0xFF
        # convert signed 8-bit
        if t_L & 0x80: t_L -= 256
        if t_R & 0x80: t_R -= 256

        drv_regs = self._read_registers(REG_DRIVER_TEMP, 1)
        if drv_regs is None:
            drv_c = float("nan")
        else:
            drv_c = _u16_to_s16(drv_regs[0]) * 0.1

        res.success = True
        res.message = (
            f"Motor L: {t_L}°C, R: {t_R}°C  |  Driver: {drv_c:.1f}°C"
        )
        self.get_logger().info(res.message)
        return res

    # 8. Parking mode on/off
    def _srv_set_parking_mode(self, req: SetBool.Request, res: SetBool.Response):
        val = 0x0001 if req.data else 0x0000
        ok = self._write_register(REG_PARKING_MODE, val)
        res.success = ok
        res.message = (
            f"Parking mode {'ON' if req.data else 'OFF'}" if ok
            else "Set parking mode FAILED"
        )
        self.get_logger().info(res.message)
        return res

    # 9. Reset hardware encoder counters (both wheels)
    def _srv_reset_encoder(self, _req, res: Trigger.Response):
        # write 0x0003 → clear feedback position for both L and R
        ok = self._write_register(REG_RESET_FB_POS, 0x0003)
        if ok:
            # force re-init so we don't compute a huge jump next odom cycle
            self._enc_init = False
        res.success = ok
        res.message = (
            "Hardware encoder counters reset (L+R)" if ok
            else "Reset encoder FAILED"
        )
        self.get_logger().info(res.message)
        return res

    # 10. Toggle max speed (high / conservative)
    def _srv_set_max_speed(self, req: SetBool.Request, res: SetBool.Response):
        # true  → use the parameter max_rpm (full speed)
        # false → use conservative 25% (safer for docking / crowded areas)
        self._max_rpm = self._max_rpm_high if req.data else self._max_rpm_low
        res.success = True
        res.message = (
            f"Max RPM set to {self._max_rpm} "
            f"({'HIGH' if req.data else 'LOW'})"
        )
        self.get_logger().info(res.message)
        return res

    # ── lifecycle ─────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        try:
            self._client.write_registers(
                address=REG_RPM_START, values=[0, 0], slave=self._slave
            )
        except Exception:
            pass
        try:
            self._client.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZLACNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
