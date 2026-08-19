#!/usr/bin/env python3
"""
amr_timing/timing_node.py

Per-cycle transaction sequence — measures velocity feedback (not encoder
position). Mirrors what paper Section 4.1 describes:

  1. writeMultiple (FC=0x10) → REG_RPM_START (0x2088), values [rpmL, rpmR]
  2. readHolding   (FC=0x03) → REG_VEL_FEEDBACK (0x20AB), qty=2
                                 (reads left + right velocity in ONE frame
                                  because 0x20AB and 0x20AC are adjacent)

Design notes:
  - Velocity feedback (0x20AB, 0x20AC) is read rather than encoder position,
    in a single transaction (qty=2), since the two registers are adjacent
  - A cycle is counted as missed when its period exceeds 1.5 x T
  - auto_run_all sweeps the full matrix {0, 200} RPM x {20, 50, 100, 200} Hz
    (8 runs), so the idle and loaded conditions are captured in one launch
  - Per-cycle CPU utilisation (system-wide and for this process) is logged via
    psutil using a non-blocking read, so that instrumentation does not perturb
    the timing being measured; the columns fall back to NaN if psutil is
    unavailable

Per ZLAC8015D RS485 manual v1.04:
  0x20AB = Actual velocity (Left),  I16, RO, unit: 0.1 r/min
  0x20AC = Actual velocity (Right), I16, RO, unit: 0.1 r/min
"""

from __future__ import annotations

import csv
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

try:
    import psutil
    _HAVE_PSUTIL = True
except ImportError:
    psutil = None
    _HAVE_PSUTIL = False


TOTAL_CYCLES = 10_000
SUPPORTED_HZ = [20, 50, 100, 200]      # control frequencies under test (Hz)
TEST_RPMS    = [0, 200]                # idle (0 RPM) and loaded (200 RPM)
MISSED_THRESHOLD = 1.5  # paper change: callbacks > 1.5×T counted as missed

# Match zlac_node.cpp + ZLAC8015D datasheet
REG_ENABLE       = 0x200E
VAL_ENABLE       = 0x0008
REG_RPM_START    = 0x2088   # Target velocity (Left); Right is at 0x2089
REG_VEL_FEEDBACK = 0x20AB   # Actual velocity (Left); Right is at 0x20AC


# ── Minimal Modbus RTU helper (mirrors modbus_rtu.cpp) ────────────────────────

class _ModbusRTU:
    """Python mirror of batly_bringup::ModbusRTU for timing measurement."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.20):
        import serial
        self._ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        time.sleep(0.05)
        self._ser.reset_input_buffer()

    def close(self) -> None:
        try:
            self._ser.close()
        except Exception:
            pass

    @staticmethod
    def _crc16(data: bytes) -> int:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    @staticmethod
    def _with_crc(payload: bytes) -> bytes:
        crc = _ModbusRTU._crc16(payload)
        return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])

    # ── FC 0x06 ──────────────────────────────────────────────────
    def write_single(self, slave: int, reg: int, value: int) -> float:
        value = value & 0xFFFF
        payload = bytes([
            slave, 0x06,
            (reg >> 8) & 0xFF, reg & 0xFF,
            (value >> 8) & 0xFF, value & 0xFF,
        ])
        frame = self._with_crc(payload)

        self._ser.reset_input_buffer()
        t0 = time.perf_counter()
        self._ser.write(frame)
        rx = self._ser.read(8)
        t1 = time.perf_counter()

        if len(rx) < 8:
            raise RuntimeError(f"RS485 timeout (write_single): got {len(rx)}")
        if self._crc16(rx) != 0:
            raise RuntimeError("CRC error (write_single)")
        return (t1 - t0) * 1e3

    # ── FC 0x10 ──────────────────────────────────────────────────
    def write_multiple(self, slave: int, start_reg: int,
                       values: list[int]) -> float:
        qty = len(values)
        bc  = qty * 2
        payload = bytes([
            slave, 0x10,
            (start_reg >> 8) & 0xFF, start_reg & 0xFF,
            (qty >> 8) & 0xFF,       qty & 0xFF,
            bc,
        ])
        for v in values:
            v = v & 0xFFFF
            payload += bytes([(v >> 8) & 0xFF, v & 0xFF])
        frame = self._with_crc(payload)

        self._ser.reset_input_buffer()
        t0 = time.perf_counter()
        self._ser.write(frame)
        rx = self._ser.read(8)
        t1 = time.perf_counter()

        if len(rx) < 8:
            raise RuntimeError(f"RS485 timeout (write_multiple): got {len(rx)}")
        if self._crc16(rx) != 0:
            raise RuntimeError("CRC error (write_multiple)")
        return (t1 - t0) * 1e3

    # ── FC 0x03 ──────────────────────────────────────────────────
    def read_holding(self, slave: int, start_reg: int, qty: int
                     ) -> tuple[float, list[int]]:
        """Returns (round_trip_ms, [signed_int16_values])."""
        payload = bytes([
            slave, 0x03,
            (start_reg >> 8) & 0xFF, start_reg & 0xFF,
            (qty >> 8) & 0xFF, qty & 0xFF,
        ])
        frame = self._with_crc(payload)
        expected_len = 5 + 2 * qty

        self._ser.reset_input_buffer()
        t0 = time.perf_counter()
        self._ser.write(frame)
        rx = self._ser.read(expected_len)
        t1 = time.perf_counter()

        if len(rx) < expected_len:
            raise RuntimeError(
                f"RS485 timeout (read_holding): got {len(rx)}/{expected_len}"
            )
        if self._crc16(rx) != 0:
            raise RuntimeError("CRC error (read_holding)")
        if rx[1] == (0x03 | 0x80):
            raise RuntimeError(f"Modbus exception 0x{rx[2]:02X}")
        if rx[2] != 2 * qty:
            raise RuntimeError(f"Unexpected byte count {rx[2]}")

        # decode as signed int16 (velocity values are I16)
        regs = []
        for i in range(qty):
            hi = rx[3 + 2*i]
            lo = rx[3 + 2*i + 1]
            raw = (hi << 8) | lo
            if raw >= 0x8000:
                raw -= 0x10000
            regs.append(raw)

        return (t1 - t0) * 1e3, regs


# ── Main node ─────────────────────────────────────────────────────────────────

class TimingNode(Node):

    def __init__(self) -> None:
        super().__init__("amr_timing_node")

        # ── parameters ────────────────────────────────────────────────
        self.declare_parameter("frequency_hz",  50)
        self.declare_parameter("output_dir",    "/tmp/timing_results")
        self.declare_parameter("auto_run_all",  False)
        self.declare_parameter("mode",          "rs485")

        self.declare_parameter("port",          "/dev/rs485")
        self.declare_parameter("baudrate",      115200)
        self.declare_parameter("slave_id",      1)
        self.declare_parameter("mb_timeout_ms", 150)

        # test_rpm:
        #   0   = idle test (motors enabled, RPM=0, no rotation)
        #   20  = loaded test (wheels actually turn at 20 RPM)
        self.declare_parameter("test_rpm",      0)

        # tag added to output filename so idle/loaded files don't collide
        # auto-set to "idle" or "rpm20" based on test_rpm
        self.declare_parameter("run_tag",       "")

        self.add_on_set_parameters_callback(self._on_param_change)

        self._mode = self.get_parameter("mode").get_parameter_value().string_value
        self._out_dir = Path(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )
        self._out_dir.mkdir(parents=True, exist_ok=True)
        self._test_rpm = int(
            self.get_parameter("test_rpm").get_parameter_value().integer_value
        )

        tag = self.get_parameter("run_tag").get_parameter_value().string_value
        self._run_tag_override = tag   # non-empty → used in single-run mode
        if not tag:
            tag = "idle" if self._test_rpm == 0 else f"rpm{abs(self._test_rpm)}"
        self._run_tag = tag

        # ── RS485 setup ───────────────────────────────────────────────
        self._mb: _ModbusRTU | None = None
        self._slave = 1
        if self._mode == "rs485":
            self._setup_rs485()

        # ── state ─────────────────────────────────────────────────────
        self._hz     = 0
        self._timer  = None
        self._active = False
        self._queue: list[tuple[int, int]] = []   # (rpm, hz) pairs

        self._t_prev: float = 0.0
        self._loop_periods:   list[float] = []
        self._write_times:    list[float] = []
        self._read_times:     list[float] = []
        self._modbus_totals:  list[float] = []
        self._callback_times: list[float] = []
        self._vel_L: list[float] = []   # actual feedback velocity (RPM)
        self._vel_R: list[float] = []
        self._cpu_sys:  list[float] = []   # system-wide CPU % since last cycle
        self._cpu_proc: list[float] = []   # this process CPU % since last cycle

        # psutil process handle (None if psutil missing → CPU logged as NaN)
        self._proc = psutil.Process() if _HAVE_PSUTIL else None
        if not _HAVE_PSUTIL:
            self.get_logger().warn(
                "psutil not installed — CPU utilisation will be NaN. "
                "Install with: pip install psutil (or apt install python3-psutil)"
            )

        self._n_fail = 0
        self._first_err_logged = False
        self._consecutive_fails = 0

        self._status_pub = self.create_publisher(String, "~/status", 10)

        auto = self.get_parameter("auto_run_all").get_parameter_value().bool_value
        self._auto = bool(auto)
        if auto:
            # full matrix: idle frequencies first, then loaded frequencies
            self._queue = [(rpm, hz) for rpm in TEST_RPMS for hz in SUPPORTED_HZ]
            self.get_logger().info(
                f"AUTO sweep: {len(self._queue)} runs = "
                f"{TEST_RPMS} RPM × {SUPPORTED_HZ} Hz"
            )
            self._start_next()
        else:
            hz = int(
                self.get_parameter("frequency_hz").get_parameter_value().integer_value
            )
            self._start_run(self._test_rpm, hz)

    # ── RS485 init + motor enable ─────────────────────────────────────

    def _setup_rs485(self) -> None:
        port    = self.get_parameter("port").get_parameter_value().string_value
        baud    = self.get_parameter("baudrate").get_parameter_value().integer_value
        timeout = (
            self.get_parameter("mb_timeout_ms").get_parameter_value().integer_value
            / 1000.0
        )
        try:
            self._mb = _ModbusRTU(port, baud, timeout)
            self._slave = int(
                self.get_parameter("slave_id").get_parameter_value().integer_value
            )
            self.get_logger().info(
                f"RS485 open: {port} @ {baud} baud, slave={self._slave}"
            )
        except Exception as e:
            self.get_logger().error(
                f"RS485 open FAILED ({e}) — falling back to timer_only mode"
            )
            self._mode = "timer_only"
            self._mb   = None
            return

        self._enable_motor(retries=5)

    def _enable_motor(self, retries: int = 5) -> bool:
        if self._mb is None:
            return False
        for i in range(retries):
            try:
                self._mb.write_single(self._slave, REG_ENABLE, VAL_ENABLE)
                self.get_logger().info(
                    f"Motor ENABLE OK (reg=0x{REG_ENABLE:04X} "
                    f"val=0x{VAL_ENABLE:04X})"
                )
                return True
            except Exception as e:
                self.get_logger().warn(
                    f"Motor ENABLE retry {i+1}/{retries}: {e}"
                )
                time.sleep(0.2)
        self.get_logger().error("Motor ENABLE FAILED")
        return False

    def _on_param_change(self, params):
        for p in params:
            if p.name == "output_dir":
                self._out_dir = Path(p.value)
                self._out_dir.mkdir(parents=True, exist_ok=True)
        return SetParametersResult(successful=True)

    # ── run control ───────────────────────────────────────────────────

    def _start_next(self) -> None:
        if self._queue:
            rpm, hz = self._queue.pop(0)
            self._start_run(rpm, hz)
        else:
            self.get_logger().info("All runs complete.")
            self._pub_status("ALL_DONE")
            rclpy.shutdown()

    def _start_run(self, rpm: int, hz: int) -> None:
        if self._timer:
            self._timer.cancel()

        self._test_rpm       = int(rpm)
        # auto-derive tag; honour an explicit run_tag only in single-run mode
        if self._run_tag_override and not self._auto:
            self._run_tag = self._run_tag_override
        else:
            self._run_tag = "idle" if rpm == 0 else f"rpm{abs(rpm)}"
        self._hz             = hz
        self._period         = 1.0 / hz
        self._loop_periods   = []
        self._write_times    = []
        self._read_times     = []
        self._modbus_totals  = []
        self._callback_times = []
        self._vel_L          = []
        self._vel_R          = []
        self._cpu_sys        = []
        self._cpu_proc       = []
        self._n_fail         = 0
        self._t_prev         = 0.0
        self._active         = True
        self._first_err_logged = False
        self._consecutive_fails = 0

        # prime CPU counters so the first real reading is a valid delta
        # (psutil's first cpu_percent(interval=None) call returns 0.0)
        if _HAVE_PSUTIL:
            try:
                psutil.cpu_percent(interval=None)
                self._proc.cpu_percent(interval=None)
            except Exception:
                pass

        if self._mode == "rs485":
            self._enable_motor(retries=3)

        self.get_logger().info(
            f"▶ {hz} Hz | mode={self._mode} | test_rpm={self._test_rpm} | "
            f"tag={self._run_tag} | "
            f"{TOTAL_CYCLES} cycles (~{TOTAL_CYCLES/hz:.0f} s)"
        )
        self._pub_status(
            f"START {hz}Hz mode={self._mode} tag={self._run_tag}"
        )

        self._timer = self.create_timer(self._period, self._callback)

    def _stop_run(self) -> None:
        self._active = False
        if self._timer:
            self._timer.cancel()
            self._timer = None
        # safety: always stop wheels at end of run
        if self._mode == "rs485" and self._mb is not None:
            try:
                self._mb.write_multiple(self._slave, REG_RPM_START, [0, 0])
            except Exception:
                pass

    # ── callback (write + read velocity) ──────────────────────────────

    def _callback(self) -> None:
        if not self._active:
            return

        t_cb_start = time.perf_counter()

        # 1) loop period
        if self._t_prev > 0.0:
            self._loop_periods.append((t_cb_start - self._t_prev) * 1e3)
        self._t_prev = t_cb_start

        # 2) Modbus: write RPM + read velocity feedback (single transaction qty=2)
        w_ms     = float("nan")
        r_ms     = float("nan")
        vL_rpm   = float("nan")
        vR_rpm   = float("nan")
        cycle_failed = False

        if self._mode == "rs485" and self._mb is not None:
            try:
                # (a) write RPM setpoints
                w_ms = self._mb.write_multiple(
                    self._slave, REG_RPM_START,
                    [self._test_rpm, self._test_rpm],
                )
                # (b) read velocity feedback (both wheels in one read, qty=2)
                r_ms, regs = self._mb.read_holding(
                    self._slave, REG_VEL_FEEDBACK, 2
                )
                # regs[0]=left, regs[1]=right, unit: 0.1 r/min → convert to RPM
                vL_rpm = regs[0] * 0.1
                vR_rpm = regs[1] * 0.1

                self._consecutive_fails = 0
            except Exception as e:
                cycle_failed = True
                self._n_fail += 1
                self._consecutive_fails += 1
                if not self._first_err_logged:
                    self.get_logger().warn(f"RS485 transaction error: {e}")
                    self._first_err_logged = True
                if self._consecutive_fails == 5:
                    self.get_logger().warn(
                        "5 consecutive failures — re-enabling motor"
                    )
                    try:
                        self._mb.write_single(
                            self._slave, REG_ENABLE, VAL_ENABLE
                        )
                        self._consecutive_fails = 0
                    except Exception:
                        pass

        self._write_times.append(w_ms)
        self._read_times.append(r_ms)
        self._vel_L.append(vL_rpm)
        self._vel_R.append(vR_rpm)

        if not cycle_failed and not np.isnan(w_ms):
            self._modbus_totals.append(w_ms + r_ms)
        else:
            self._modbus_totals.append(float("nan"))

        # 3) total callback time (measured BEFORE the CPU read below, so the
        #    cheap /proc read does not inflate callback_total)
        self._callback_times.append((time.perf_counter() - t_cb_start) * 1e3)

        # 4) CPU utilisation — non-blocking (interval=None reads /proc deltas
        #    since the previous cycle; ~microseconds, does not perturb timing)
        if _HAVE_PSUTIL:
            try:
                self._cpu_sys.append(psutil.cpu_percent(interval=None))
                self._cpu_proc.append(self._proc.cpu_percent(interval=None))
            except Exception:
                self._cpu_sys.append(float("nan"))
                self._cpu_proc.append(float("nan"))
        else:
            self._cpu_sys.append(float("nan"))
            self._cpu_proc.append(float("nan"))

        n = len(self._callback_times)
        if n % (TOTAL_CYCLES // 10) == 0:
            mean_vL = np.nanmean(self._vel_L[-100:]) if self._vel_L else 0.0
            mean_vR = np.nanmean(self._vel_R[-100:]) if self._vel_R else 0.0
            self.get_logger().info(
                f"  [{self._hz} Hz] {n}/{TOTAL_CYCLES} "
                f"({100*n//TOTAL_CYCLES}%)  fail={self._n_fail}  "
                f"vel_fb≈[L={mean_vL:+.1f}, R={mean_vR:+.1f}] RPM"
            )

        if n >= TOTAL_CYCLES:
            self._stop_run()
            self._process_results()

    # ── results ───────────────────────────────────────────────────────

    def _process_results(self) -> None:
        hz       = self._hz
        expected = self._period * 1e3

        lp   = np.array(self._loop_periods)
        wr   = np.array(self._write_times[1:])
        rd   = np.array(self._read_times[1:])
        mtot = np.array(self._modbus_totals[1:])
        cbt  = np.array(self._callback_times[1:])
        vL   = np.array(self._vel_L[1:])
        vR   = np.array(self._vel_R[1:])
        cpu_sys  = np.array(self._cpu_sys[1:])
        cpu_proc = np.array(self._cpu_proc[1:])

        jitter = lp - expected

        def stats(arr: np.ndarray, label: str) -> dict:
            v = arr[~np.isnan(arr)]
            if len(v) == 0:
                return {"label": label, "n": 0,
                        "mean": float("nan"), "std": float("nan"),
                        "min": float("nan"), "p50": float("nan"),
                        "p95": float("nan"), "p99": float("nan"),
                        "max": float("nan")}
            return {
                "label": label,
                "n":     int(len(v)),
                "mean":  float(np.mean(v)),
                "std":   float(np.std(v)),
                "min":   float(np.min(v)),
                "p50":   float(np.percentile(v, 50)),
                "p95":   float(np.percentile(v, 95)),
                "p99":   float(np.percentile(v, 99)),
                "max":   float(np.max(v)),
            }

        s_jit  = stats(jitter, "loop_jitter")
        s_wr   = stats(wr,     "modbus_write")
        s_rd   = stats(rd,     "modbus_read_vel")
        s_tot  = stats(mtot,   "modbus_total")
        s_cb   = stats(cbt,    "callback_total")
        s_vL   = stats(vL,     "feedback_vel_L")
        s_vR   = stats(vR,     "feedback_vel_R")
        s_cpus = stats(cpu_sys,  "cpu_system_pct")
        s_cpup = stats(cpu_proc, "cpu_process_pct")

        # changed threshold from 2× to 1.5×
        missed = int(np.sum(lp > MISSED_THRESHOLD * expected))

        self.get_logger().info("=" * 70)
        self.get_logger().info(
            f"  RESULTS  {hz} Hz  |  T = {expected:.3f} ms  |  "
            f"tag={self._run_tag}  |  test_rpm={self._test_rpm}"
        )
        self.get_logger().info("=" * 70)
        for s in (s_jit, s_wr, s_rd, s_tot, s_cb):
            self.get_logger().info(
                f"  [{s['label']:18s}]  "
                f"mean={s['mean']:+8.4f}  std={s['std']:7.4f}  "
                f"p99={s['p99']:+8.4f}  max={s['max']:+8.4f}  ms"
            )
        self.get_logger().info(
            f"  [feedback velocity]   "
            f"L: mean={s_vL['mean']:+.2f} std={s_vL['std']:.2f}  "
            f"R: mean={s_vR['mean']:+.2f} std={s_vR['std']:.2f} RPM"
        )
        self.get_logger().info(
            f"  [CPU utilisation]     "
            f"system: mean={s_cpus['mean']:.1f}% p99={s_cpus['p99']:.1f}%  "
            f"process: mean={s_cpup['mean']:.1f}% p99={s_cpup['p99']:.1f}%"
        )
        self.get_logger().info(
            f"  missed (>{MISSED_THRESHOLD}×T): {missed}  |  "
            f"RS485 errors: {self._n_fail}"
        )
        self.get_logger().info("=" * 70)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._export_csv(lp, wr, rd, mtot, cbt, vL, vR,
                         cpu_sys, cpu_proc, jitter, expected, stamp)
        self._export_figures(
            jitter, wr, rd, mtot, cbt, vL, vR, cpu_sys, cpu_proc, expected,
            s_jit, s_wr, s_rd, s_tot, s_cb, s_vL, s_vR, s_cpus, s_cpup,
            missed, stamp,
        )
        self._pub_status(
            f"DONE {hz}Hz tag={self._run_tag} "
            f"jitter_p99={s_jit['p99']:+.4f} "
            f"mb_total_mean={s_tot['mean']:.4f} missed={missed}"
        )

        if self._queue:
            self._start_next()

    # ── CSV ──────────────────────────────────────────────────────────

    def _export_csv(self, lp, wr, rd, mtot, cbt, vL, vR,
                    cpu_sys, cpu_proc, jitter, expected_ms, stamp):
        path = self._out_dir / (
            f"timing_{self._hz}hz_{self._run_tag}_{stamp}.csv"
        )
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "cycle", "loop_period_ms", "jitter_ms",
                "modbus_write_ms", "modbus_read_vel_ms",
                "modbus_total_ms", "callback_total_ms",
                "feedback_vel_L_rpm", "feedback_vel_R_rpm",
                "cpu_sys_pct", "cpu_proc_pct",
                "expected_ms",
            ])

            def fmt(v):
                return f"{v:.6f}" if not np.isnan(v) else ""

            for i in range(len(lp)):
                w.writerow([
                    i + 1,
                    f"{lp[i]:.6f}",
                    f"{jitter[i]:.6f}",
                    fmt(wr[i]),
                    fmt(rd[i]),
                    fmt(mtot[i]),
                    f"{cbt[i]:.6f}",
                    fmt(vL[i]),
                    fmt(vR[i]),
                    fmt(cpu_sys[i]),
                    fmt(cpu_proc[i]),
                    f"{expected_ms:.6f}",
                ])
        self.get_logger().info(f"CSV → {path}")

    # ── figures ──────────────────────────────────────────────────────

    def _export_figures(self, jitter, wr, rd, mtot, cbt, vL, vR,
                         cpu_sys, cpu_proc,
                         expected_ms,
                         s_jit, s_wr, s_rd, s_tot, s_cb, s_vL, s_vR,
                         s_cpus, s_cpup,
                         missed, stamp):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            import matplotlib.gridspec as gridspec
            import matplotlib.ticker as mticker
        except ImportError:
            self.get_logger().warn("matplotlib missing")
            return

        hz = self._hz
        has_modbus = self._mode == "rs485" and not np.all(np.isnan(wr))

        fig = plt.figure(figsize=(12, 13))
        gs  = gridspec.GridSpec(
            5, 2, height_ratios=[2.6, 1.6, 1.6, 1.1, 1.0],
            hspace=0.55, wspace=0.30,
        )

        fig.suptitle(
            f"Control Loop Timing Characterisation — velocity feedback\n"
            f"Batly R1 · ZLTECH 8015D · Pi5 · ROS 2 Jazzy · "
            f"{hz} Hz · {TOTAL_CYCLES:,} cycles · "
            f"tag={self._run_tag} · test_rpm={self._test_rpm}",
            fontsize=10, y=0.995,
        )

        sigma_j = float(np.std(jitter))
        n_bins  = min(120, max(40, int(len(jitter)**0.5)))

        # ── (top) jitter histogram ────────────────────────────────
        ax = fig.add_subplot(gs[0, :])
        _, edges, patches = ax.hist(
            jitter, bins=n_bins, color="#1a6faf",
            edgecolor="none", alpha=0.85,
        )
        for patch, left in zip(patches, edges[:-1]):
            if abs(left) > 2 * sigma_j:
                patch.set_facecolor("#c0392b")
                patch.set_alpha(0.75)

        ax.axvline(0, color="#222", lw=0.8, ls="--", label="ideal (0 ms)")
        ax.axvline(s_jit["mean"], color="#e67e22", lw=1.3,
                   label=f"mean {s_jit['mean']:+.3f} ms")
        ax.axvline(s_jit["p99"], color="#8e44ad", lw=1.3, ls="-.",
                   label=f"p99  {s_jit['p99']:+.3f} ms")

        ax.set_xlabel("Loop period jitter (ms)", fontsize=10)
        ax.set_ylabel("Callbacks", fontsize=10)
        ax.legend(fontsize=9, framealpha=0.65)
        ax.grid(axis="y", lw=0.35, alpha=0.5)
        ax.xaxis.set_minor_locator(mticker.AutoMinorLocator())

        ax.text(
            0.985, 0.97,
            f"T_nom = {expected_ms:.3f} ms\n"
            f"n     = {s_jit['n']:,}\n"
            f"μ     = {s_jit['mean']:+.4f} ms\n"
            f"σ     = {sigma_j:.4f} ms\n"
            f"p95   = {s_jit['p95']:+.4f} ms\n"
            f"p99   = {s_jit['p99']:+.4f} ms\n"
            f"max   = {s_jit['max']:+.4f} ms\n"
            f"missed(>{MISSED_THRESHOLD}×T)= {missed}\n"
            f"cpu_sys  μ={s_cpus['mean']:.1f}% p99={s_cpus['p99']:.1f}%\n"
            f"cpu_proc μ={s_cpup['mean']:.1f}% p99={s_cpup['p99']:.1f}%",
            transform=ax.transAxes, fontsize=7.5,
            va="top", ha="right", fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.4", fc="white",
                      alpha=0.75, ec="#bbb"),
        )

        # ── Row 2: write | read_velocity ──────────────────────────
        ax_w  = fig.add_subplot(gs[1, 0])
        ax_rd = fig.add_subplot(gs[1, 1])
        # ── Row 3: modbus_total | velocity feedback time-series ───
        ax_tot = fig.add_subplot(gs[2, 0])
        ax_v   = fig.add_subplot(gs[2, 1])

        if has_modbus:
            for ax_x, arr, st, title, color in [
                (ax_w,   wr,   s_wr,
                 "Modbus write (FC 0x10, 2 reg)", "#27ae60"),
                (ax_rd,  rd,   s_rd,
                 "Modbus read velocity (FC 0x03, qty=2)", "#3498db"),
                (ax_tot, mtot, s_tot,
                 "Modbus TOTAL (write + read)", "#e67e22"),
            ]:
                v = arr[~np.isnan(arr)]
                if len(v) == 0:
                    ax_x.text(0.5, 0.5, "(no data)", ha="center",
                              va="center", transform=ax_x.transAxes)
                    continue
                ax_x.hist(v, bins=60, color=color, edgecolor="none", alpha=0.85)
                ax_x.axvline(st["mean"], color="#222", lw=1.0,
                             label=f"mean {st['mean']:.3f}")
                ax_x.axvline(st["p99"], color="#c0392b", lw=1.0, ls="-.",
                             label=f"p99  {st['p99']:.3f}")
                ax_x.set_title(title, fontsize=9)
                ax_x.set_xlabel("Duration (ms)", fontsize=9)
                ax_x.set_ylabel("Count", fontsize=9)
                ax_x.legend(fontsize=8, framealpha=0.6)
                ax_x.grid(axis="y", lw=0.35, alpha=0.5)

            # ── velocity feedback time series ─────────────────────
            ax_v.plot(vL, color="#3498db", lw=0.5, alpha=0.7,
                      label=f"L (μ={s_vL['mean']:.2f} RPM)")
            ax_v.plot(vR, color="#9b59b6", lw=0.5, alpha=0.7,
                      label=f"R (μ={s_vR['mean']:.2f} RPM)")
            ax_v.axhline(self._test_rpm, color="#e67e22", lw=0.8, ls="--",
                         label=f"commanded ({self._test_rpm} RPM)")
            ax_v.set_title("Velocity feedback over time", fontsize=9)
            ax_v.set_xlabel("Cycle index", fontsize=9)
            ax_v.set_ylabel("Velocity (RPM)", fontsize=9)
            ax_v.legend(fontsize=7, framealpha=0.6, loc="best")
            ax_v.grid(lw=0.3, alpha=0.4)
        else:
            for ax_x in (ax_w, ax_rd, ax_tot, ax_v):
                ax_x.text(0.5, 0.5, "timer_only mode\n(no RS485)",
                          ha="center", va="center",
                          transform=ax_x.transAxes, fontsize=10, color="#888")

        # ── Row 4: CPU utilisation time-series ────────────────────
        ax_cpu = fig.add_subplot(gs[3, :])
        if not np.all(np.isnan(cpu_sys)) or not np.all(np.isnan(cpu_proc)):
            ax_cpu.plot(cpu_sys, color="#16a085", lw=0.4, alpha=0.7,
                        label=f"system (μ={s_cpus['mean']:.1f}%, "
                              f"p99={s_cpus['p99']:.1f}%)")
            ax_cpu.plot(cpu_proc, color="#c0392b", lw=0.4, alpha=0.7,
                        label=f"this process (μ={s_cpup['mean']:.1f}%, "
                              f"p99={s_cpup['p99']:.1f}%)")
            ax_cpu.set_ylabel("CPU (%)", fontsize=9)
            ax_cpu.set_xlim(0, len(cpu_sys))
            ax_cpu.set_ylim(bottom=0)
            ax_cpu.legend(fontsize=7, framealpha=0.6, loc="best", ncol=2)
            ax_cpu.grid(lw=0.3, alpha=0.4)
        else:
            ax_cpu.text(0.5, 0.5, "CPU utilisation unavailable\n(psutil not installed)",
                        ha="center", va="center", transform=ax_cpu.transAxes,
                        fontsize=10, color="#888")
        ax_cpu.set_title("CPU utilisation over time", fontsize=9)
        ax_cpu.set_xlabel("Cycle index", fontsize=9)

        # ── Row 5: time-series jitter ─────────────────────────────
        ax_ts = fig.add_subplot(gs[4, :])
        ax_ts.plot(jitter, color="#1a6faf", lw=0.35, alpha=0.6)
        ax_ts.axhline(0, color="#222", lw=0.6, ls="--")
        ax_ts.set_xlabel("Cycle index", fontsize=9)
        ax_ts.set_ylabel("Jitter (ms)", fontsize=9)
        ax_ts.set_xlim(0, len(jitter))
        ax_ts.grid(lw=0.3, alpha=0.4)

        fig.subplots_adjust(
            top=0.94, bottom=0.05, left=0.07, right=0.97,
            hspace=0.60, wspace=0.30,
        )

        p1 = self._out_dir / (
            f"timing_{hz}hz_{self._run_tag}_{stamp}.png"
        )
        fig.savefig(p1, dpi=150, bbox_inches="tight")
        plt.close(fig)
        self.get_logger().info(f"Fig → {p1}")

    # ── helpers ──────────────────────────────────────────────────────

    def _pub_status(self, msg: str) -> None:
        m = String()
        m.data = msg
        self._status_pub.publish(m)

    def destroy_node(self) -> None:
        if self._mb:
            # safety: zero velocity before closing
            try:
                self._mb.write_multiple(self._slave, REG_RPM_START, [0, 0])
            except Exception:
                pass
            self._mb.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
