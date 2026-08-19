/*****************************************************************************************
  Batly R1 - RP2040 micro-ROS peripheral node
  Arduino Nano RP2040 Connect (LSM6DSOX 6-DoF IMU, WS2812B status LEDs,
  HC-SR04 ultrasonic proximity, alert servo)

  Publishes:  imu/data           (sensor_msgs/Imu,  50 Hz)
              ultrasonic_alert   (std_msgs/Bool,    on change)
  Subscribes: cmd_vel            (geometry_msgs/Twist)

  ---------------------------------------------------------------------------------------
  Design notes
  ---------------------------------------------------------------------------------------
  Timestamps.  header.stamp is taken from rmw_uros_epoch_nanos(), not millis().
    millis() reports board uptime, whereas robot_localization compares incoming
    stamps against its own filter time on the Unix epoch and discards measurements
    that appear older than the last one processed. rmw_uros_sync_session() is
    therefore a prerequisite for fusion.

  Message fields.  micro-ROS does not allocate string fields automatically, so
    header.frame_id is assigned explicitly. angular_velocity is converted from
    deg/s to rad/s per REP-103, and orientation is taken from the Madgwick filter.
    Covariance diagonals are populated: an all-zero covariance is interpreted by
    ROS as a perfectly confident measurement rather than as unknown.

  Loop timing.  LED refresh is non-blocking, because WS2812B transmission disables
    interrupts and competes with the USB serial transport. The HC-SR04 pulseIn()
    timeout is 5 ms, which covers roughly 85 cm - well beyond the 45 cm alert
    threshold - while keeping blocking time inside the executor loop small.

  ---------------------------------------------------------------------------------------
  Platform-dependent constants
  ---------------------------------------------------------------------------------------
  The following are specific to one physical unit and are re-measured for each
  build; procedures are documented at the point of use below.

    - IMU axis remap (remapAxes) depends on board mounting orientation.
      REP-103 requires x-forward, y-left, z-up.
    - IMU covariance diagonals are derived from a static recording.
    - Gyroscope zero-rate offsets vary with temperature between sessions.
*****************************************************************************************/

#include <FastLED.h>
#include <Arduino_LSM6DSOX.h>
#include <MadgwickAHRS.h>
#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <hardware/watchdog.h>
#include <math.h>
#include <string.h>

// ======================= SERVO (RP2040_ISR) ========================
#if ( defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && !defined(ARDUINO_ARCH_MBED)
  #if !defined(RP2040_ISR_SERVO_USING_MBED)
    #define RP2040_ISR_SERVO_USING_MBED     false
  #endif
#elif ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && defined(ARDUINO_ARCH_MBED)
  #if !defined(RP2040_ISR_SERVO_USING_MBED)
    #define RP2040_ISR_SERVO_USING_MBED     true
  #endif
#endif

#include "RP2040_ISR_Servo.h"

#define SERVO_PIN_A0      A0
#define MIN_MICROS        800
#define MAX_MICROS        2450
#define MIN_POS           60
#define MAX_POS           120
#define CENTER_POS        90

int servoIndex = -1;
int current_servo_pos = CENTER_POS;
bool servo_direction_up = true;
unsigned long last_servo_ms = 0;
const int servo_speed_ms = 8;

// ======================= FASTLED ========================
#define DATA_PIN    17
#define NUM_LEDS    6
#define BRIGHTNESS  150
#define LED_REFRESH_MS 33          // ~30 Hz, refreshed non-blocking
CRGB leds[NUM_LEDS];
unsigned long last_led_ms = 0;

// ======================= ULTRASONIC =====================
#define TRIG_PIN 10
#define ECHO_PIN 9
#define ECHO_TIMEOUT_US 5000       // ~85 cm, well past the alert threshold
#define ALERT_DISTANCE_CM 45
unsigned long last_ultrasonic_ms = 0;

// ======================= IMU CONFIG =====================
#define IMU_FRAME_ID   "imu_link"  // must match the TF frame published on the Pi
#define IMU_RATE_HZ    50
const unsigned long imu_interval = 1000 / IMU_RATE_HZ;

// The Madgwick filter requires time to converge from its initial attitude.
// Until it has, orientation carries no information and its covariance is
// inflated accordingly.
#define MADGWICK_WARMUP_SAMPLES (IMU_RATE_HZ * 5)   // 5 seconds

// Measurement covariances. Obtained from a 60 s static recording of /imu/data
// on this unit (per-channel variance of angular_velocity and
// linear_acceleration); re-measure when the sensor or mounting changes.
const double ORI_RP_VAR    = 2.0e-4;   // rad^2, roll & pitch from Madgwick
const double ORI_YAW_VAR   = 1.0e6;    // yaw is gyro-only and unreferenced (no
                                       // magnetometer on LSM6DSOX) -> effectively
                                       // no information. ekf.yaml also disables it.
const double GYRO_VAR      = 3.7e-3;   // (rad/s)^2
const double ACCEL_VAR     = 4.0e-3;   // (m/s^2)^2
const double ORI_WARMUP_VAR = 1.0e6;   // used until Madgwick has converged

// IMU axis remap: sensor axes -> REP-103 body axes (x forward, y left, z up).
// Identity for the mounting orientation used here. Verified by tilting the
// chassis nose-down (pitch negative), rolling right (roll positive), and
// rotating counter-clockwise viewed from above (angular_velocity.z positive).
static inline void remapAxes(float sx, float sy, float sz,
                             float &bx, float &by, float &bz) {
  bx =  sx;
  by =  sy;
  bz =  sz;
}

Madgwick filter;
unsigned long last_imu_ms = 0;
uint32_t madgwick_samples = 0;

// ======================= SYSTEM STATE ===================
enum State { WAITING_AGENT, AGENT_CONNECTED };
State current_state = WAITING_AGENT;
bool alert_mode = false;
bool is_moving = false;

// ======================= micro-ROS ======================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t pub_imu;
rcl_publisher_t pub_alert;
rcl_subscription_t sub_cmd_vel;
geometry_msgs__msg__Twist msg_cmd_vel;
std_msgs__msg__Bool msg_alert;
rclc_executor_t executor;
sensor_msgs__msg__Imu msg_imu;

static char imu_frame_id_buf[] = IMU_FRAME_ID;

// Receives velocity commands from ROS 2 (teleop or Nav2). Used only to drive the
// status LED colour; this node does not control the drive motors.
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  is_moving = (fabs(msg->linear.x) > 0.01 || fabs(msg->angular.z) > 0.01);
}

// =======================================================
// IMU MESSAGE SETUP
// =======================================================
void initImuMessage() {
  // micro-ROS does not allocate string fields; assign a static buffer.
  msg_imu.header.frame_id.data     = imu_frame_id_buf;
  msg_imu.header.frame_id.size     = strlen(imu_frame_id_buf);
  msg_imu.header.frame_id.capacity = sizeof(imu_frame_id_buf);

  for (int i = 0; i < 9; i++) {
    msg_imu.orientation_covariance[i]         = 0.0;
    msg_imu.angular_velocity_covariance[i]    = 0.0;
    msg_imu.linear_acceleration_covariance[i] = 0.0;
  }
  msg_imu.angular_velocity_covariance[0] = GYRO_VAR;
  msg_imu.angular_velocity_covariance[4] = GYRO_VAR;
  msg_imu.angular_velocity_covariance[8] = GYRO_VAR;

  msg_imu.linear_acceleration_covariance[0] = ACCEL_VAR;
  msg_imu.linear_acceleration_covariance[4] = ACCEL_VAR;
  msg_imu.linear_acceleration_covariance[8] = ACCEL_VAR;

  msg_imu.orientation.w = 1.0;   // valid identity, not the all-zero default
  msg_imu.orientation.x = 0.0;
  msg_imu.orientation.y = 0.0;
  msg_imu.orientation.z = 0.0;
}

// Stamp a message with synchronised ROS time. Returns false if the session has
// not been synchronised, in which case the caller should not publish: an
// unsynchronised stamp will be silently discarded downstream anyway.
bool stampNow(builtin_interfaces__msg__Time &stamp) {
  if (!rmw_uros_epoch_synchronized()) return false;
  int64_t t = rmw_uros_epoch_nanos();
  stamp.sec     = (int32_t)(t / 1000000000LL);
  stamp.nanosec = (uint32_t)(t % 1000000000LL);
  return true;
}

// =======================================================
// SERVO TASK
// =======================================================
void updateServoTask() {
  if (servoIndex == -1) return;

  if (alert_mode) {
    // Alert: sweep continuously between the end stops.
    if (millis() - last_servo_ms >= (unsigned long)servo_speed_ms) {
      last_servo_ms = millis();
      if (servo_direction_up) {
        current_servo_pos += 2;
        if (current_servo_pos >= MAX_POS) {
          current_servo_pos = MAX_POS;
          servo_direction_up = false;
        }
      } else {
        current_servo_pos -= 2;
        if (current_servo_pos <= MIN_POS) {
          current_servo_pos = MIN_POS;
          servo_direction_up = true;
        }
      }
      RP2040_ISR_Servos.setPosition(servoIndex, current_servo_pos);
    }
  } else {
    // Normal: return to centre and hold.
    if (current_servo_pos != CENTER_POS) {
      if (millis() - last_servo_ms >= 15) {
        last_servo_ms = millis();
        if (current_servo_pos < CENTER_POS) current_servo_pos++;
        else if (current_servo_pos > CENTER_POS) current_servo_pos--;
        RP2040_ISR_Servos.setPosition(servoIndex, current_servo_pos);
      }
    }
  }
}

// =======================================================
// STARTUP ANIMATION
// =======================================================
void startupAnimation() {
  int left_out[]  = {2, 1, 0};
  int right_out[] = {3, 4, 5};
  int left_in[]   = {0, 1, 2};
  int right_in[]  = {5, 4, 3};

  FastLED.clear();
  for (int i = 0; i < 3; i++) {
    leds[left_out[i]]  = CHSV(150, 80, 90);
    leds[right_out[i]] = CHSV(150, 80, 90);
    FastLED.show();
    watchdog_update();                 // animation is ~1.1 s; watchdog is 3 s
    delay(120);
  }
  FastLED.clear();
  for (int i = 0; i < 3; i++) {
    leds[left_in[i]]  = CHSV(150, 80, 90);
    leds[right_in[i]] = CHSV(150, 80, 90);
    FastLED.show();
    watchdog_update();
    delay(120);
  }
  FastLED.clear();
  int speed = 160;
  for (int i = 0; i < 3; i++) {
    leds[left_in[i]]  = CHSV(150, 100, 120);
    leds[right_in[i]] = CHSV(150, 100, 120);
    FastLED.show();
    watchdog_update();
    delay(speed);
    speed = max(speed - 35, 45);
  }
  FastLED.clear(true);
  FastLED.show();
}

// =======================================================
// LED TASK  (rate-limited)
// =======================================================
void updateLEDTask() {
  if (millis() - last_led_ms < LED_REFRESH_MS) return;
  last_led_ms = millis();

  static uint8_t brightness = 0;

  if (current_state == WAITING_AGENT) {
    if (brightness < 200) brightness++;
    fill_solid(leds, NUM_LEDS, CHSV(150, 70, brightness));
  }
  else {
    if (alert_mode) {
      static bool blink = false;
      static unsigned long last_blink = 0;
      if (millis() - last_blink > 300) {
        last_blink = millis();
        blink = !blink;
      }
      fill_solid(leds, NUM_LEDS, blink ? CRGB::Red : CRGB::Black);
    }
    else if (is_moving) {
      fill_solid(leds, NUM_LEDS, CHSV(150, 180, 200));
    }
    else {
      fill_solid(leds, NUM_LEDS, CHSV(110, 140, 200));
    }
  }
  FastLED.show();
}

// =======================================================
// IMU TASK
// =======================================================
void updateIMUTask() {
  if (current_state != AGENT_CONNECTED) return;
  if (millis() - last_imu_ms < imu_interval) return;
  last_imu_ms = millis();

  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

  float sax, say, saz, sgx, sgy, sgz;
  IMU.readAcceleration(sax, say, saz);   // g
  IMU.readGyroscope(sgx, sgy, sgz);      // deg/s

  float ax, ay, az, gx, gy, gz;
  remapAxes(sax, say, saz, ax, ay, az);
  remapAxes(sgx, sgy, sgz, gx, gy, gz);

  // Gyroscope zero-rate offsets (deg/s), measured on this unit.
  // These drift with temperature between sessions; on the unit used here the
  // yaw offset ranged from +0.245 to +1.162 deg/s. Procedure: park the robot
  // with motors powered for 2 min, record angular_velocity.z from /imu/data,
  // and take the mean (x 57.2958 for deg/s). Where per-session calibration is
  // impractical, index 11 of imu0_config in ekf.yaml is set false and the
  // filter runs on wheel odometry alone.

  gx -= -0.048f;
  gy -= -0.471f;
  gz -= +0.245f;

  // Madgwick expects gyro in deg/s and accelerometer in g, which is exactly
  // what the Arduino_LSM6DSOX API returns. No conversion here.
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  if (madgwick_samples < MADGWICK_WARMUP_SAMPLES) madgwick_samples++;
  bool converged = (madgwick_samples >= MADGWICK_WARMUP_SAMPLES);

  // Madgwick returns Euler angles in degrees.
  float roll  = filter.getRoll()  * DEG_TO_RAD;
  float pitch = filter.getPitch() * DEG_TO_RAD;
  float yaw   = filter.getYaw()   * DEG_TO_RAD;

  // RPY (ZYX intrinsic) -> quaternion
  float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);

  msg_imu.orientation.w = cr * cp * cy + sr * sp * sy;
  msg_imu.orientation.x = sr * cp * cy - cr * sp * sy;
  msg_imu.orientation.y = cr * sp * cy + sr * cp * sy;
  msg_imu.orientation.z = cr * cp * sy - sr * sp * cy;

  double rp_var = converged ? ORI_RP_VAR : ORI_WARMUP_VAR;
  msg_imu.orientation_covariance[0] = rp_var;        // roll
  msg_imu.orientation_covariance[4] = rp_var;        // pitch
  msg_imu.orientation_covariance[8] = ORI_YAW_VAR;   // yaw: no absolute reference

  msg_imu.angular_velocity.x = gx * DEG_TO_RAD;
  msg_imu.angular_velocity.y = gy * DEG_TO_RAD;
  msg_imu.angular_velocity.z = gz * DEG_TO_RAD;

  msg_imu.linear_acceleration.x = ax * 9.80665;
  msg_imu.linear_acceleration.y = ay * 9.80665;
  msg_imu.linear_acceleration.z = az * 9.80665;

  if (!stampNow(msg_imu.header.stamp)) return;   // unsynchronised: do not publish
  rcl_publish(&pub_imu, &msg_imu, NULL);
}

// =======================================================
// ULTRASONIC TASK
// =======================================================
void updateUltrasonicTask() {
  if (current_state != AGENT_CONNECTED) return;
  if (millis() - last_ultrasonic_ms < 80) return;
  last_ultrasonic_ms = millis();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long d = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  int cm = (d > 0) ? (int)(d * 0.034 / 2) : 999;   // 999 = no echo / out of range

  bool prev_alert = alert_mode;
  alert_mode = (cm > 0 && cm < ALERT_DISTANCE_CM);

  if (alert_mode != prev_alert) {
    msg_alert.data = alert_mode;
    rcl_publish(&pub_alert, &msg_alert, NULL);
  }
}

// =======================================================
// micro-ROS ENTITIES AND RECONNECTION
// =======================================================
bool create_entities() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "rp2040_robot_node", "", &support) != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(&pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&pub_alert, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "ultrasonic_alert") != RCL_RET_OK) return false;
  if (rclc_subscription_init_default(&sub_cmd_vel, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) return false;

  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &sub_cmd_vel, &msg_cmd_vel,
        &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  // Synchronise this session's clock with the agent. Without this, timestamps
  // are board uptime and robot_localization discards every message as stale.
  // Must be re-done on every reconnect, hence its placement here.
  rmw_uros_sync_session(1000);

  initImuMessage();
  madgwick_samples = 0;   // restart the Madgwick warm-up on reconnect

  return true;
}

void destroy_entities() {
  rcl_publisher_fini(&pub_imu, &node);
  rcl_publisher_fini(&pub_alert, &node);
  rcl_subscription_fini(&sub_cmd_vel, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void updateMicroROSTask() {
  static unsigned long last_check = 0;
  if (millis() - last_check < 500) return;
  last_check = millis();

  if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) {
    if (current_state == WAITING_AGENT) {
      if (create_entities()) current_state = AGENT_CONNECTED;
      else destroy_entities();
    }
  } else if (current_state == AGENT_CONNECTED) {
    watchdog_reboot(0, 0, 0);   // hard reset is the simplest safe recovery path
  }
}

// =======================================================
// SETUP / LOOP
// =======================================================
void setup() {
  watchdog_enable(3000, 1);

  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  servoIndex = RP2040_ISR_Servos.setupServo(SERVO_PIN_A0, MIN_MICROS, MAX_MICROS);
  if (servoIndex != -1) {
    RP2040_ISR_Servos.setPosition(servoIndex, CENTER_POS);
  }

  if (!IMU.begin()) {
    // Hold a solid red pattern so an IMU failure is visible on the robot
    // rather than silently publishing nothing.
    for (;;) {
      watchdog_update();
      fill_solid(leds, NUM_LEDS, CRGB::Red);
      FastLED.show();
      delay(200);
      fill_solid(leds, NUM_LEDS, CRGB::Black);
      FastLED.show();
      delay(200);
    }
  }
  filter.begin(IMU_RATE_HZ);

  initImuMessage();

  set_microros_transports();
  startupAnimation();
}

void loop() {
  watchdog_update();
  updateMicroROSTask();

  if (current_state == AGENT_CONNECTED) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    updateIMUTask();
    updateUltrasonicTask();
  }

  updateServoTask();
  updateLEDTask();
}
