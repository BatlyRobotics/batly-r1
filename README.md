# Batly R1

ROS 2 driver, firmware, configuration and analysis code for the Batly R1
differential-drive autonomous mobile robot.

This repository accompanies the paper *A Python ROS 2 Driver and
Fiducial-Referenced Repeatability Characterisation for a Differential-Drive
Autonomous Mobile Robot Platform* (under review). It contains the code and
configuration that produced the reported results, not a cleaned-up
reimplementation of them — see **Known issues** below.

---

## Platform

| Subsystem | Component |
|---|---|
| Drive | 2 × ZLTECH ZLLG40ASM100-S brushless DC hub motors, differential, spring-loaded castors |
| Motor controller | ZLTECH ZLAC8015D V4.0, dual channel, RS485 Modbus RTU, 115200 baud 8N1 |
| Wheels | 107 mm diameter (53.5 mm rolling radius), 230 mm track width, 4096 counts/rev |
| Compute | Raspberry Pi 5, 8 GB, Ubuntu 24.04 LTS, ROS 2 Jazzy Jalisco |
| RS485 adapter | Waveshare USB TO RS485 (B), WCH CH343G, `cdc_acm`, via a USB 3.0 hub |
| Laser scanner | Slamtec RPLIDAR C1, 12 m, 0.72° at 10 Hz, CP210x, `/dev/rplidar` |
| Inertial | Arduino Nano RP2040 Connect, on-board LSM6DSOX, micro-ROS over USB serial |
| Downward camera | ELP USBGS1200P01 global shutter, 850 nm IR illumination and matched bandpass filter |
| Battery | LiFePO₄ 24 V 13 Ah (312 Wh) |

Chassis mass is 15 kg. The motor manufacturer rates the two-motor set for a 50 kg
total system mass; that is a drive rating and **not** a payload specification, since
the load capacity of the castors, the structural capacity of the frame and the effect
of payload centre-of-gravity height are all uncharacterised. Deck payloads up to
approximately 15 kg have been observed to operate smoothly. Wheel speed is capped at
200 RPM (1.12 m/s) because of slip observed at high payload and high speed.

---

## Layout

```
firmware/
  firmware.ino                RP2040 micro-ROS node: LSM6DSOX IMU, addressable
                              status LEDs, ultrasonic proximity, alert servo

src/
  batly_bringup_python/       ZLAC8015D RS485 Modbus driver (zlac_node), teleop,
                              bringup launch and the static transform tree
  batly_nav/                  Nav2 and SLAM Toolbox launch configuration
  batly_params/               parameters, maps, camera calibration, RViz configs
  batly_docking/              AprilTag-referenced dock pose publisher (C++)
  amr_timing/                 control loop timing characterisation node

analysis/
  analyse_repeatability.py    return-to-pose repeatability: statistics + JSON record
  plot_nav_repeatability.py   the same statistics as published figures + stats.md

figures/
  fig2_system_architecture.py system architecture diagram (Figure 2)
  fig3_driver_architecture.py driver architecture diagram (Figure 3)
```

### Not included

Deliberately, and stated as such in the paper:

- **Camera driver node.** Not released.
- **Fiducial characterisation and navigation trial acquisition scripts.** The
  procedures are described in full in the paper (§6.2, §7.1); documented
  implementations will accompany subsequent work on fiducial-referenced docking.
- **ZLTECH controller documentation.** Available from the manufacturer; not
  redistributed here.
- **Raw experimental logs.** Available from the corresponding author on request.

There is no URDF. The coordinate frame tree is defined by the static transform
publishers in `batly_bringup_python`, which is a more precise specification for
reproduction than a URDF would be.

---

## Build

The repository is a ROS 2 workspace overlay. `colcon` discovers packages
recursively, so either layout works:

```bash
# clone as the workspace
git clone https://github.com/BatlyRobotics/batly-r1.git batly_ws
cd batly_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

```bash
# or clone into an existing workspace
cd ~/ros2_ws/src && git clone https://github.com/BatlyRobotics/batly-r1.git
cd ~/ros2_ws && colcon build --symlink-install
```

One package at a time:

```bash
colcon build --packages-select amr_timing --symlink-install
```

**Firmware** is built with the Arduino toolchain for the Arduino Nano RP2040
Connect and requires `micro_ros_arduino` plus the libraries listed in the includes
at the top of `firmware.ino`. Flash it before starting the micro-ROS agent, which
`batly_bringup_python` launches on the device path configured there.

**Python dependencies** for the analysis and figure scripts: `numpy`,
`matplotlib`, and optionally `scipy` (without it, confidence intervals are omitted
and the drift p-value falls back to a normal approximation). `amr_timing`
additionally uses `psutil` for the CPU utilisation columns.

The analysis and figure scripts need neither ROS nor the robot.

---

## Reproducing the reported results

```bash
# Section 4: control loop timing. Stops the driver for the duration of the run,
# since both open the serial device exclusively.
ros2 run amr_timing timing_node --ros-args -p rate_hz:=50 -p test_rpm:=200

# Section 7: return-to-pose repeatability, from a trial log
python3 analysis/plot_nav_repeatability.py --csv trials.csv --out-dir figs --dpi 600
python3 analysis/analyse_repeatability.py trials.csv --run-tag n30 --outdir figs

# Figures 2 and 3
python3 figures/fig2_system_architecture.py -o figs
python3 figures/fig3_driver_architecture.py -o figs
```

`analyse_repeatability.py` writes a JSON record of every statistic so that
published numbers are traceable to a file. Both scripts apply two trial exclusions
and always print the reason: the reference marker must appear in the detector's own
list for that capture window, and a trial whose position exactly duplicates an
earlier one is a carried-forward stale value rather than an independent
measurement. No other outlier rejection is applied.

---

## Configuration in force for the reported results

| Where | Setting |
|---|---|
| `batly_bringup_python` | `cmd_rate_hz` 20, `odom_rate_hz` 50, `SIGMA_VX` 0.02, `SIGMA_VYAW` 0.061 |
| `batly_params/ekf.yaml` | `two_d_mode: true`, `imu0_differential: false`, `imu0_config` index 11 (vyaw) the only inertial channel enabled |
| `batly_params/nav.yaml` | `xy_goal_tolerance` 0.05 m, `yaw_goal_tolerance` 0.03 rad, `controller_frequency` 20 Hz, `desired_linear_vel` 0.25 m/s |
| `batly_docking` | `controller_frequency` 50 Hz |
| `firmware.ino` | `GYRO_VAR` 3.7e-3, yaw zero-rate offset applied as a fixed constant measured immediately before the session |

---

## Platform-dependent constants

These are specific to one physical unit and must be re-measured per build. The
procedure is documented at the point of use.

- **Gyroscope zero-rate offsets** (`firmware.ino`). Applied as fixed constants.
  They drift with temperature: the yaw offset on the development unit measured
  +1.162 °/s in one session and +0.245 °/s in another, and an uncorrected offset of
  that size integrates to roughly 70° of heading error per minute at rest. Record
  the stationary offset immediately before each session. Online estimation is
  identified as future work in the paper.
- **IMU axis remap and covariance diagonals** (`firmware.ino`). The remap is
  identity and has not been confirmed by a rotation test; verify it empirically
  before relying on it. The covariances are engineering estimates, not measured
  noise models.
- **Wheel rolling radius and track width** (`batly_bringup_python` parameters).
  The kinematics depend on the effective rolling radius, not the nominal wheel
  diameter.
- **Camera intrinsics and marker physical size** (`batly_params`). Marker size is
  measured to the outer edge of the black border, the convention the detector
  expects; a 1 mm error there produces a 5% distance scale error.

---

## Known issues

Present in this code because it is what produced the reported results. Each is
documented in the paper.

| Where | Issue |
|---|---|
| `batly_params/ekf.yaml`, `nav.yaml` | Several comments contradict their live values, carried over from earlier revisions. **The values are authoritative; do not trust the comments in these two files.** |
| `amr_timing` | Opens the serial device exclusively, so the driver must be stopped during a run. It reads the controller's actual-velocity register, whereas the deployed driver reads the position registers in two separate transactions. Both are disclosed in the paper (§4.3). |
| `batly_bringup_python` | The Modbus timeout parameter was stored but not applied in an earlier revision. Verify `mb_timeout_ms` reaches the transport before relying on it. |
| `batly_params` | May contain calibration files from an earlier stereo configuration that no longer exists. `right_camera.yaml`, if present, is not a valid calibration. |

Two behaviours are worth knowing because they fail silently rather than loudly, and
both cost significant debugging time during this work:

- **A zero measurement covariance is not "unknown" to `robot_localization`** — it is
  interpreted as perfect confidence, and it renders any co-fused sensor inert
  regardless of that sensor's own covariance.
- **micro-ROS must call `rmw_uros_sync_session()`** before publishing. Without it,
  messages carry board uptime rather than epoch time and the filter rejects every
  one of them without raising an error at the publisher.

---

## Citation

The accompanying paper is under review. Please cite the repository in the interim:

```
Batly R1: ROS 2 driver, firmware and configuration for a differential-drive
autonomous mobile robot. BatlyRobotics, 2026.
https://github.com/BatlyRobotics/batly-r1
```

---

## Licence

Apache License 2.0. See [LICENSE](LICENSE).
