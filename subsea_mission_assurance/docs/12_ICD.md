# Interface Control Document (ICD)
_Generated 2026-09-03. The controlled baseline: merges IRD + IDD per interface._

- **Baseline:** 0.1 (draft) (draft)
- **Control authority:** PI
- **Change policy:** A baselined interface may change only by editing model/*.yaml, re-running check.py (0 errors) and regenerating the ICD. Both the producer-side and consumer-side component owners review the diff before the new ICD is frozen.

## IF-MISIN — mission_input
**C-OPC (Operator console) → C-CRAB (crab (mission dispatcher))**  ·  ros_topic · `std_msgs/String`

- Direction: C-OPC -> C-CRAB (tasking)
- Encoding: std_msgs/String — mission token / tag id (text protocol)  ·  Rate: event (operator-driven)  ·  QoS: depth=50, RELIABLE
- Units/Range: n/a (text) / valid tag ids / commands
- Error handling: unknown token logged and ignored; no crash, no actuation
- Requirements:
    - **IRD-13** (error): Malformed mission_input messages shall be rejected without crashing the dispatcher and without commanding the propulsors.

## IF-MISCMD — mission_cmd
**C-CRAB (crab (mission dispatcher)) → C-CTRL (controller (execution engine))**  ·  ros_topic · `std_msgs/String`

- Direction: C-CRAB -> C-CTRL (active mission)
- Encoding: std_msgs/String — the dispatched mission for execution  ·  Rate: event (per dispatch)  ·  QoS: depth=10, RELIABLE
- Units/Range: n/a (text) / single active mission token
- Error handling: controller validates before acting
- Requirements:
    - **IRD-16** (ordering): mission_cmd shall carry exactly one active mission and the controller shall act only on the most recent message.

## IF-MISSTAT — mission_status
**C-CTRL (controller (execution engine)) → C-CRAB (crab (mission dispatcher))**  ·  ros_topic · `std_msgs/String`

- Direction: C-CTRL -> C-CRAB (+console) (status)
- Encoding: std_msgs/String — state machine status (SCANNING/LOCKING/HEADING/ACHIEVED/STUCK/HOVERING)  ·  Rate: on change, <= 5 Hz (STATUS_PERIOD 0.2 s)  ·  QoS: depth=10 (pub) / depth=50 (crab sub), RELIABLE
- Units/Range: n/a (text) / defined state set
- Error handling: idempotent; latest status wins
- Requirements:
    - **IRD-12** (timing): mission_status shall be published on each state change and rate-limited to a minimum spacing (default 5 Hz). _[TBC]_

## IF-CFG — robot_config
**C-CRAB (crab (mission dispatcher)) → C-CTRL (controller (execution engine))**  ·  ros_topic · `std_msgs/String (latched)`

- Direction: C-CRAB -> C-CTRL (latched config)
- Encoding: std_msgs/String (JSON) — actuator map, modes, rates, nominal gait  ·  Rate: one-shot at startup (re-latched on change)  ·  QoS: TRANSIENT_LOCAL, depth=1 (latched)
- Units/Range: mixed (config fields) / valid robot configuration
- Error handling: controller does not actuate until received (SYS-020)
- Requirements:
    - **IRD-11** (ordering): robot_config shall be published once on a latched (transient-local, depth 1) topic so any node that starts later still receives the configuration.

## IF-JCMD — joint_cmd
**C-CTRL (controller (execution engine)) → C-DXLIF (dynamixel interface)**  ·  ros_topic · `std_msgs/Float32MultiArray`

- Direction: C-CTRL -> C-DXLIF (command)
- Encoding: std_msgs/Float32MultiArray — joint positions, actuator-map order  ·  Rate: 200 Hz produce [TBC]; applied to bus at 100 Hz hardware loop  ·  QoS: depth=1, RELIABLE (latest-only)
- Units/Range: radians / per-joint mechanical [min,max], clamped (SYS-017)
- Error handling: out-of-range clamped; none published before config (SYS-020)
- Requirements:
    - **IRD-01** (timing): The controller shall publish a complete joint-position command vector at the configured control rate (default 200 Hz). _[TBC]_
    - **IRD-02** (data): The joint_cmd payload shall be a Float32 vector of joint positions in radians, ordered by the latched actuator map, one element per active joint.
    - **IRD-03** (error): Every joint_cmd element shall be clamped to the joint's configured mechanical min/max before it is published.
    - **IRD-04** (ordering): No joint_cmd shall be published before the latched robot_config has been received.

## IF-JFB — joint_feedback
**C-DXLIF (dynamixel interface) → C-CTRL (controller (execution engine))**  ·  ros_topic · `std_msgs/Float32MultiArray`

- Direction: C-DXLIF -> C-CTRL (feedback)
- Encoding: std_msgs/Float32MultiArray — present joint positions  ·  Rate: 100 Hz hardware loop [TBC]  ·  QoS: depth=1, RELIABLE (latest-only)
- Units/Range: radians / joint travel
- Error handling: controller tolerates dropped/late frames (IRD-06)
- Requirements:
    - **IRD-05** (timing): The Dynamixel interface shall publish joint_feedback at the hardware loop rate (default 100 Hz), latest-sample-only. _[TBC]_
    - **IRD-06** (error): The controller shall tolerate missing or late joint_feedback frames without commanding an uncommanded manoeuvre.

## IF-IMU — imu_data
**C-IMUIF (imu interface) → C-CTRL (controller (execution engine))**  ·  ros_topic · `sensor_msgs/Imu`

- Direction: C-IMUIF -> C-CTRL (attitude)
- Encoding: sensor_msgs/Imu — accel + gyro (+ orientation)  ·  Rate: 100 Hz [TBC] (sample_rate)  ·  QoS: depth=10, RELIABLE
- Units/Range: m/s^2, rad/s / sensor full-scale
- Error handling: loss of stream -> HOVERING fail-safe (SYS-016)
- Requirements:
    - **IRD-10** (timing): The IMU interface shall publish imu_data at the configured sample rate (default 100 Hz) as sensor_msgs/Imu. _[TBC]_

## IF-TAG — apriltag_detections
**C-APRIL (apriltag_interface (perception)) → C-CTRL (controller (execution engine))**  ·  ros_topic · `std_msgs/Float32MultiArray`

- Direction: C-APRIL -> C-CTRL (range/bearing)
- Encoding: std_msgs/Float32MultiArray — [tag_id, range, bearing, ...] + stamp  ·  Rate: 30 Hz [TBC] (detect_rate, camera-bound)  ·  QoS: depth=10, RELIABLE
- Units/Range: m, rad / >= 1.5 m detection (SYS-007)
- Error handling: detections older than 0.5 s ignored (SYS-019)
- Requirements:
    - **IRD-08** (timing): AprilTag detections shall be produced at the detector rate (default 30 Hz) and carry a timestamp. _[TBC]_
    - **IRD-09** (error): The controller shall ignore detections older than the configured staleness threshold (default 0.5 s). _[TBC]_

## IF-IMG — camera/image_raw
**C-CAMIF (stellarhd interface) → C-APRIL (apriltag_interface (perception))**  ·  ros_topic · `sensor_msgs/Image`

- Direction: C-CAMIF -> C-APRIL (raw frames)
- Encoding: sensor_msgs/Image — raw frames for perception  ·  Rate: <= 15 Hz cap [TBC] (publish_rate)  ·  QoS: depth=10, RELIABLE
- Units/Range: pixels / camera resolution
- Error handling: camera interface is sole frame owner; drops frames above cap
- Requirements:
    - **IRD-07** (timing): The camera interface shall republish frames at no more than the configured cap (default 15 Hz). _[TBC]_

## IF-TLM — telemetry
**C-CTRL (controller (execution engine)) → C-OPC (Operator console)**  ·  ros_topic · `std_msgs/Float32MultiArray`

- Direction: C-CTRL -> C-OPC (telemetry)
- Encoding: std_msgs/Float32MultiArray — flattened telemetry vector  ·  Rate: loop-paced  ·  QoS: depth=10, RELIABLE
- Units/Range: mixed / n/a
- Error handling: lossy ok (monitoring only)
- Requirements:
    - **IRD-17** (data): telemetry shall be a Float32 vector for monitoring only; its loss shall not affect vehicle control.

## IF-RS485 — servo bus (RS-485)
**C-COMPUTE (Jetson Orin Nano) → C-SERVO (Dynamixel XW430-T200 x4)**  ·  physical · `RS-485 / U2D2 1 Mbaud`

- Direction: C-COMPUTE -> C-SERVO (actuation bus)
- Encoding: Dynamixel Protocol 2.0 frames (sync/bulk read+write)  ·  Rate: 1,000,000 baud; 100 Hz hardware loop  ·  QoS: n/a (physical)
- Units/Range: protocol register values / 4 servo IDs
- Error handling: comm result checked per packet; failed IDs logged
- Requirements:
    - **IRD-14** (protocol): The servo bus shall run Dynamixel Protocol 2.0 over RS-485 at 1 Mbaud via the U2D2 adapter, addressing four servos.

## IF-I2C — IMU bus (I2C)
**C-COMPUTE (Jetson Orin Nano) → C-IMU (ICM-20948 IMU)**  ·  physical · `I2C 0x69`

- Direction: C-COMPUTE -> C-IMU (sensor bus)
- Encoding: I2C register reads (accel/gyro/mag)  ·  Rate: 100 Hz sampling  ·  QoS: n/a (physical)
- Units/Range: register values / addr 0x69
- Error handling: read failure -> skip sample / log
- Requirements:
    - **IRD-15** (protocol): The IMU shall be addressed on I2C at 0x69.

## IF-USB — camera (USB)
**C-CAM (StellarHD camera) → C-COMPUTE (Jetson Orin Nano)**  ·  physical · `USB UVC`

- Direction: C-CAM -> C-COMPUTE (video in)
- Encoding: USB UVC video stream  ·  Rate: camera native; capped on republish  ·  QoS: n/a (physical)
- Units/Range: frames / UVC modes
- Error handling: capture thread join on shutdown; reconnect on loss
- Requirements:
    - **IRD-18** (protocol): The camera shall stream over USB UVC and the camera interface shall recover the stream on disconnect without crashing.
