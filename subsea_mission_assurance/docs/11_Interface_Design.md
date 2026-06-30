# Interface Design Description (IDD)
_Generated 2026-06-07 from model/interfaces_detail.yaml._

## IF-MISIN — mission_input  (ros_topic)
- **Direction:** C-OPC -> C-CRAB (tasking)
- **Encoding:** std_msgs/String — mission token / tag id (text protocol)
- **Rate:** event (operator-driven)
- **QoS:** depth=50, RELIABLE
- **Units:** n/a (text)  ·  **Range:** valid tag ids / commands
- **Error handling:** unknown token logged and ignored; no crash, no actuation
- **Source:** `crab.py:105`

## IF-MISCMD — mission_cmd  (ros_topic)
- **Direction:** C-CRAB -> C-CTRL (active mission)
- **Encoding:** std_msgs/String — the dispatched mission for execution
- **Rate:** event (per dispatch)
- **QoS:** depth=10, RELIABLE
- **Units:** n/a (text)  ·  **Range:** single active mission token
- **Error handling:** controller validates before acting
- **Source:** `crab.py:104, controller.py:166`

## IF-MISSTAT — mission_status  (ros_topic)
- **Direction:** C-CTRL -> C-CRAB (+console) (status)
- **Encoding:** std_msgs/String — state machine status (SCANNING/LOCKING/HEADING/ACHIEVED/STUCK/HOVERING)
- **Rate:** on change, <= 5 Hz (STATUS_PERIOD 0.2 s)
- **QoS:** depth=10 (pub) / depth=50 (crab sub), RELIABLE
- **Units:** n/a (text)  ·  **Range:** defined state set
- **Error handling:** idempotent; latest status wins
- **Source:** `controller.py:83,173; crab.py:106`

## IF-CFG — robot_config  (ros_topic)
- **Direction:** C-CRAB -> C-CTRL (latched config)
- **Encoding:** std_msgs/String (JSON) — actuator map, modes, rates, nominal gait
- **Rate:** one-shot at startup (re-latched on change)
- **QoS:** TRANSIENT_LOCAL, depth=1 (latched)
- **Units:** mixed (config fields)  ·  **Range:** valid robot configuration
- **Error handling:** controller does not actuate until received (SYS-020)
- **Source:** `crab.py:101-103,124-129; controller.py:163-165,196`

## IF-JCMD — joint_cmd  (ros_topic)
- **Direction:** C-CTRL -> C-DXLIF (command)
- **Encoding:** std_msgs/Float32MultiArray — joint positions, actuator-map order
- **Rate:** 400 Hz produce [TBC]; applied to bus at 500 Hz hardware loop
- **QoS:** depth=1, RELIABLE (latest-only)
- **Units:** radians  ·  **Range:** per-joint mechanical [min,max], clamped (SYS-017)
- **Error handling:** out-of-range clamped; none published before config (SYS-020)
- **Source:** `controller.py:172,176; Dynamixel_XW430_T200_interface.py:97,100-101`

## IF-JFB — joint_feedback  (ros_topic)
- **Direction:** C-DXLIF -> C-CTRL (feedback)
- **Encoding:** std_msgs/Float32MultiArray — present joint positions
- **Rate:** 500 Hz hardware loop [TBC]
- **QoS:** depth=1, RELIABLE (latest-only)
- **Units:** radians  ·  **Range:** joint travel
- **Error handling:** controller tolerates dropped/late frames (IRD-06)
- **Source:** `Dynamixel_XW430_T200_interface.py:98; controller.py:167`

## IF-IMU — imu_data  (ros_topic)
- **Direction:** C-IMUIF -> C-CTRL (attitude)
- **Encoding:** sensor_msgs/Imu — accel + gyro (+ orientation)
- **Rate:** 100 Hz [TBC] (sample_rate)
- **QoS:** depth=10, RELIABLE
- **Units:** m/s^2, rad/s  ·  **Range:** sensor full-scale
- **Error handling:** loss of stream -> HOVERING fail-safe (SYS-016)
- **Source:** `icm20948_interface.py:44,63; controller.py:168`

## IF-TAG — apriltag_detections  (ros_topic)
- **Direction:** C-APRIL -> C-CTRL (range/bearing)
- **Encoding:** std_msgs/Float32MultiArray — [tag_id, range, bearing, ...] + stamp
- **Rate:** 30 Hz [TBC] (detect_rate, camera-bound)
- **QoS:** depth=10, RELIABLE
- **Units:** m, rad  ·  **Range:** >= 1.5 m detection (SYS-007)
- **Error handling:** detections older than 0.5 s ignored (SYS-019)
- **Source:** `apriltag_interface.py:58,77; controller.py:169,346`

## IF-IMG — camera/image_raw  (ros_topic)
- **Direction:** C-CAMIF -> C-APRIL (raw frames)
- **Encoding:** sensor_msgs/Image — raw frames for perception
- **Rate:** <= 15 Hz cap [TBC] (publish_rate)
- **QoS:** depth=10, RELIABLE
- **Units:** pixels  ·  **Range:** camera resolution
- **Error handling:** camera interface is sole frame owner; drops frames above cap
- **Source:** `stellarhd_interface.py:61,107,111; apriltag_interface.py:92`

## IF-TLM — telemetry  (ros_topic)
- **Direction:** C-CTRL -> C-OPC (telemetry)
- **Encoding:** std_msgs/Float32MultiArray — flattened telemetry vector
- **Rate:** loop-paced
- **QoS:** depth=10, RELIABLE
- **Units:** mixed  ·  **Range:** n/a
- **Error handling:** lossy ok (monitoring only)
- **Source:** `controller.py:174`

## IF-RS485 — servo bus (RS-485)  (physical)
- **Direction:** C-COMPUTE -> C-SERVO (actuation bus)
- **Encoding:** Dynamixel Protocol 2.0 frames (sync/bulk read+write)
- **Rate:** 1,000,000 baud; 500 Hz hardware loop
- **QoS:** n/a (physical)
- **Units:** protocol register values  ·  **Range:** 4 servo IDs
- **Error handling:** comm result checked per packet; failed IDs logged
- **Source:** `Dynamixel_XW430_T200_interface.py:67-68,85-88`

## IF-I2C — IMU bus (I2C)  (physical)
- **Direction:** C-COMPUTE -> C-IMU (sensor bus)
- **Encoding:** I2C register reads (accel/gyro/mag)
- **Rate:** 100 Hz sampling
- **QoS:** n/a (physical)
- **Units:** register values  ·  **Range:** addr 0x69
- **Error handling:** read failure -> skip sample / log
- **Source:** `icm20948_interface.py`

## IF-USB — camera (USB)  (physical)
- **Direction:** C-CAM -> C-COMPUTE (video in)
- **Encoding:** USB UVC video stream
- **Rate:** camera native; capped on republish
- **QoS:** n/a (physical)
- **Units:** frames  ·  **Range:** UVC modes
- **Error handling:** capture thread join on shutdown; reconnect on loss
- **Source:** `stellarhd_interface.py`
