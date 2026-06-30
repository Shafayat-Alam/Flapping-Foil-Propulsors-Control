# Interface Requirements Document (IRD)
_Generated 2026-06-07 from model/interfaces_detail.yaml._
_Baseline 0.1 (draft) (draft). `[TBC]` = derived from the ROS code, not yet ratified._

| ID | Interface | Category | Requirement | Value | TBC | Verifies via | Status |
|---|---|---|---|---|---|---|---|
| IRD-01 | IF-JCMD (joint_cmd) | timing | The controller shall publish a complete joint-position command vector at the configured control rate (default 400 Hz). | 400 Hz [TBC] (controller _control_loop timer) | yes | VER-017 | draft |
| IRD-02 | IF-JCMD (joint_cmd) | data | The joint_cmd payload shall be a Float32 vector of joint positions in radians, ordered by the latched actuator map, one element per active joint. | Float32MultiArray; rad; actuator-map order | — | VER-020 | draft |
| IRD-03 | IF-JCMD (joint_cmd) | error | Every joint_cmd element shall be clamped to the joint's configured mechanical min/max before it is published. | clamp to [min,max] per joint | — | VER-017 | draft |
| IRD-04 | IF-JCMD (joint_cmd) | ordering | No joint_cmd shall be published before the latched robot_config has been received. | config-before-actuation gate | — | VER-020 | draft |
| IRD-05 | IF-JFB (joint_feedback) | timing | The Dynamixel interface shall publish joint_feedback at the hardware loop rate (default 500 Hz), latest-sample-only. | 500 Hz [TBC], QoS depth=1 (Dynamixel hardware loop) | yes | — | draft |
| IRD-06 | IF-JFB (joint_feedback) | error | The controller shall tolerate missing or late joint_feedback frames without commanding an uncommanded manoeuvre. | graceful-degradation | — | VER-016 | draft |
| IRD-07 | IF-IMG (camera/image_raw) | timing | The camera interface shall republish frames at no more than the configured cap (default 15 Hz). | <= 15 Hz [TBC] (stellarhd publish_rate cap) | yes | — | draft |
| IRD-08 | IF-TAG (apriltag_detections) | timing | AprilTag detections shall be produced at the detector rate (default 30 Hz) and carry a timestamp. | 30 Hz [TBC] + per-message stamp (apriltag detect_rate) | yes | — | draft |
| IRD-09 | IF-TAG (apriltag_detections) | error | The controller shall ignore detections older than the configured staleness threshold (default 0.5 s). | 0.5 s [TBC] stale-reject window (controller stale check) | yes | VER-019 | draft |
| IRD-10 | IF-IMU (imu_data) | timing | The IMU interface shall publish imu_data at the configured sample rate (default 100 Hz) as sensor_msgs/Imu. | 100 Hz [TBC] (icm20948 sample_rate) | yes | — | draft |
| IRD-11 | IF-CFG (robot_config) | ordering | robot_config shall be published once on a latched (transient-local, depth 1) topic so any node that starts later still receives the configuration. | QoS TRANSIENT_LOCAL, depth=1 (latched) | — | VER-020 | draft |
| IRD-12 | IF-MISSTAT (mission_status) | timing | mission_status shall be published on each state change and rate-limited to a minimum spacing (default 5 Hz). | <= 5 Hz [TBC] (controller STATUS_PERIOD 0.2 s) | yes | VER-005 | draft |
| IRD-13 | IF-MISIN (mission_input) | error | Malformed mission_input messages shall be rejected without crashing the dispatcher and without commanding the propulsors. | reject-and-continue | — | VER-018 | draft |
| IRD-14 | IF-RS485 (servo bus (RS-485)) | protocol | The servo bus shall run Dynamixel Protocol 2.0 over RS-485 at 1 Mbaud via the U2D2 adapter, addressing four servos. | RS-485, 1,000,000 baud, Protocol 2.0, 4 IDs | — | VER-E05 | draft |
| IRD-15 | IF-I2C (IMU bus (I2C)) | protocol | The IMU shall be addressed on I2C at 0x69. | I2C addr 0x69 | — | — | draft |
| IRD-16 | IF-MISCMD (mission_cmd) | ordering | mission_cmd shall carry exactly one active mission and the controller shall act only on the most recent message. | single active mission; latest-wins | — | VER-011 | draft |
| IRD-17 | IF-TLM (telemetry) | data | telemetry shall be a Float32 vector for monitoring only; its loss shall not affect vehicle control. | Float32MultiArray; non-safety, lossy-tolerant | — | — | draft |
| IRD-18 | IF-USB (camera (USB)) | protocol | The camera shall stream over USB UVC and the camera interface shall recover the stream on disconnect without crashing. | USB UVC; reconnect-on-loss | — | VER-016 | draft |

## Rationale

- **IRD-01** (IF-JCMD): A 1 Hz flapping stroke must be well oversampled for smooth gait; the servo bus applies commands at 500 Hz.
- **IRD-02** (IF-JCMD): Deterministic element order is required so the right command reaches the right servo (SYS-020).
- **IRD-03** (IF-JCMD): Prevents the worst credible software mishap — hardware self-damage from an out-of-limit command (SYS-017).
- **IRD-04** (IF-JCMD): Actuating before the actuator map is known could drive the wrong joint (SYS-020).
- **IRD-05** (IF-JFB): The controller needs fresh joint state; only the latest sample matters, so depth=1.
- **IRD-06** (IF-JFB): A dropped feedback frame must not destabilise the loop (fail-safe HOVERING, SYS-016).
- **IRD-07** (IF-IMG): Bounds perception/compute load; the camera interface is the sole frame owner.
- **IRD-08** (IF-TAG): Range/bearing must be fresh and stamped so the consumer can reject stale data.
- **IRD-09** (IF-TAG): Acting on stale detections can drive erratic motion into pool walls/hardware (SYS-019).
- **IRD-10** (IF-IMU): Attitude/heading reference for the control loop.
- **IRD-11** (IF-CFG): The controller may start before/after the dispatcher; the config must survive that race (SYS-020).
- **IRD-12** (IF-MISSTAT): Operator visibility without flooding the console.
- **IRD-13** (IF-MISIN): Bad operator input must not produce undefined actuation (SYS-018).
- **IRD-14** (IF-RS485): Actuation bus integrity underpins all motion (ELE-005).
- **IRD-15** (IF-I2C): Deterministic bus addressing for the attitude sensor.
- **IRD-16** (IF-MISCMD): A new dispatch or priority override must cleanly supersede the running mission (SYS-011).
- **IRD-17** (IF-TLM): Console telemetry is observational; dropping it must never perturb the control loop.
- **IRD-18** (IF-USB): Camera dropout must degrade perception gracefully, not take down the node (fail-safe SYS-016).