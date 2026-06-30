# Soft Propulsors Control

A ROS 2 Jazzy control stack for a bio-inspired blue crab robotic system with multimodal soft actuator design. The robot is deployed in water, autonomously seeks AprilTags, and executes a queue of navigation missions using real-time IMU and camera feedback. The architecture cleanly separates **deliberation** (what to do) from **execution** (how to do it), with thin hardware interfaces underneath.

---

## Architecture

```
   bash feeder
      │ mission_input (String)
      ▼
┌──────────────────────┐   robot_config (latched)   ┌──────────────────────────┐
│        crab          │ ─────────────────────────► │       controller         │
│ Mission Dispatcher + │   mission_cmd  ──────────► │  Autonomous Execution    │
│  Configuration Master│ ◄──────────  mission_status│   Engine (state machine) │
└──────────────────────┘                            └──────────────────────────┘
                                                       │ joint_cmd     ▲ joint_feedback
                                                       │               │ imu_data
                                                       │               │ apriltag_detections
                                                       ▼               │
                                          ┌─────────────────────────────────────┐
                                          │  Hardware / Perception Interfaces   │
                                          │  Dynamixel · ICM20948 · AprilTag    │
                                          └─────────────────────────────────────┘
```

**Layering**
- **crab — deliberative layer.** Holds the robot configuration (single source of truth) and an unbounded mission queue. Dispatches one mission at a time, owns retry/human-escalation policy. Never touches sensors or servos.
- **controller — reactive layer.** Owns every real-time sensor (servo feedback, IMU, AprilTag). Runs a state machine that drives the robot toward the active mission and reports only interpreted status upward. Generates motion directly from the `motion_command` library.
- **hardware/perception — thin interfaces.** Publish sensor data, execute servo commands, detect tags. No mission logic.

**Key Design Features**
- **Deliberative / reactive split:** crab decides *what*, controller decides *how* — the boundary is the `mission_cmd` / `mission_status` pair.
- **Configuration master:** crab broadcasts the actuator map, operating mode, rates, and nominal gait once on a latched topic; all nodes build their structures from it.
- **Autonomous state machine:** `BOOT → WAIT → SCANNING → LOCKING → HEADING → STUCK → HOVERING`, driven entirely by sensor state.
- **Mission queue with overrides:** unbounded FIFO fed from a terminal/bash script; missions can queue, `discard`-preempt, or `requeue`-preempt the running mission.
- **Composable motion math:** `motion_command.py` is a flat library of pure functions — waveforms compose into `flap`/`paddle` gaits, with the waveform passed in as an argument.
- **Heading missions:** crab commands a compass heading (N..NW); the controller scans for the cardinal tag(s), then swims toward the heading (currently `flap` only — differential to turn, synchronous to cruise; `paddle` and an adaptive progress-rate gait optimizer come later). Stroke is set by per-mission `velocity` (rad/s) + `effort` (rad).

---

## Table of Contents

1. [Node Specifications](#node-specifications)
   - [Mission Dispatcher (crab.py)](#mission-dispatcher-crabpy)
   - [Execution Engine (controller.py)](#execution-engine-controllerpy)
   - [Motion Command Library (motion_command.py)](#motion-command-library-motion_commandpy)
   - [AprilTag Perception (apriltag_interface.py)](#apriltag-perception-apriltag_interfacepy)
   - [Hardware Interface (Dynamixel_XW430_T200_interface.py)](#hardware-interface)
   - [IMU Interface (icm20948_interface.py)](#imu-interface)
   - [Camera Interface (stellarhd_interface.py)](#camera-interface)
2. [ROS2 Topic Specifications](#ros2-topic-specifications)
3. [Mission Format & Feeding](#mission-format--feeding)

---

## Node Specifications

### Mission Dispatcher (`crab.py`)
**Deliberative layer — mission dispatcher + configuration master**

crab does two jobs and nothing else: it broadcasts the robot configuration once at startup, and it dispatches missions from an unbounded queue, owning the retry/human-escalation policy. It never reads a sensor or commands a servo.

**Responsibilities:**
- Broadcast `robot_config` once on a latched topic (actuator map, operating mode, rates, nominal gait)
- Buffer missions from `mission_input` in an unbounded FIFO queue
- Dispatch one mission at a time on `mission_cmd`; advance on `ACHIEVED`
- Apply override semantics: `none` (queue), `discard` (preempt + drop), `requeue` (preempt + save to front)
- On `STUCK`: auto-retry up to `max_retries`, then prompt the operator (10 s timeout → move on; "y" grants 2 more)
- Command `HOVER` when the queue drains

**Input ROS2 Topics:**
- `mission_input` (std_msgs/String): one mission line from a terminal/bash feeder
- `mission_status` (std_msgs/String): interpreted status/events from the controller

**Output ROS2 Topics:**
- `robot_config` (std_msgs/String, transient_local): one-shot full configuration
- `mission_cmd` (std_msgs/String): the active mission goal

**Parameters:**
- `actuator_map`: JSON `[[id, homing_offset, set_id, min, max, custom], ...]` — `homing_offset` (rad) is written to the servo by the hardware node; within a set the FIRST entry is roll and the SECOND is pitch; `custom` is an optional spare value (unused)
- `operating_mode`: 'position' or 'velocity'
- `control_rate`, `startup_delay`, `gait_velocity`, `gait_effort`, `default_retries`

---

### Execution Engine (`controller.py`)
**Reactive layer — autonomous state machine**

The controller is the robot's reactive brain. It owns every real-time sensor, runs a state machine toward the active mission, generates motion from the `motion_command` library, and reports only high-level status to crab.

**State machine:** `BOOT` (deployment-settling countdown) → `WAIT` → `SCANNING` (sweep one body axis at a time, searching) → `LOCKING` (confirm a stable detection) → `HEADING` (swim toward tag) → `STUCK` (zero progress → hover, let crab decide) → `HOVERING` (level, IMU-stabilised hold).

**Responsibilities:**
- Build servo/fin structure from `robot_config` (each set = a fin: roll + pitch servo)
- Wait `startup_delay` seconds after launch before any motion (deployment settling)
- Fuse IMU orientation + AprilTag distance/bearing into mission progress
- Resolve a heading mission to its cardinal tag(s), scan to acquire, then swim (currently `flap` only)
- Detect STUCK (no progress over the stuck window) and report it
- Compute progress = `0.7·(1 − dist/dist₀) + 0.3·(1 − |bearing|/bearing₀)`
- Optionally apply outer-loop PID; derive velocity from position in velocity mode

**Input ROS2 Topics:**
- `robot_config` (String, transient_local), `mission_cmd` (String)
- `joint_feedback` (Float32MultiArray), `imu_data` (sensor_msgs/Imu)
- `apriltag_detections` (Float32MultiArray)

**Output ROS2 Topics:**
- `joint_cmd` (Float32MultiArray): servo commands `[ids, modes, values]`
- `mission_status` (std_msgs/String): JSON status/events to crab
- `telemetry` (Float32MultiArray): commanded goals + feedback for logging

**Parameters:**
- `kp`, `ki`, `kd`: outer-loop PID gains (0.0 = passthrough)
- `control_rate`, `telemetry_decimation`, `gait_velocity`, `gait_effort`, `max_freq`, `startup_delay`

---

### Motion Command Library (`motion_command.py`)
**Pure motion math — no ROS, no state**

A flat module of stateless functions the controller composes in its real-time loop. Layered so larger motions are built from smaller ones:

- **Waveforms** `(t, freq, amp, phase) → float`: `sine`, `cosine`, `square`, `triangle`, `sawtooth`, `trapezoid`. The *shape* of a motion is just which one you pass in.
- **Servo targets** `→ {servo_id: value}`: `drive`, `drive_multi`, `hold`.
- **Gaits**: `flap(roll_id, pitch_id, …, waveform=sine)` holds roll broadside (π/2) and oscillates pitch around 0; `paddle(…)` is a rowing stroke with a power phase (broadside) and a graceful, feathered, low-drag recovery. The waveform is an argument, so `flap(sine)` and `flap(square)` are the same function, different feel.
- **Search helper**: `sweep(servo_id, t, rate, span)` — a slow one-axis ramp the controller composes into scanning.

---

### AprilTag Perception (`apriltag_interface.py`)
**Thin perception interface — camera → heading cues**

Detects AprilTags with OpenCV's ArUco/AprilTag detector, estimates each tag's pose via `solvePnP`, and publishes a compact detection array. The controller consumes only this interpreted output, never raw images. Owns the camera frames it processes (`source: 'camera'` for a local device, `source: 'topic'` for a Gazebo/replay image stream).

**Output ROS2 Topics:**
- `apriltag_detections` (Float32MultiArray): `[tag_id, distance_m, bearing_rad, elevation_rad, valid]` per tag (bearing > 0 = left, elevation > 0 = up). Empty array = no detection.

**Parameters:**
- `source` ('camera'/'topic'), `camera_index`, `image_topic`, `detect_rate`, `tag_family`
- `tag_size` and intrinsics `fx, fy, cx, cy` (set from a camera calibration for real distances)

---

### Hardware Interface
**Hardware Interface Node - Dynamixel Protocol 2.0**

Exclusive owner of the serial bus. Translates ROS2 commands into Dynamixel SDK protocol packets with synchronized writes to eliminate inter-servo latency.

**Responsibilities:**
- Configure servo operating modes, gains, and current limits
- Execute synchronized position/velocity writes via GroupSyncWrite
- Read encoder feedback via GroupSyncRead at 500 Hz
- Emergency torque disable on SIGINT (Ctrl+C)
- Manage hardware configuration and teardown

**Input ROS2 Topics:**
- `joint_cmd` (std_msgs/Float32MultiArray): Final servo commands

**Output ROS2 Topics:**
- `joint_feedback` (std_msgs/Float32MultiArray): Encoder position, velocity, current, voltage

**Parameters:**
- `port`: Serial port (default: '/dev/ttyUSB0')
- `baudrate`: Communication speed (default: 1000000)
- `hardware_rate`: Feedback publishing rate in Hz (default: 500.0)
- `current_limit`: Motor current limit in mA (default: 800)
- `servo_velocity_i_gain`, `servo_velocity_p_gain`: Velocity PID gains
- `servo_position_d_gain`, `servo_position_i_gain`, `servo_position_p_gain`: Position PID gains

**Key Features:**
- GroupSyncWrite for zero inter-servo latency
- GroupSyncRead for efficient multi-servo feedback
- One-time hardware configuration at startup
- SIGINT signal handler for instant torque disable
- Runs at 500 Hz for smooth feedback

---

**Hardware Interface Node - Adafruit ICM-20948 9-DOF IMU**

Continuously reads and publishes accelerometer, gyroscope, and magnetometer data from the ICM-20948 IMU sensor over I2C.

**Responsibilities:**
- Configure IMU operating mode and sample rate
- Read raw sensor data at specified frequency
- Convert to ROS2 sensor_msgs and publish on `imu_data` and `mag_data` topics
- Manage IMU configuration and teardown

**Output ROS2 Topics:**
- `imu_data` (sensor_msgs/Imu): Accelerometer (m/s^2) and gyroscope (rad/s) data
- `mag_data` (sensor_msgs/MagneticField): Magnetometer data (Tesla)

**Parameters:**
- `i2c_address`: I2C bus address (default: 0x69)
- `sample_rate`: IMU data sample rate in Hz (default: 100.0)
- `frame_id`: ROS2 TF frame name (default: 'imu_link')

**Key Features:**
- Publishes 9-DOF IMU data at configurable rate
- Transforms sensor data to ROS2 standard message types
- Manages low-level I2C communication with ICM-20948

---

**Hardware Interface Node - DWE StellarHD USB Camera**

Records video continuously and segments recordings based on robot command execution. Each command (from start to finish) is saved as a separate video file.

**Responsibilities:**
- Configure camera resolution, frame rate, and codec
- Capture frames continuously, republish them on `camera/image_raw` for perception
- Record video to disk segmented per mission (one file per mission label)
- Manage video file output and camera configuration

**Input ROS2 Topics:**
- `mission_cmd` (std_msgs/String): JSON mission; new label starts a new recording segment
- `mission_status` (std_msgs/String): JSON status; `ALL_MISSIONS_DONE` event stops recording

**Parameters:**
- `camera_index`: OpenCV camera index (default: 0)
- `video_width`, `video_height`: Camera resolution (default: 1920×1080)
- `fps`: Video frames per second (default: 30.0)
- `output_directory`: Directory to save recorded videos (default: `~/videos`)
- `fourcc`: OpenCV video codec (default: 'mp4v')

**Key Features:**
- Records high-resolution video synchronized with robot commands
- Segments recordings automatically based on command execution
- Configurable resolution, frame rate, and codec
- Separate capture thread for non-blocking recording

---

## ROS2 Topic Specifications

| ROS2 Topic | Type | Direction | Wire Format | Purpose |
|-------|------|-----------|-------------|---------|
| `mission_input` | String | bash → crab | `heading:NE velocity:6 effort:0.6 distance:0.1 override:none` | Enqueue / override missions |
| `robot_config` | String (latched) | crab → all | JSON config object | One-shot robot configuration |
| `mission_cmd` | String | crab → controller | JSON `{kind, label, max_retries, heading?, target_tag_id?, velocity?, effort?, distance?}` | Active mission goal |
| `mission_status` | String | controller → crab | JSON `{label, state, progress, orientation, event}` | Interpreted mission status |
| `manual_cmd` | String | bash → controller | `gait set:1 freq:1.0 amp:0.6` / `drive id:3 pos:0.5` / `stop` | Lab teleop — overrides missions while armed |
| `apriltag_detections` | Float32MultiArray | perception → controller | `[tag_id, dist, bearing, elev, valid]` per tag | Heading cues |
| `joint_cmd` | Float32MultiArray | controller → hardware | `[ids, modes, values]` | Final servo commands |
| `joint_feedback` | Float32MultiArray | hardware → controller | `[id, mode, pos, vel, curr, volt]` per servo | Encoder feedback |
| `imu_data` | Imu | IMU → controller | ROS2 standard message | Orientation, angular velocity, accel |
| `mag_data` | MagneticField | IMU → all | ROS2 standard message | Magnetic field strength |
| `telemetry` | Float32MultiArray | controller → logging | `[seq, sample, goal, pos, vel, curr, volt]` per servo | Logging / analysis |

### Topic Details

#### `robot_config` (latched, crab → all)
JSON broadcast once at startup; late subscribers still receive it (transient_local QoS):
```json
{
  "actuator_map": [[4, 0.0, 1, -3.14, 3.14], [3, 0.0, 1, -1.57, 1.57]],
  "cardinal_map": {"N": 0, "E": 1, "S": 2, "W": 3},
  "operating_mode": "position",
  "control_rate": 400.0,
  "startup_delay": 10.0,
  "gait_velocity": 3.77,
  "gait_effort": 0.6
}
```
Each set is one fin; within a set the first servo is roll and the second is pitch (positional convention — only the gaits care which is which). The hardware node reads `actuator_map` for the per-servo homing offsets; the controller reads `cardinal_map` to resolve heading missions to tag ids.

#### `mission_cmd` (crab → controller)
```json
{"kind": "heading", "heading": "NE", "label": "NE", "max_retries": 2,
 "velocity": 6.0, "effort": 0.6, "distance": 0.1}
```
`kind` is `heading` (swim a compass direction), `scan` (search only), `hover` (hold station), or `tag` (legacy single-tag seek). The controller owns sequencing: a heading mission scans for its cardinal tag(s) itself, then heads. ACHIEVED when facing the heading and within `distance` (m) of the reference tag.

#### `mission_status` (controller → crab)
```json
{"label": "NORTH", "target_tag_id": 3, "state": "HEADING",
 "progress": 42.5, "orientation": [roll, pitch, yaw], "event": "TAG_ACQUIRED"}
```
`state` is the current state-machine state; `event` is a one-shot transition marker (`MISSION_BEGIN`, `TAG_DETECTED`, `TAG_ACQUIRED`, `TAG_LOST`, `ACHIEVED`, `STUCK`, `ALL_MISSIONS_DONE`). Plain progress updates carry `event: null`.

#### `apriltag_detections` (perception → controller)
**Structure:** `[tag_id, distance_m, bearing_rad, elevation_rad, valid]` repeated per tag. `bearing > 0` = tag to the left, `elevation > 0` = above centre, `valid = 1.0`. An empty array means no tag in view.

#### `joint_cmd` Wire Format
**Structure:** `[id0, id1, ..., mode0, mode1, ..., val0, val1, ...]` — mode 3.0 = position, 1.0 = velocity. Final commands after offsets, limits, and any PID correction.

#### `joint_feedback` Wire Format
**Structure:** `[id, mode, pos_rad, vel_rad_s, curr_A, volt_V]` repeated per servo, at 500 Hz from the hardware interface.

#### `telemetry` Wire Format
**Structure:** `[seq, sample, goal0, pos0, vel0, curr0, volt0, goal1, ...]` — `seq` is the mission sequence number; per-servo commanded goal + actual feedback for logging.

---

## Mission Format & Feeding

Missions are fed at runtime as plain text lines on `/mission_input`:

A mission is one of four kinds (the controller owns all sequencing):

```
heading:<dir> velocity:<v> effort:<a> distance:<m> ...   # swim a compass heading
scan ...                                                 # sweep / search only
hover ...                                                # hold station
tag:<id> ...                                             # legacy single-tag seek
```

| Field | Required | Default | Meaning |
|-------|----------|---------|---------|
| `heading` | heading mission | — | one of `N NE E SE S SW W NW`; intercardinals steer to the bisector of two cardinal tags |
| `tag` | tag mission | — | target AprilTag id (legacy single-tag seek) |
| `distance` | no | `0.30` | arrival distance (m): ACHIEVED when facing the heading and within this of the reference tag |
| `velocity` | no | `gait_velocity` | peak stroke rate (rad/s) for this mission |
| `effort` | no | `gait_effort` | stroke amplitude (rad); controller derives `freq = velocity / (2π·effort)` |
| `label` | no | heading/kind | human-readable name (used to match status) |
| `retries` | no | 2 | auto-retries on STUCK before asking the operator |
| `override` | no | `none` | `none` = queue at back; `discard` = preempt + drop current; `requeue` = preempt + push current to front |

**Retry / escalation:** on `STUCK`, crab silently re-sends the mission up to `retries` times. Once exhausted it prompts on the terminal and waits 10 s — `y` grants 2 fresh retries, `n` or a timeout moves on to the next mission.

**Feeding missions** (run after the stack is launched):
```bash
# Built-in demo sequence (one mission every 3 s)
ros2 run soft_propulsors_control feed_missions.sh        # if installed
# or directly from source:
src/soft_propulsors_control/scripts/feed_missions.sh missions.txt 3

# Or push a single mission by hand:
ros2 topic pub --once /mission_input std_msgs/msg/String \
  "{data: 'heading:NE velocity:6 effort:0.6 distance:0.1 override:none'}"
```

When crab prompts for a stuck mission, answer on the terminal where crab is running (the `y`/`n` prompt has a 10 s timeout).

### Manual / bench teleop (`manual_cmd`)

A lab-only channel for poking servos directly. While a manual command is armed it **overrides the mission state machine** (one node, one bus), so use it when no mission is running. Raw freq/amp (rad); limits still apply. A deadman (`manual_timeout`, default 30 s, `:=0` to disable) holds neutral if commands stop arriving. crab must be up so the controller has the actuator map.

```bash
# Flap fin (set 1) at 1 Hz, 0.6 rad amplitude
ros2 topic pub --once /manual_cmd std_msgs/msg/String "{data: 'gait set:1 freq:1.0 amp:0.6'}"
# Sweep a single servo
ros2 topic pub --once /manual_cmd std_msgs/msg/String "{data: 'gait id:3 rate:0.2 span:1.0'}"
# Drive a servo to a static position
ros2 topic pub --once /manual_cmd std_msgs/msg/String "{data: 'drive id:3 pos:0.5'}"
# Release back to the mission flow
ros2 topic pub --once /manual_cmd std_msgs/msg/String "{data: 'stop'}"
```

---

## Motion Command Library (`motion_command.py`)

A flat module of pure, stateless functions — no ROS, no `self`, no node reference. The controller imports what it needs and composes these in its real-time loop.

### Layers

**Waveforms** — `(t, freq, amp, phase) → float`:
```python
mc.sine(t, freq, amp, phase)       # smooth sinusoid
mc.cosine(t, freq, amp, phase)     # 90°-shifted sinusoid
mc.square(t, freq, amp, phase, duty=0.5)
mc.triangle(t, freq, amp, phase)
mc.sawtooth(t, freq, amp, phase)
mc.trapezoid(t, freq, amp, phase, ramp=0.25)
```

**Servo targets** — `→ {servo_id: value}`:
```python
mc.drive(servo_id, value)          # one servo
mc.drive_multi({3: 0.5, 4: -0.3})  # several at once
mc.hold([3, 4], value=0.0)         # hold a group at one value
```

**Gaits** — coordinated roll + pitch fin motion (waveform is an argument):
```python
# Roll held broadside (π/2), pitch oscillates around 0 with the chosen waveform
mc.flap(roll_id, pitch_id, t, freq, amp, waveform=mc.sine)

# Rowing stroke: broadside power phase + graceful feathered recovery
mc.paddle(roll_id, pitch_id, t, freq, amp, power_fraction=0.5)
```

**Search helper**:
```python
mc.sweep(servo_id, t, rate, span)  # slow one-axis triangle sweep for scanning
```

### Adding a new gait
1. Write a pure function in `motion_command.py` that returns `{servo_id: value}`, composing the waveform and `drive` helpers.
2. Accept the waveform as an argument (`waveform=mc.sine`) so the shape stays interchangeable.
3. Call it from the controller's motion-generation methods, passing the fin's `roll_id`/`pitch_id`.
4. Rebuild: `colcon build && source install/setup.bash`.

---

## Recording and Analysis (`recorder.py`)

Automated data collection script for ROS2 bag recording, CSV extraction, and plot generation organized by servo and command.

**Features:**
- Records all control ROS2 topics (`mission_input`, `mission_cmd`, `mission_status`, `apriltag_detections`, `joint_cmd`, `joint_feedback`, `telemetry`)
- Supports both mcap and sqlite3 bag formats (auto-detects)
- Extracts master CSVs for all ROS2 topics
- Generates CSV snippets per servo per command for focused analysis
- Creates comprehensive plots:
  - System-wide master plots (all topics, all servos combined)
  - Per-servo master plots (all commands for one servo)
  - Per-servo per-command plots (individual command execution)
  - Goal vs actual position tracking
  - Positions, velocities, currents, voltages

**Usage:**
```bash
python3 recorder.py session_name
# Press Ctrl+C to stop recording
# Automatically extracts to CSV and generates plots
```

**Output Structure:**

```
session_name/
├── rosbag/                    # ROS2 bag files (mcap or sqlite3)
├── csv/                       # Master CSV files (all data)
│   ├── mission_input.csv
│   ├── mission_cmd.csv
│   ├── mission_status.csv
│   ├── apriltag_detections.csv
│   ├── joint_cmd.csv
│   ├── joint_feedback.csv
│   └── telemetry.csv
└── plots/
    ├── master/
    │   └── plots/            # System-wide plots
    │       ├── telemetry_positions.png
    │       ├── telemetry_velocities.png
    │       ├── telemetry_currents.png
    │       ├── telemetry_voltages.png
    │       ├── feedback_positions.png
    │       ├── feedback_velocities.png
    │       ├── feedback_currents.png
    │       ├── feedback_voltages.png
    │       ├── mission_progress.png
    │       ├── mission_orientation.png
    │       ├── apriltag_distance.png
    │       └── apriltag_bearing.png
    ├── servo_3/
    │   ├── master/
    │   │   └── plots/        # All commands for servo 3
    │   │       ├── servo_3_positions.png
    │   │       ├── servo_3_velocities.png
    │   │       ├── servo_3_currents.png
    │   │       ├── servo_3_voltages.png
    │   │       └── feedback_*.png
    │   ├── cmd_0/
    │   │   ├── csv/
    │   │   │   └── telemetry_snippet.csv    # Just this servo, this command
    │   │   └── plots/
    │   │       ├── servo_3_cmd_0_positions.png
    │   │       ├── servo_3_cmd_0_velocities.png
    │   │       ├── servo_3_cmd_0_currents.png
    │   │       └── servo_3_cmd_0_voltages.png
    │   └── cmd_1/
    │       ├── csv/
    │       │   └── telemetry_snippet.csv
    │       └── plots/
    └── servo_4/
        └── ... (same structure)
```

**Plot Features:**
- Simple scatter plots with point markers
- Goal vs actual position overlay on telemetry plots
- Legends on all plots for clarity
- Time normalized to start at 0 ms
- Per-servo organization for easy analysis
- CSV snippets enable quick data inspection per command
- System-wide master plots show overall behavior
- Per-servo per-command plots isolate individual executions

---

## Robot Design

### Multimodal Soft Actuator System

The robot features a bio-inspired design following blue crab physiology proportions with adaptive flipper morphology for air and water environments.

**Design Philosophy:**
- Multimodal operation: Similar gait behavior in air and underwater
- Soft actuator-based propulsion with variable stiffness control
- Bio-inspired proportions based on blue crab anatomy
- Rapid prototyping iteration (8 CAD iterations, 5 physical prototypes)

### Flipper Designs

#### Air Flipper (Reinforced)
- **Structure:** Carbon fiber rod reinforcement for variable stiffness control
- **Covering:** Icarex fabric with outer skeletal frame
- **Purpose:** Maximize torque transmission and structural rigidity in air
- **Stiffness Control:** Carbon fiber rods provide optimal force distribution

#### Underwater Flipper (Compliant)
- **Structure:** Soft actuator only, no reinforcement
- **Covering:** Bare soft actuator material
- **Purpose:** Hydrodynamic efficiency and compliance for underwater propulsion
- **Design:** Mimics natural crab flipper flexibility

**Flipper Development:**
- 9 design iterations (CAD)
- 9 fabrication iterations (physical prototypes)
- Progressive refinement from first to final prototype

---

### Robot CAD Models

**[CAD Model Pictures - To Be Uploaded]**

*Reserved space for:*
- First prototype CAD render
- Final prototype CAD render
- Full assembly views

---

### Flipper Progression

**[Flipper CAD Progression - To Be Uploaded]**

*Reserved space for:*
- Air flipper CAD progression (first → final)
- Underwater flipper CAD progression (first → final)

---

### Physical Robot

**[Robot Photos - To Be Uploaded Soon]**

*Reserved space for:*
- Assembled robot (air configuration)
- Assembled robot (underwater configuration)
- Detail shots of flipper mechanisms

---

### Flipper Prototypes

**[Flipper Photos - To Be Uploaded]**

*Reserved space for:*
- Air flipper physical prototype
- Underwater flipper physical prototype
- Comparison shots showing structural differences

---

## Hardware Specification

| Component | Model | Protocol/Interface | Notes |
|-----------|-------|-------------------|-------|
| **Actuators** | Dynamixel XW430-T200 (×4) | RS-485, Protocol 2.0 | 2 per side (pitch + roll), encoder feedback |
| **IMU** | Adafruit ICM-20948 (9-Axis) | I2C (address 0x69) | Accelerometer, gyroscope, magnetometer @ 100 Hz |
| **Vision** | DWE StellarHD USB Camera | USB 2.0/3.0, OpenCV | 1920×1080 @ 30 fps, synchronized video recording |
| **Compute** | NVIDIA Jetson Orin Nano (8GB) | — | ROS 2 Jazzy, Ubuntu 24.04 |
| **Power** | 3S LiPo Battery Pack | — | Dual rail: servos + compute |
| **Servo Bus** | Dynamixel U2D2 Power Hub | USB → RS-485 | Power distribution + RS-485 interpreter |

**Power Distribution:**
```
3S LiPo Battery Pack (11.1–12.6V)
    ├── Rail 1 → U2D2 Power Hub → 4× XW430-T200 Servos
    └── Rail 2 → Jetson Orin Nano (Camera + IMU powered via Nano)
```

**Future Sensor Fusion:**
Closed-loop control will integrate vision (AprilTag) and IMU data with motor encoder feedback for state estimation, enabling position/orientation feedback and autonomous navigation in both air and water environments.

---

## Gazebo Simulation

### Overview

A Gazebo Harmonic simulation environment for kinematic testing and gait development without physical hardware. The simulation provides a **hybrid architecture** that auto-detects real servos and seamlessly merges physical and simulated feedback.

**Simulation Capabilities:**
- Full 2-DOF kinematic model (2 servos per side: pitch + roll)
- Visual rendering of robot geometry and motion
- Simulated IMU (9-axis accelerometer/gyroscope/magnetometer)
- Simulated camera (StellarHD, 1920×1080 @ 30fps)
- Hybrid mode: Real servos + simulated servos in single session
- Position and velocity control modes
- Same control stack as hardware (crab.py, controller.py)

**Simulation Scope:**
- Kinematic motion and joint dynamics
- Simplified rigid-body flipper representation
- No thermal or power consumption modeling

**Use Cases:**
- Gait development and parameter tuning before hardware testing
- Motion library function validation
- Control algorithm verification
- Trajectory visualization and debugging
- Educational demonstrations
- Hybrid testing (2 real servos + 2 simulated)

---

### Hybrid Architecture

**Automatic Hardware Detection:**

The Gazebo interface (`gazebo_dynamixel_interface.py`) automatically detects connected real servos and merges them with simulated servos:

```
Real Servos Detected    →  Use real hardware feedback for those IDs
Real Servos NOT Detected →  Use Gazebo simulation feedback for all IDs
Hybrid Configuration     →  Real feedback for IDs [1,2], Sim feedback for IDs [3,4]
```

**Benefits:**
- Seamless transition between simulation and hardware
- Test control logic with partial hardware
- Incremental hardware integration during development
- Same command interface for sim and hardware

---

### Simulation Components

**Simulated Interfaces:**

1. **Gazebo Dynamixel Interface** (`gazebo_dynamixel_interface.py`)
   - Auto-detects real servos via RS-485 ping
   - Publishes merged feedback from real + simulated servos
   - Sends commands to both real hardware and Gazebo visualization
   - Maintains 500 Hz feedback rate

2. **Gazebo IMU Interface** (`gazebo_icm20948_interface.py`)
   - Subscribes to `/imu` ROS2 topic from Gazebo
   - Publishes to `/imu/data` in sensor_msgs/Imu format
   - Simulates ICM-20948 9-axis IMU behavior

3. **Gazebo Camera Interface** (`gazebo_stellarhd_interface.py`)
   - Subscribes to `/camera/image_raw` ROS2 topic from Gazebo
   - Records video to disk (MP4 format)
   - Simulates StellarHD camera interface

**URDF Model:**
- 2-DOF robot with base_link and 4 revolute joints (left_pitch, left_roll, right_pitch, right_roll)
- Accurate mass and inertia properties from Fusion 360 CAD export
- Visual meshes for all links including electronics box, servo housings, aerial truss structures, and camera bracket
- Camera and IMU sensor links

---

### Gazebo Simulation Screenshots

**[Gazebo Screenshots - To Be Uploaded]**
s
- Gazebo environment with robot model
- Robot executing flap / paddle gaits in simulation
- Hybrid mode (real + simulated servos) visualization
- Camera view from simulated camera

---

### Launch and Usage

**Start Simulation:**
```bash
# Launch Gazebo with all simulated interfaces (includes AprilTag perception)
ros2 launch soft_propulsors_control gazebo_launch.py

# In another terminal, feed missions (same interface as hardware)
src/soft_propulsors_control/scripts/feed_missions.sh missions.txt 3

# Record simulation data
python3 recorder.py sim_session_1
```

**Hybrid Mode (Partial Hardware):**
```bash
# Connect 2 real servos via USB, launch Gazebo
ros2 launch soft_propulsors_control gazebo_launch.py

# System auto-detects servos and merges feedback:
# INFO: Detected real servo ID 1
# INFO: Detected real servo ID 2
# INFO: Hybrid Dynamixel interface ready - Real servos: [1, 2], Simulated: [3, 4]
```

**Parameters (gazebo_launch.py):**
- `joint_names`: List of joint names in URDF
- `servo_ids`: Corresponding Dynamixel servo IDs
- `port`: Serial port for real servo detection (default: `/dev/ttyUSB0`)
- `baudrate`: RS-485 communication speed (default: 1000000)

---

## Verification and Validation

### V&V Framework

This project follows a **V-Model verification and validation methodology** structured around IEEE 1012-2016 (System and Software V&V), with safety analysis informed by MIL-STD-882E (System Safety) and functional safety practices from IEC 61508. Testing is organized into four phases progressing from component-level verification through system-level validation, with continuous regression monitoring via CI/CD.

```
Requirements ─────────────────────────────────────► System Validation (Phase 3)
    │                                                 ▲
    ▼                                                 │
System Design ────────────────────────────────────► Integration Testing (Phase 2)
    │                                                 ▲
    ▼                                                 │
Component Design ─────────────────────────────────► Component Verification (Phase 1)
    │                                                 ▲
    ▼                                                 │
Implementation ───────────────────────────────────────┘
    │
    ▼
Continuous Regression (Phase 4)
```

**Testing Phases:**

| Phase | Scope | Methodology | Operation |
|-------|-------|-------------|------------|
| 1. Component Verification | Manufacturing, electrical, software | Plan-Driven V&V (Waterfall) | Partially automated — unit tests automated via CI |
| 2. Integration Testing | SIL (Gazebo), hardware-software, multi-sensor | Software/Hardware-in-the-Loop | Automated — `test_position_mode.sh`, `test_velocity_mode.sh` |
| 3. System Validation | Performance, safety, endurance | Plan-Driven V&V + Fault Injection | Partially automated — `recorder.py` + analysis scripts |
| 4. Continuous Regression | Performance tracking vs baseline | CI/CD (DevOps) | Automated — GitHub Actions |

---

### Phase 1: Component Verification

Component-level verification confirms that each manufactured part, electrical subsystem, and software module meets its design specification in isolation before integration.

#### 1.1 Manufacturing Verification

**CNC Machined Components:**

All brackets, mounts, and base plate verified by dimensional inspection against CAD drawings. Critical dimensions measured with calipers; flatness verified against a surface plate. Acceptance: ±0.1mm on mating surfaces, ±0.005" on 1/4"-20 fastener holes.

**Laser Cut Components:**

Electronics board and mounting plates verified for dimensional accuracy against DXF source files. Edge quality inspected for charring or burrs that would affect fit. Acceptance: ±0.05mm dimensional, fasteners seat without forcing.

**Soft Actuator Assembly:**

Flipper-to-servo-horn attachment verified by manual torque test to confirm no slip at maximum servo output torque. Carbon fiber rod insertion (air flipper) verified by pull test for seating and rotation resistance. Icarex fabric bonding verified by manual peel test at edges.

**AV Bay Enclosure:**

Chord grip installations verified for seating flush with enclosure wall. Silicone seals inspected for full bead coverage with no voids. Enclosure lid closure verified for full fastener engagement without binding.

**Fastener Verification:**

All M3 and 1/4"-20 joints inspected for minimum 2× diameter thread engagement. All joints subject to vibration receive nylock nuts or threadlocker. Continuity of fastener engagement verified before operation.

#### 1.2 Electrical Verification

**Power System:**

| Test | Acceptance Criteria |
|------|---------------------|
| Battery voltage, no load | 11.1–12.6V (3S LiPo nominal) |
| Servo rail voltage under load (4 servos flapping) | >11.0V at U2D2 hub |
| Nano rail voltage under compute load | Stable within regulator specification |
| Rail-to-rail ground potential | <50mV between servo rail and Nano rail |
| Cable continuity (all signal and power cables) | <1Ω end-to-end |
| Connector seating (all connectors) | No disconnection under light pull |
| Insulation integrity | No exposed conductors, no pinch points |

**Communication Bus:**

| Bus | Test | Acceptance Criteria |
|-----|------|---------------------|
| RS-485 (U2D2 → Servos) | Dynamixel SDK broadcast ping | All 4 servo IDs respond, <1ms round-trip |
| USB (U2D2 → Nano) | Device enumeration | `/dev/ttyUSB*` detected at 1 Mbaud |
| USB (Camera → Nano) | OpenCV `VideoCapture` init | Frame acquired at 1920×1080 |
| I2C (IMU → Nano) | `i2cdetect` bus scan | Device responds at address 0x69 |

**Automation:** `electrical_verification.py` performs servo ping, camera init, and IMU detect in a single pass and logs pass/fail per subsystem.

#### 1.3 Software Verification

**Unit Testing:**

Automated test suite validates all software modules with mock hardware interfaces. Total: 48 unit tests covering motion library functions (18), controller PID logic (12), command parsing (8), ROS2 wire format (6), and position limit enforcement (4). Target coverage: >80% for motion library and controller modules.

**CI Pipeline:**

GitHub Actions workflow triggers on every push and pull request. Pipeline builds workspace, runs pytest suite, and checks PEP8 compliance via flake8. Merge is blocked if any test fails or coverage drops below threshold.

---

### Phase 2: Integration Testing

Integration testing validates subsystem interactions, progressing from simulated (SIL) through hardware-software to multi-sensor integration.

#### 2.1 Software-in-the-Loop — Gazebo Simulation (IEC 61508 §7.4.7)

Control algorithms are validated against the Gazebo kinematic model before hardware deployment. The simulation environment itself is validated first (see Gazebo Testing and Validation under the Gazebo Simulation section).

**Automated Test Suites:**
- `test_position_mode.sh` — 31 position control tests (drive commands, flap/paddle variations, phase offsets, waveforms, edge cases)
- `test_velocity_mode.sh` — 32 velocity control tests (tracking, boundary enforcement, ramp response)

Total: 63 automated SIL tests.

**Acceptance Criteria:** All 63 tests pass. Phase offset error <5°. Position commands within actuator_map limits. Control loop maintains 400 ±10 Hz. Zero ROS2 topic communication failures.

**Sim-to-Hardware Correlation:** `compare_sim_hardware.py` runs identical test suites on simulation and hardware, then quantifies deviations in tracking error, phase accuracy, settling time, and steady-state error. Identifies hardware-dependent behaviors (backlash, compliance, friction) not captured by the kinematic model.

#### 2.2 Hardware-Software Integration

Each hardware subsystem is verified with its ROS2 interface node operating in the full control stack.

**Actuator Integration:**

| Test | Acceptance Criteria |
|------|---------------------|
| Broadcast ping (all 4 servo IDs) | 4/4 respond via Dynamixel SDK |
| Position write/read cycle | Encoder feedback within ±0.05 rad of commanded |
| Velocity write/read cycle | Encoder feedback within ±0.1 rad/s of commanded |
| GroupSyncWrite (4 servos simultaneous) | <1ms inter-servo latency |
| SIGINT emergency stop during motion | Torque disabled within 100ms |

**IMU Integration (Adafruit ICM-20948):**

| Test | Acceptance Criteria |
|------|---------------------|
| ROS2 topic publish rate (`/imu_data`) | 100 ±5 Hz sustained |
| Static accelerometer reading (gravity) | 9.81 ±0.5 m/s² magnitude |
| Static gyroscope reading (zero rotation) | <0.5°/s bias |
| Coordinate frame verification (rotate about known axis) | Correct axis responds |
| Magnetometer read | Non-zero field, consistent heading |

**Camera Integration (DWE StellarHD):**

| Test | Acceptance Criteria |
|------|---------------------|
| Frame acquisition | 30 ±2 FPS at 1920×1080 |
| Mission-synchronized video recording | Recording segments on `mission_cmd` / `mission_status` transitions |
| Recording integrity (60-second capture) | MP4 playback without corruption |
| AprilTag detection in FOV | Detection at 2m range, <1° orientation error |

**Automation:** `integration_test.py` executes the full hardware-software integration suite and logs pass/fail per subsystem.

#### 2.3 Multi-Sensor Integration

Validates data synchronization and correlation across subsystems operating simultaneously.

| Test | Acceptance Criteria |
|------|---------------------|
| IMU-to-encoder timestamp alignment | <10ms synchronization error |
| Camera-to-encoder timestamp alignment | <50ms synchronization error |
| Camera-to-IMU timestamp alignment | <50ms synchronization error |
| IMU vibration spectrum during flapping | Structural resonances documented |
| AprilTag + encoder fused pose vs encoder-only | Position error <5mm at 1m range |

**Automation:** `sensor_fusion_validation.py` runs multi-sensor recording during gait execution and computes cross-correlation metrics.

---

### Phase 3: System Validation

System-level validation characterizes performance with the complete system operating in its target configuration.

#### 3.1 Actuator Performance Characterization

**Step Response:**

| Metric | Target |
|--------|--------|
| Rise time (10–90%) | <200ms |
| Overshoot | <10% |
| Settling time (2% band) | <500ms |
| Steady-state error | <0.05 rad |

**Frequency Response (System Identification):**

Sine sweeps from 0.1–10 Hz (position) and 0.1–20 Hz (velocity) generate Bode plots for bandwidth identification. Expected position bandwidth: 5–10 Hz @ -3dB. Expected velocity bandwidth: 10–20 Hz @ -3dB. Phase margin: >30°.

**Gait Characterization:**

| Metric | Target |
|--------|--------|
| Phase offset accuracy (servo pair) | ±5° |
| Amplitude tracking | ±0.05 rad |
| Frequency accuracy | ±0.05 Hz |
| Inter-servo synchronization | <5ms temporal lag |

**Automation:** `generate_sysid_tests.py` generates frequency sweep sequences. `analyze_bode_plot.py`, `analyze_step_response.py`, and `analyze_gait.py` extract metrics from recorded telemetry CSVs.

#### 3.2 Sensor Characterization

**IMU Calibration and Noise:**

6-position tumble test extracts accelerometer and gyroscope bias (<50 mg, <0.5°/s targets) and scale factors (<0.5% error target). Magnetometer hard/soft iron compensation via sphere fit (R² >0.95 target). 10-minute static recording yields power spectral density and angular random walk for noise baseline.

**Automation:** `imu_calibrate` performs tumble test procedure. `analyze_imu_noise.py` computes PSD and Allan variance.

**Camera Calibration:**

Checkerboard-based intrinsic calibration via OpenCV. Target reprojection error: <0.5px. Distortion model coefficients (K1, K2, P1, P2) extracted and stored in `camera_intrinsics.yaml`. Motion blur characterized at flapping frequencies for image quality assessment.

**Automation:** `calibrate_camera.py` performs full intrinsic calibration from checkerboard captures.

#### 3.3 Safety Verification (MIL-STD-882E)

Fault injection testing validates system response to hazardous conditions. Each fault is injected during active gait execution.

| Fault | Expected Response |
|-------|-------------------|
| SIGINT (Ctrl+C) during motion | Servo torque disabled within 100ms |
| USB disconnection (U2D2 cable) | Servos hold last safe state or disable, no uncontrolled motion |
| Power brownout (voltage drop to 10V) | Servo protection activates, no erratic behavior |
| I2C bus failure (IMU disconnect) | System continues without IMU, error logged |
| USB failure (camera disconnect) | System continues without camera, error logged |
| Command beyond position limits | Clamped at actuator_map boundary, no physical limit contact |
| Velocity toward position limit | Velocity zeros at boundary, smooth stop |
| Controller node crash (kill controller.py) | Application layer detects, servos disable within 500ms |
| Servo overcurrent (blocked flipper) | Current limit activates at 1200mA threshold |

**Automation:** `safety_test.py` performs automated fault injection where possible. Manual faults (USB disconnect, power brownout) follow documented procedures with pass/fail criteria.

#### 3.4 Endurance Validation

| Test | Duration | Acceptance Criteria |
|------|----------|---------------------|
| Continuous flapping (1 Hz, max amplitude) | 30 minutes | No faults, servo case temp <60°C, no tracking degradation |
| Continuous flapping (max frequency) | 10 minutes | No faults, no communication errors |
| Repeated start/stop cycles | 100 cycles | No communication errors or state corruption |
| Battery discharge test | Until voltage cutoff | Runtime documented, no brownout faults |

**Automation:** `endurance_test.py` runs timed gait sequences with continuous telemetry recording. `analyze_thermal.py` plots temperature rise curves from servo current data.

#### 3.5 Electrical Characterization Under Load

| Test | Acceptance Criteria |
|------|---------------------|
| Servo rail voltage ripple during flapping (oscilloscope) | <5% ripple |
| Peak current draw per servo (ammeter) | <2A at 1 Hz flapping |
| RS-485 signal integrity (logic analyzer, 1 Mbaud) | Zero errors over 1 hour |
| Servo case temperature after 10 min operation (IR thermometer) | <60°C |
| Jetson Orin Nano temperature under full load | <80°C |

#### 3.6 Mechanical Characterization Under Load

| Test | Acceptance Criteria |
|------|---------------------|
| Flipper deflection during flapping (high-speed camera + OpenCV) | <5° from rigid body |
| Joint backlash (encoder delta on direction reversal) | <0.2° mechanical deadband |
| Flipper resonance (frequency sweep, accelerometer at tip) | Resonant frequencies documented |
| Fastener retention after 1000 flapping cycles | No loosening |
| Chord grip seal after submersion (10 min @ 1m depth) | No moisture ingress |

---

### Phase 4: Continuous Regression (CI/CD)

Regression testing detects performance degradation on code changes. Baseline recordings (golden datasets) for each motion function are stored in `test/golden_datasets/`, tagged with git commit hash.

**Regression Metrics:**

| Metric | Tolerance | Action if Exceeded |
|--------|-----------|-------------------|
| Position tracking error | ±5% vs baseline | Block merge |
| Velocity tracking error | ±5% vs baseline | Block merge |
| Control loop rate | ±10 Hz vs baseline | Block merge |
| Phase offset accuracy | ±2° vs baseline | Block merge |
| CPU / memory usage | +10% vs baseline | Warning, document reason |

**Automation:** `regression_test.py` compares new telemetry CSVs against golden datasets and outputs pass/fail per metric. GitHub Actions workflow runs regression suite on every pull request, generates comparison plots, and blocks merge if any metric exceeds tolerance.

---

### Performance Benchmarks

| Metric | Target | Measured |
|--------|--------|----------|
| Control loop rate | 400 Hz | |
| Control loop jitter | <1 ms std | |
| Position tracking error | <0.05 rad RMS | |
| Velocity tracking error | <0.1 rad/s RMS | |
| ROS2 topic latency (cmd→fb) | <5 ms | |
| CPU usage (all nodes) | <30% | |
| Memory footprint | <500 MB | |
| IMU sample rate | 100 Hz | |
| Camera frame rate | 30 FPS | |
| Emergency stop response | <100 ms | |
| Battery runtime (1 Hz flap) | >30 min | |

---

## Dependencies

**System:**
- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic (for simulation)

**Python Packages:**
```bash
pip install dynamixel-sdk numpy pandas matplotlib opencv-python --break-system-packages
```
`opencv-python` provides the ArUco/AprilTag detector used by `apriltag_interface`.

**ROS 2 Packages:**
```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher
# cv_bridge is only needed for apriltag_interface in 'topic' (Gazebo) mode:
sudo apt install ros-jazzy-cv-bridge
```

**Gazebo (Optional - for simulation):**
```bash
sudo apt install gz-harmonic
```

---

## Installation and Build

```bash
# Clone repository
cd ~/
git clone <repository-url> compliant-propulsors-control
cd compliant-propulsors-control

# Install dependencies
pip install dynamixel-sdk numpy pandas matplotlib --break-system-packages

# Build ROS2 workspace
colcon build
source install/setup.bash
```

---

## Launch

**Hardware Mode:**
```bash
# Launch full autonomous stack with real servos, IMU, and camera perception
ros2 launch soft_propulsors_control crab_launch.py

# In another terminal, feed the mission queue (one mission every 3 s)
src/soft_propulsors_control/scripts/feed_missions.sh missions.txt 3

# Record session data
python3 recorder.py test_session_1
```

The robot waits `startup_delay` seconds after launch (deployment settling) before
moving — launch on land, then drop it in the water within that window. It then
scans for each mission's AprilTag and heads toward it autonomously.

**Simulation Mode:**
```bash
# Launch Gazebo simulation (same control stack + simulated camera/IMU)
ros2 launch soft_propulsors_control gazebo_launch.py

# Feed missions (same interface as hardware)
src/soft_propulsors_control/scripts/feed_missions.sh missions.txt 3

# Record simulation data
python3 recorder.py sim_test_session_1
```

---

## Project Status

**Status:** In Progress

---

## License


---

## Acknowledgments

This project implements the ROS2 Control Framework architecture for modular robot control system design.