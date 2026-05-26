# Compliant Propulsors Control 

A ROS 2 Jazzy control stack for a bio-inspired blue crab robotic system with multimodal soft actuator design. The architecture follows the ROS2 Control Framework with modular decoupling between behavioral logic, motion generation, control, and hardware interfaces, enabling operation in both air and water environments.

---

## ROS2 Control Framework Architecture

```
┌──────────────┐      ┌──────────────┐      ┌─────────────────┐      ┌──────────────────────┐
│ Application  │◄────►│  Controller  │◄────►│  HW Interface   │◄────►│      Hardware        │
│  (crab.py)   │      │(controller.py)│      │(Dynamixel...py) │      │ (Servos, IMU, Cam)   │
└──────────────┘      └──────────────┘      └─────────────────┘      └──────────────────────┘
       │                      │                       │                          │
       │                      │                       │                ┌─────────┴─────────┐
   robot_cmd             motion_cmd             joint_cmd              │                   │
   telemetry             telemetry           joint_feedback   Dynamixel SDK    I2C (IMU)   USB (Camera)
                              └────────── ros2_control framework ────────┘    Adafruit      OpenCV
                                                                              ICM20948      StellarHD
```

*Architecture based on ROS2 Control Framework*

**Control Software Key Design Features:**
- **Telemetry-Driven Architecture:** 400 Hz closed-loop control driven by hardware feedback
- **Servo-Level Gait Control:** Motion library functions can command individual servos or synchronized groups
- **Position Limits in Motion Wire:** Single source of truth for safety limits, passed dynamically per command
- **Emergency Torque Disable:** SIGINT handler for instant servo torque shutdown on Ctrl+C
- **Per-Servo Differential Motion:** Support for phase-offset, frequency ratios, and amplitude scaling between servo pairs
- **Extensible Motion Library:** User-defined motion functions with flexible kwargs-based parameterization

---

## Table of Contents

1. [Node Specifications](#node-specifications)
   - [Application Layer (crab.py)](#application-layer-crabpy)
   - [Controller Layer (controller.py)](#controller-layer-controllerpy)
   - [Hardware Interface (Dynamixel_XW430_T200_interface.py)](#hardware-interface-dynamixel_xw430_t200_interfacepy)
2. [ROS2 Topic Specifications](#ros2-topic-specifications)
3. [Motion Library](#motion-library-motion_librarypy)
4. [Recording and Analysis](#recording-and-analysis-recorderpy)
5. [Robot Design](#robot-design)
6. [Hardware Specification](#hardware-specification)
7. [Gazebo Simulation](#gazebo-simulation)
8. [Verification and Validation](#verification-and-validation)
   - [V&V Framework](#vv-framework)
   - [Phase 1: Component Verification](#phase-1-component-verification)
   - [Phase 2: Integration Testing](#phase-2-integration-testing)
   - [Phase 3: System Validation](#phase-3-system-validation)
   - [Phase 4: Continuous Regression](#phase-4-continuous-regression)
   - [Performance Benchmarks](#performance-benchmarks)
9. [Dependencies](#dependencies)
10. [Installation and Build](#installation-and-build)
11. [Launch](#launch)
12. [Project Status](#project-status)
13. [License](#license)
14. [Acknowledgments](#acknowledgments)

---

## Node Specifications

### Application Layer (`crab.py`)
**The System "Brain" - Gait Engine and Command Orchestrator**

Manages behavioral state, command queuing, and motion function execution. Parses high-level robot commands and translates them into time-series motion trajectories.

**Responsibilities:**
- Parse and queue robot commands from `robot_cmd` ROS2 topic
- Execute motion functions from motion library at 400 Hz (telemetry-driven)
- Apply servo offsets and position limits from actuator map
- Manage command sequencing with cycle-based duration control
- Support both gait-based (time-series) and direct (immediate) commands

**Input ROS2 Topics:**
- `robot_cmd` (std_msgs/String): Behavioral command strings
- `telemetry` (std_msgs/Float32MultiArray): Feedback from controller triggering next gait sample

**Output ROS2 Topics:**
- `motion_cmd` (std_msgs/Float32MultiArray): Commanded positions/velocities with position limits

**Parameters:**
- `actuator_map`: JSON array `[[id, offset_rad, set_id, min_limit, max_limit], ...]`
- `operating_mode`: 'position' or 'velocity'

**Key Features:**
- Telemetry-driven execution (no timers, pure feedback-driven)
- Servo-level and set-level motion function support
- Dynamic position limit enforcement
- Command queue with sequential execution

---

### Controller Layer (`controller.py`)
**The "Kinematic Engine" - Outer-Loop PID Controller**

Applies optional outer-loop PID correction on top of servo internal control. Acts as a trajectory conditioner and safety layer between motion commands and hardware.

**Responsibilities:**
- Parse motion commands with embedded position limits
- Apply outer-loop PID correction (position or velocity mode)
- Enforce position limits via clamping (position mode) or velocity zeroing (velocity mode)
- Publish telemetry for gait engine synchronization
- Run at 400 Hz control rate

**Input ROS2 Topics:**
- `motion_cmd` (std_msgs/Float32MultiArray): Wire format `[cmd_id, ids, modes, values, limits]`
- `joint_feedback` (std_msgs/Float32MultiArray): Encoder feedback from hardware

**Output ROS2 Topics:**
- `joint_cmd` (std_msgs/Float32MultiArray): Final corrected commands to hardware
- `telemetry` (std_msgs/Float32MultiArray): Commanded goals + actual feedback for logging/control

**Parameters:**
- `kp`, `ki`, `kd`: Outer-loop PID gains (0.0 = open-loop passthrough)
- `control_rate`: Control loop frequency in Hz (default: 400.0)
- `telemetry_decimation`: Publish every Nth sample (default: 1)

**Key Features:**
- Cascaded PID structure (outer + servo internal)
- Position limit enforcement from motion_cmd
- DOF-agnostic (handles any number of servos)
- Telemetry decimation for reduced bandwidth

---

### Hardware Interface (`Dynamixel_XW430_T200_interface.py`)
**The Hardware Driver - Dynamixel Protocol 2.0 Interface**

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

## ROS2 Topic Specifications

| ROS2 Topic | Type | Direction | Wire Format | Purpose |
|-------|------|-----------|-------------|---------|
| `robot_cmd` | String | User → crab | Key-value pairs `cmd_id:[1] func:[name] ...` | High-level behavioral commands |
| `motion_cmd` | Float32MultiArray | crab → controller | `[cmd_id, ids, modes, values, limits]` | Motion commands with position limits |
| `joint_cmd` | Float32MultiArray | controller → hardware | `[ids, modes, values]` | Final servo commands |
| `joint_feedback` | Float32MultiArray | hardware → controller | `[id, mode, pos, vel, curr, volt]` per servo | Encoder feedback |
| `telemetry` | Float32MultiArray | controller → crab | `[cmd_id, sample, goal, pos, vel, curr, volt]` per servo | Control loop synchronization |

### Topic Details

#### `robot_cmd` (User Input)
**Format:** `"cmd_id:[id] func:[name] freq:[hz] amp:[rad] phase:[rad] cycles:[n] sets:[s1,s2,...] freq_ratio:[r] amp_ratio:[r] phase_offset:[rad]"`

**Command Types:**

1. **Gait Commands** (time-based trajectories):
```bash
cmd_id:[1] func:[sine_flap] freq:[0.5] amp:[1.0] phase:[0.0] cycles:[3] sets:[1]
```
- `cmd_id`: Unique command identifier
- `func`: Motion function name from motion_library.py
- `freq`: Oscillation frequency (Hz)
- `amp`: Amplitude (radians or rad/s)
- `phase`: Phase offset (radians)
- `cycles`: Number of oscillation cycles
- `sets`: Which servo sets to activate
- **Optional kwargs:** `freq_ratio`, `amp_ratio`, `phase_offset`, or any custom parameters

2. **Direct Commands** (immediate):
```bash
cmd_id:[1] func:[drive] servo:[3] value:[0.5]
cmd_id:[2] func:[drive_multi] servos:{3:0.5, 4:-0.3}
```
- Commands servos immediately and persists until overwritten

#### `motion_cmd` Wire Format
**Structure:** `[cmd_id, id0, id1, ..., mode0, mode1, ..., val0, val1, ..., min0, max0, min1, max1, ...]`

**Example (2 servos):**
```
[1.0,           # cmd_id
 3.0, 4.0,      # servo IDs
 3.0, 3.0,      # modes (3=position, 1=velocity)
 2.5, 3.2,      # commanded values (rad or rad/s)
 -8.0, 8.0,     # servo 3 limits (min, max)
 -8.0, 8.0]     # servo 4 limits (min, max)
```

**Key Feature:** Position limits embedded in wire format - single source of truth from actuator_map

#### `joint_cmd` Wire Format
**Structure:** `[id0, id1, ..., mode0, mode1, ..., val0, val1, ...]`

Final commands after PID correction and limit enforcement.

#### `joint_feedback` Wire Format
**Structure:** `[id, mode, pos_rad, vel_rad_s, curr_A, volt_V]` repeated per servo

Encoder feedback at 500 Hz from hardware interface.

#### `telemetry` Wire Format
**Structure:** `[cmd_id, sample, goal0, pos0, vel0, curr0, volt0, goal1, pos1, vel1, curr1, volt1, ...]`

Control loop data at 400 Hz for synchronization and logging.

---

## Motion Library (`motion_library.py`)

The motion library contains all motion functions. Each function receives core parameters `(t, freq, amp, phase)` and optional custom parameters via `**kwargs`.

### Motion Function Signature
```python
def my_motion(self, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict:
    """
    Core Parameters:
        t: Time (seconds)
        freq: Frequency (Hz)
        amp: Amplitude (radians or rad/s)
        phase: Phase offset (radians)
    
    Kwargs:
        custom_param (type): Description (default: value)
    
    Returns:
        dict: {servo_id: value} or {set_id: value}
    """
    custom_param = kwargs.get('custom_param', default_value)
    # ... motion logic ...
    return {servo_id: value}
```

### Available Motion Functions

**Direct Control:**
- `drive(servo, value)`: Single servo control
- `drive_multi(**kwargs)`: Multiple servo control via `servos:{3:0.5, 4:-0.3}` or individual kwargs

**Waveform Functions:**
- `sine()`: Smooth sinusoidal oscillation
- `square()`: Bang-bang control with duty cycle
- `triangle()`: Constant-velocity sweeps
- `sawtooth()`: Linear ramp with instant reset
- `step()`: Discrete state switching
- `trapezoid()`: Smooth acceleration/deceleration

**Flapping Functions:**
- `sine_flap()`: Differential phase/frequency/amplitude between servo pairs
  - Default: 90° phase offset for standard flapping
  - Supports `freq_ratio`, `amp_ratio`, `phase_offset` kwargs
- `sine_paddle()`: Rowing motion with sawtooth + sine pairing

### Adding Custom Motion Functions

1. Add function to `MotionLibrary` class in `motion_library.py`
2. Use signature: `def my_motion(self, t, freq, amp, phase, **kwargs)`
3. Extract custom parameters: `my_param = kwargs.get('my_param', default)`
4. Return `{servo_id: value}` or `{set_id: value}`
5. Rebuild: `colcon build && source install/setup.bash`
6. Use in robot_cmd: `func:[my_motion] freq:[0.5] my_param:[2.5] cycles:[3] sets:[1]`

**Access to servo configuration:**
- `self.node.all_ids`: List of all servo IDs
- `self.node.set_map`: `{set_id: [servo_ids...]}`
- `self.node.offsets`: `{servo_id: offset_rad}`
- `self.node.position_limits`: `{servo_id: (min, max)}`

### Usage Examples

**Basic flapping (90° phase offset):**
```bash
ros2 topic pub --once /robot_cmd std_msgs/String \
  "data: 'cmd_id:[1] func:[sine_flap] freq:[0.5] amp:[1.0] phase:[0.0] cycles:[3] sets:[1]'"
```

**Custom phase offset (180°):**
```bash
ros2 topic pub --once /robot_cmd std_msgs/String \
  "data: 'cmd_id:[1] func:[sine_flap] freq:[0.5] amp:[1.0] phase_offset:[3.14] cycles:[3] sets:[1]'"
```

**Different frequencies (even servo 2x faster):**
```bash
ros2 topic pub --once /robot_cmd std_msgs/String \
  "data: 'cmd_id:[1] func:[sine_flap] freq:[0.5] amp:[1.0] freq_ratio:[2.0] cycles:[3] sets:[1]'"
```

**Hold one servo, oscillate other:**
```bash
ros2 topic pub --once /robot_cmd std_msgs/String \
  "data: 'cmd_id:[1] func:[sine_flap] freq:[0.5] amp:[1.0] freq_ratio:[0.0] amp_ratio:[1.0] cycles:[3] sets:[1]'"
```

**Direct multi-servo control:**
```bash
ros2 topic pub --once /robot_cmd std_msgs/String \
  "data: 'cmd_id:[1] func:[drive_multi] servos:{3:0.5, 4:-0.3}'"
```

---

## Recording and Analysis (`recorder.py`)

Automated data collection script for ROS2 bag recording, CSV extraction, and plot generation organized by servo and command.

**Features:**
- Records all control ROS2 topics (`robot_cmd`, `motion_cmd`, `joint_cmd`, `joint_feedback`, `telemetry`)
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
│   ├── robot_cmd.csv
│   ├── motion_cmd.csv
│   ├── joint_cmd.csv
│   ├── joint_feedback.csv
│   └── telemetry.csv
└── plots/
    ├── master/
    │   └── plots/            # System-wide plots (all servos combined)
    │       ├── telemetry_all_positions.png
    │       ├── telemetry_all_velocities.png
    │       ├── telemetry_all_currents.png
    │       ├── telemetry_all_voltages.png
    │       ├── feedback_all_positions.png
    │       ├── feedback_all_velocities.png
    │       ├── feedback_all_currents.png
    │       ├── feedback_all_voltages.png
    │       ├── motion_cmd_ids.png
    │       └── joint_cmd_length.png
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
- Full 4-DOF kinematic model (2 servos per side: pitch + roll)
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
- 4-DOF robot with base_link and 4 revolute joints (left_pitch, left_roll, right_pitch, right_roll)
- Accurate mass and inertia properties from Fusion 360 CAD export
- Visual meshes for all links including electronics box, servo housings, aerial truss structures, and camera bracket
- Camera and IMU sensor links

---

### Gazebo Simulation Screenshots

**[Gazebo Screenshots - To Be Uploaded]**

*Reserved space for:*
- Gazebo environment with robot model
- Robot executing sine_flap gait in simulation
- Hybrid mode (real + simulated servos) visualization
- Camera view from simulated camera

---

### Launch and Usage

**Start Simulation:**
```bash
# Launch Gazebo with all simulated interfaces
ros2 launch compliant_propulsors_control gazebo_launch.py

# In another terminal, send commands (same as hardware)
ros2 topic pub --once /robot_cmd std_msgs/String \
  "data: 'cmd_id:[1] func:[sine_flap] freq:[0.5] amp:[1.0] cycles:[3] sets:[1]'"

# Record simulation data
python3 recorder.py sim_session_1
```

**Hybrid Mode (Partial Hardware):**
```bash
# Connect 2 real servos via USB, launch Gazebo
ros2 launch compliant_propulsors_control gazebo_launch.py

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
Requirements ──────────────────────────────── System Validation (Phase 3)
    │                                                    │
    └── System Design ──────────────── Integration Testing (Phase 2)
            │                                    │
            └── Component Design ──── Component Verification (Phase 1)
                       │                         │
                       └──── Implementation ────┘
                                   │
                          Continuous Regression (Phase 4)
```

**Testing Phases:**

| Phase | Scope | Methodology | Automation |
|-------|-------|-------------|------------|
| 1. Component Verification | Manufacturing, electrical, software | Plan-Driven V&V (Waterfall) | Partial — unit tests automated via CI |
| 2. Integration Testing | SIL (Gazebo), hardware-software, multi-sensor | Software/Hardware-in-the-Loop | Automated — `test_position_mode.sh`, `test_velocity_mode.sh` |
| 3. System Validation | Performance, safety, endurance | Plan-Driven V&V + Fault Injection | Partial — `recorder.py` + analysis scripts |
| 4. Continuous Regression | Performance tracking vs baseline | CI/CD (DevOps) | Fully automated — GitHub Actions |

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
- `test_position_mode.sh` — 31 position control tests (drive commands, sine_flap variations, phase offsets, waveforms, edge cases)
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
| Command-synchronized video recording | Recording starts/stops on `robot_cmd` transitions |
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
pip install dynamixel-sdk numpy pandas matplotlib --break-system-packages
```

**ROS 2 Packages:**
```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher
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
# Launch full control stack with real servos
ros2 launch compliant_propulsors_control crab_launch.py

# In another terminal, send commands
ros2 topic pub --once /robot_cmd std_msgs/String \
  "data: 'cmd_id:[1] func:[sine_flap] freq:[0.5] amp:[1.0] cycles:[3] sets:[1]'"

# Record session data
python3 recorder.py test_session_1
```

**Simulation Mode:**
```bash
# Launch Gazebo simulation
ros2 launch compliant_propulsors_control gazebo_launch.py

# Send commands (same interface as hardware)
ros2 topic pub --once /robot_cmd std_msgs/String \
  "data: 'cmd_id:[1] func:[sine_flap] freq:[0.5] amp:[1.0] cycles:[3] sets:[1]'"

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