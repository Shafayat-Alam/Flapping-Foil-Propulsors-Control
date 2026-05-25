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
8. [Testing and Validation](#testing-and-validation)
   - [Quick Functional Testing](#quick-functional-testing-automated-scripts)
   - [Gazebo Simulation Testing](#gazebo-simulation-testing)
   - [Comprehensive System Validation](#comprehensive-system-validation)
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

## Hardware Specification

| Component | Model | Protocol/Interface | Notes |
|-----------|-------|-------------------|-------|
| **Actuators** | Dynamixel XW430-T200 (×2) | RS-485, Protocol 2.0 | Position/velocity control, encoder feedback |
| **IMU** | Adafruit ICM-20948 (9-Axis) | I2C (address 0x69) | Accelerometer, gyroscope, magnetometer @ 100 Hz |
| **Vision** | DWE StellarHD USB Camera | USB 2.0/3.0, OpenCV | 1920×1080 @ 30 fps, synchronized video recording |
| **Compute** | NVIDIA Jetson Orin Nano (8GB) | — | ROS 2 Jazzy, Ubuntu 24.04 |
| **Power** | 12V LiPo Battery | — | Servo power supply |

**Future Sensor Fusion:**
Closed-loop control will integrate vision and IMU data for state estimation, enabling position/orientation feedback and autonomous navigation in both air and water environments.

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

## Gazebo Simulation

### Overview

A Gazebo Harmonic simulation environment for kinematic testing and gait development without physical hardware. The simulation provides a **hybrid architecture** that auto-detects real servos and seamlessly merges physical and simulated feedback.

**Simulation Capabilities:**
- Full 4-DOF kinematic model (2 servos per side)
- Visual rendering of robot geometry and motion
- Simulated IMU (9-axis accelerometer/gyroscope/magnetometer)
- Simulated camera (stereo fisheye, 170° FOV)
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
   - Simulates ELP Fisheye camera (170° FOV)

**URDF Model:**
- 4-DOF robot with base_link and 4 revolute joints
- Accurate mass and inertia properties from CAD
- Visual meshes for flipper geometry
- Camera and IMU sensor links

---

### Gazebo Simulation Screenshots

**[Gazebo Screenshots - To Be Uploaded]**

*Reserved space for:*
- Gazebo environment with robot model
- Robot executing sine_flap gait in simulation
- Hybrid mode (real + simulated servos) visualization
- Camera view from simulated fisheye camera

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

### Simulation Workflow

**Typical Development Cycle:**

1. **Develop in Simulation:**
   - Create new motion functions in motion_library.py
   - Test gaits in Gazebo (no hardware risk)
   - Visualize trajectories and timing
   - Record simulation data for analysis

2. **Validate in Hybrid Mode:**
   - Connect 1-2 real servos
   - Verify motion on real hardware
   - Compare real vs simulated feedback
   - Identify hardware-specific behavior

3. **Deploy to Full Hardware:**
   - Switch to hardware launch file
   - Run full validation tests
   - Characterize performance differences
   - Refine motion parameters

---

### Gazebo Testing and Validation

**Simulation Environment Validation:**

The Gazebo simulation environment itself requires validation before it can be used as a testing platform for control algorithms. This validation ensures the simulation accurately represents the kinematic model and that all interfaces function correctly.

**Test Coverage:**

1. **Launch Verification**
   - Gazebo startup without errors
   - Robot model visibility and mesh loading
   - URDF parsing and joint configuration

2. **Joint Command Response**
   - Simulated servos respond to position commands
   - Commanded values match joint_states feedback
   - All 4 joints independently controllable

3. **Feedback Loop Integrity**
   - Telemetry publishes at 400 Hz target rate
   - No dropped messages or rate instability
   - Feedback synchronization with commanded trajectories

4. **Hybrid Mode Detection**
   - Auto-detection of connected real servos
   - Correct partition of real vs simulated servo IDs
   - Merged feedback stream from both sources

5. **Visual-Feedback Consistency**
   - Gazebo visualization matches telemetry data
   - Joint angles in GUI correspond to encoder feedback
   - Camera and IMU ROS2 topics publish at expected rates

**Acceptance Criteria:**
- All 4 joints respond to position commands within 100 ms
- Telemetry publish rate: 400 ± 10 Hz sustained over 5 minutes
- Zero console errors during continuous sine_flap execution
- Hybrid mode correctly identifies and merges real servo feedback

**Known Issues Documented:**
- URDF mesh loading failures require correct file path configuration in prototype.xacro
- Joint controller startup requires 3-second delay (implemented in launch file)
- Gazebo physics instability at high frequencies (>5 Hz gaits) due to solver limitations
- Camera/IMU ROS2 topic latency: 30-50 ms behind joint states (expected simulation overhead)

---

### Simulation Validation

**Comparison Metrics:**
- Trajectory tracking error (sim vs hardware)
- Phase offset accuracy between servo pairs
- Timing consistency (control loop jitter)
- Command latency (cmd → execution)

**Known Discrepancies:**
- Simulated servos have zero backlash/deadband
- No current draw or thermal effects in simulation
- Gravity effects simplified (no flipper droop)
- No mechanical compliance or structural damping

**Use simulation data as baseline, not ground truth for hardware performance.**

---

## Testing and Validation

### Testing Overview

**Two-Tier Testing Methodology:**

1. **Quick Functional Testing** (Automated Scripts)
   - Position mode validation (`test_position_mode.sh` - 31 tests)
   - Velocity mode validation (`test_velocity_mode.sh` - 32 tests)
   - Basic gait execution verification
   - **Purpose:** Rapid smoke testing for basic functionality
   - **Time:** ~15-30 minutes per mode
   - **When:** After code changes, before comprehensive validation

2. **Comprehensive System Validation** (Rigorous Characterization)
   - **Software Testing:** Unit tests, system identification, control loop characterization
   - **Hardware Validation:** Electrical testing, mechanical testing, calibration procedures
   - **Environmental Testing:** Air vs water characterization, thermal performance
   - **Safety & Failure Modes:** Emergency response, fault injection, failsafe verification
   - **Data Quality Assurance:** Synchronization validation, regression testing
   - **Purpose:** Full system characterization for research publication and deployment
   - **Time:** ~8-12 hours complete suite
   - **When:** Major milestones, pre-deployment, research documentation

---

### Quick Functional Testing (Automated Scripts)

**Test Script Generator (`generate_test_scripts.py`):**

Automated Python script that generates bash test sequences for position and velocity modes.

**Features:**
- Generates `test_position_mode.sh` with 31 position control tests
- Generates `test_velocity_mode.sh` with 32 velocity control tests
- Parameterized test commands with configurable delays
- Covers: drive commands, sine_flap variations, phase offsets, frequency/amplitude ratios, waveforms, edge cases

**Usage:**
```bash
python3 generate_test_scripts.py
# Generates test_position_mode.sh and test_velocity_mode.sh

# Run position mode tests
./test_position_mode.sh

# Run velocity mode tests (after changing operating_mode to 'velocity' in launch file)
./test_velocity_mode.sh
```

**Generated Test Coverage:**
- Individual servo control (drive commands)
- Basic sine_flap gaits with varying parameters
- Phase offset variations (up/down flap, 180° shifts)
- Frequency and amplitude ratio tests
- Hold modes and differential motion
- Waveform function tests
- Edge cases (low frequency, small amplitude, position limits)
- Rapid command sequences

---

#### Position Mode Test (`test_position_mode.sh`)

**What it tests:**
- Position control accuracy and tracking
- Servo response to commanded positions
- Position limit enforcement
- Offset application and calibration

**What insights it provides:**
- Steady-state position error
- Settling time and overshoot
- Position limit clamping behavior
- Servo mechanical zero accuracy

**Procedure:**
1. Calibration (move to zero offsets)
2. Step commands to various positions within limits
3. Boundary testing (min/max limits)
4. Return to zero

**Status:** Tests completed. Data being analyzed for position accuracy, settling time, and steady-state error characterization.

---

#### Velocity Mode Test (`test_velocity_mode.sh`)

**What it tests:**
- Velocity control accuracy
- Velocity tracking performance
- Position limit enforcement in velocity mode (auto-stop at boundaries)
- Velocity ramping and transitions

**What insights it provides:**
- Velocity tracking error
- Response to velocity step changes
- Boundary detection and stopping behavior
- Acceleration/deceleration characteristics

**Procedure:**
1. Calibration
2. Constant velocity commands (positive/negative)
3. Velocity ramps and steps
4. Boundary approach testing (velocity zeros at limits)

**Status:** Tests completed. Data being analyzed for velocity tracking accuracy and boundary enforcement behavior.

---

#### Gait Execution Test (Position Mode)

**What it tests:**
- Sine flap motion function execution
- Differential servo control (phase offsets)
- Continuous trajectory tracking
- Telemetry-driven loop performance

**What insights it provides:**
- Phase offset accuracy between servo pairs
- Amplitude and frequency accuracy
- Control loop jitter and timing consistency
- Servo synchronization

**Procedure:**
1. Execute sine_flap with default 90° phase offset
2. Record 3 cycles at 0.5 Hz
3. Verify servo motion via encoder feedback
4. Extract telemetry data for analysis

**Status:** Tests completed. Data being analyzed for phase accuracy, amplitude tracking, and inter-servo synchronization.

---

### Gazebo Simulation Testing

**Purpose:**
Validate control logic and motion library functions in simulation before hardware testing. Reduces hardware wear and enables rapid parameter iteration.

**Prerequisites:**
Run Gazebo environment validation tests (see Gazebo Testing and Validation subsection) to ensure simulation is functioning correctly before using it for control testing.

**Test Workflow:**
```bash
# 0. Verify Gazebo environment first (see Gazebo section)
ros2 launch compliant_propulsors_control gazebo_launch.py
# Confirm all 5 environment validation tests pass

# 1. Run automated test scripts (same as hardware)
./test_position_mode.sh  # Uses simulated feedback
./test_velocity_mode.sh

# 2. Record simulation data
python3 recorder.py sim_validation_session

# 3. Compare simulation vs hardware results
python3 compare_sim_hardware.py sim_validation_session/ hardware_session/
```

**What Simulation Testing Validates:**
- Motion library function logic and trajectory generation
- Command parsing and actuator mapping correctness
- Telemetry-driven control loop timing (software only)
- Phase offset and differential motion calculations
- Position limit enforcement logic

**What Simulation CANNOT Validate:**
- Servo PID tuning and tracking accuracy (hardware-dependent)
- Current draw and thermal behavior
- Mechanical backlash and compliance effects
- Sensor noise and communication errors
- Real-world timing jitter and latency

**Simulation as Pre-Hardware Gate:**
- All new motion functions MUST pass simulation tests before hardware deployment
- Gait parameters tuned in simulation, refined on hardware
- Simulation tests run in CI/CD pipeline for regression detection

---

### Comprehensive System Validation

**Overview:**
Rigorous multi-layer validation covering software, hardware, environmental, and safety testing. Designed for research-grade characterization and deployment readiness.

**Scope:**
- **Software Testing (32 tests):** Unit tests, motion library validation, command parsing, telemetry loop integrity
- **Hardware Interface (32 tests):** Serial communication, servo configuration, position/velocity control, feedback systems
- **Controller Layer (32 tests):** Command processing, PID control, limit enforcement, telemetry publishing
- **Application Layer (30 tests):** Command parsing, actuator mapping, telemetry-driven execution, motion functions
- **System Identification (18 tests):** Frequency response, step response, tracking error, timing jitter
- **Hardware Validation (16 tests):** Electrical testing, mechanical testing, calibration, environmental characterization
- **Safety & Failure Modes (14 tests):** Emergency response, fault injection, failsafe verification
- **Data Quality Assurance (12 tests):** Synchronization validation, regression testing, automated validation

**Total:** 186 tests across 8 validation domains

**Test Methodology:**
Each test follows structured format:
1. **WHAT:** Clear test objective
2. **WHY:** Technical rationale and importance
3. **HOW:** Executable command sequence with embedded bash/python scripts
4. **PASS Criteria:** Quantitative metrics and observable outcomes
5. **WHY this proves it:** Technical explanation of validation logic
6. **FAIL Indicators:** Common failure modes and symptoms
7. **If FAIL:** Debug procedures and resolution steps

**Status:** Testing procedure documentation in progress. Estimated total test time: 8-12 hours for complete validation suite.

---

### Software Testing

**Unit Testing Framework:**
```bash
# Install test dependencies
pip install pytest pytest-ros unittest --break-system-packages

# Run unit tests
pytest test/
```

**Test Coverage:**
- Motion library function validation with mock hardware
- Controller PID logic verification with synthetic feedback
- Command parsing and actuator mapping tests
- Telemetry loop timing validation

**Example Unit Test Structure:**
```python
# test/test_motion_library.py
def test_sine_flap_phase_offset():
    """Verify 90° phase offset between servo pairs"""
    motion = MotionLibrary(mock_node)
    result = motion.sine_flap(t=0, freq=1.0, amp=1.0, phase=0.0)
    assert abs(result[3] - result[4]) == pytest.approx(pi/2, abs=0.01)
```

---

### System Identification

**Control Loop Characterization:**
- **Step Response:** Rise time, overshoot, settling time measurement
- **Frequency Response:** Bandwidth and phase margin from sine sweeps
- **Tracking Error:** RMS error vs. frequency and amplitude
- **Timing Jitter:** Control loop execution time histogram

**Automated System ID Script:**
```bash
# Generate frequency sweep test
python3 generate_sysid_tests.py --mode position --freq_range 0.1,10.0 --steps 20
./run_sysid_sweep.sh
python3 analyze_bode_plot.py session_name/  # Generates Bode plots from CSV
```

**Key Metrics:**
- Position mode bandwidth (expected: ~5-10 Hz @ -3dB)
- Velocity mode bandwidth (expected: ~10-20 Hz @ -3dB)
- Steady-state error (<0.05 rad for position, <0.1 rad/s for velocity)
- Control loop jitter (<1 ms std dev @ 400 Hz)

---

### Hardware Validation

**Electrical Testing:**
| Test | Method | Acceptance Criteria |
|------|--------|---------------------|
| Power supply stability | Oscilloscope @ servo terminals during motion | <5% voltage ripple |
| Current draw | Ammeter in series with battery | <2A peak per servo |
| RS-485 signal integrity | Logic analyzer @ 1 Mbaud | No errors over 1 hour |
| Thermal performance | IR thermometer after 10 min operation | <60°C servo case temp |

**Mechanical Testing:**
| Test | Method | Acceptance Criteria |
|------|--------|---------------------|
| Flipper deflection | High-speed camera + OpenCV tracking | <5° deviation from rigid body |
| Bearing wear | Visual inspection after 1000 cycles | No visible scoring or play |
| Waterproofing | Submersion test, leak indicator paper | No moisture ingress |
| Structural fatigue | 10,000 cycle stress test @ max amplitude | No cracks or delamination |

**Calibration Procedures:**
```bash
# Servo zero calibration (mechanical reference point)
ros2 topic pub --once /robot_cmd std_msgs/String \
  "data: 'cmd_id:[0] func:[drive_multi] servos:{3:0.0, 4:0.0}'"
# Manually align flippers to reference marks, record encoder values

# IMU calibration (6-position tumble test)
ros2 run compliant_propulsors_control imu_calibrate
# Follow on-screen instructions for bias and scale factor estimation
```

---

### Safety and Failure Modes

**Emergency Response Validation:**
```bash
# Test SIGINT torque disable (must respond <100ms)
ros2 launch compliant_propulsors_control crab_launch.py
# During motion, press Ctrl+C
# PASS: Servos disable instantly, no coasting
# FAIL: Delayed response or continued motion

# Test position limit enforcement
ros2 topic pub --once /robot_cmd std_msgs/String \
  "data: 'cmd_id:[1] func:[drive] servo:[3] value:[15.0]'"
# PASS: Servo clamps at max_limit from actuator_map
# FAIL: Servo attempts to exceed limit or crashes
```

**Fault Injection Tests:**
- Communication loss: Unplug USB mid-motion, verify failsafe state
- Power brownout: Drop voltage to 10V, verify torque disable
- Overcurrent: Block flipper mechanically, verify current limiting
- Sensor failure: Disconnect encoder, verify safe degradation

---

### Data Quality Assurance

**Automated Validation:**
```python
# analyze_data_quality.py
def validate_telemetry(csv_path):
    """Check for dropped samples, outliers, timestamp jitter"""
    df = pd.read_csv(csv_path)
    
    # Check for missing samples (should be ~400 Hz)
    dt = df['time'].diff()
    assert dt.mean() < 0.0026, f"Sample rate low: {1/dt.mean():.1f} Hz"
    
    # Check for outliers (>3σ from mean)
    for col in ['position', 'velocity', 'current']:
        z_scores = np.abs((df[col] - df[col].mean()) / df[col].std())
        assert (z_scores > 3).sum() < len(df) * 0.01, f"Outliers in {col}"
    
    print("✓ Data quality PASS")
```

**Regression Testing:**
- Maintain reference recordings for each gait function
- Automated CSV comparison on code changes
- Alert on >5% performance degradation vs. baseline

---

### Continuous Integration

**GitHub Actions Workflow (`.github/workflows/ros2_test.yml`):**
```yaml
name: ROS2 Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v3
      - name: Install ROS2 Jazzy
        run: |
          sudo apt update
          sudo apt install -y ros-jazzy-ros-base python3-pytest
      - name: Build workspace
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon build
      - name: Run unit tests
        run: |
          source install/setup.bash
          pytest test/ -v
```

**Pre-commit Hooks:**
- Linting (flake8, black)
- Unit test execution
- Launch file syntax validation

---

### Performance Benchmarks

**Baseline Metrics (Hardware: Jetson Orin Nano 8GB):**
| Metric | Target | Measured |
|--------|--------|----------|
| Control loop rate | 400 Hz |  |
| Control loop jitter | <1 ms std |  |
| Position tracking error | <0.05 rad RMS |  |
| Velocity tracking error | <0.1 rad/s RMS |  |
| ROS2 topic latency (cmd→fb) | <5 ms |  |
| CPU usage (3 nodes) | <30% |  |
| Memory footprint | <500 MB |  |

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