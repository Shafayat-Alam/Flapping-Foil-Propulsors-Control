# Flapping Foil Propulsors Control System

A ROS 2 Jazzy control stack for a bio-inspired blue crab robotic system with multimodal soft actuator design. The architecture follows the ROS2 Control Framework with modular decoupling between behavioral logic, motion generation, control, and hardware interfaces, enabling operation in both air and water environments.

---

## ROS2 Control Framework Architecture

```
┌──────────────┐      ┌──────────────┐      ┌─────────────────┐      ┌──────────────────────┐
│ Application  │◄────►│  Controller  │◄────►│  HW Interface   │◄────►│      Hardware        │
│  (crab.py)   │      │(controller.py)│      │(Dynamixel...py) │      │ (Servos, IMU, Cam)   │
└──────────────┘      └──────────────┘      └─────────────────┘      └──────────────────────┘
       │                      │                       │
       │                      │                       │
   robot_cmd             motion_cmd             joint_cmd
   telemetry             telemetry           joint_feedback
                              └────────── ros2_control framework ────────┘
```

*Architecture based on ROS2 Control Framework*

**Key Design Features:**
- **Telemetry-Driven Architecture:** 400 Hz closed-loop control driven by hardware feedback
- **Servo-Level Gait Control:** Motion library functions can command individual servos or synchronized groups
- **Position Limits in Motion Wire:** Single source of truth for safety limits, passed dynamically per command
- **Emergency Torque Disable:** SIGINT handler for instant servo torque shutdown on Ctrl+C
- **Per-Servo Differential Motion:** Support for phase-offset, frequency ratios, and amplitude scaling between servo pairs
- **Extensible Motion Library:** User-defined motion functions with flexible kwargs-based parameterization

---

## Node Specifications

### Application Layer (`crab.py`)
**The System "Brain" - Gait Engine and Command Orchestrator**

Manages behavioral state, command queuing, and motion function execution. Parses high-level robot commands and translates them into time-series motion trajectories.

**Responsibilities:**
- Parse and queue robot commands from `robot_cmd` topic
- Execute motion functions from motion library at 400 Hz (telemetry-driven)
- Apply servo offsets and position limits from actuator map
- Manage command sequencing with cycle-based duration control
- Support both gait-based (time-series) and direct (immediate) commands

**Input Topics:**
- `robot_cmd` (std_msgs/String): Behavioral command strings
- `telemetry` (std_msgs/Float32MultiArray): Feedback from controller triggering next gait sample

**Output Topics:**
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

**Input Topics:**
- `motion_cmd` (std_msgs/Float32MultiArray): Wire format `[cmd_id, ids, modes, values, limits]`
- `joint_feedback` (std_msgs/Float32MultiArray): Encoder feedback from hardware

**Output Topics:**
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

**Input Topics:**
- `joint_cmd` (std_msgs/Float32MultiArray): Final servo commands

**Output Topics:**
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

## Topic Specifications

| Topic | Type | Direction | Wire Format | Purpose |
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
- Records all control topics (`robot_cmd`, `motion_cmd`, `joint_cmd`, `joint_feedback`, `telemetry`)
- Supports both mcap and sqlite3 bag formats (auto-detects)
- Extracts master CSVs for all topics
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

## Testing Procedure for Basic Actuator Motion Control 

### Quick Test Scripts (Automated)

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

### Position Mode Test (`test_position_mode.sh`)

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

### Velocity Mode Test (`test_velocity_mode.sh`)

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

### Gait Execution Test (Position Mode)

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

### Comprehensive Testing Procedure (Rigorous Validation)

**Overview:**
A comprehensive 126 test validation procedure covering all system layers from hardware interface to application logic. Designed for rigorous characterization and performance verification.

**Scope:**
- **Hardware Interface (32 tests):** Serial communication, servo configuration, position/velocity control, feedback systems
- **Controller Layer (32 tests):** Command processing, PID control, limit enforcement, telemetry publishing
- **Application Layer (30 tests):** Command parsing, actuator mapping, telemetry-driven execution, motion functions
- **Motion Library (18 tests):** Direct control, waveform functions, flapping gaits, kwargs system
- **Data Recording (16 tests):** Bag recording, CSV extraction, snippet generation, plot generation
- **System Integration (12 tests):** Multi-node communication, telemetry loop integrity, launch system
- **Edge Cases (14 tests):** Boundary conditions, error handling, long-duration stability
- **Performance Metrics (12 tests):** Timing accuracy, tracking accuracy, power efficiency

**Documentation:**
Complete testing procedure with WHAT-WHY-HOW methodology. Each test includes:
- What is being tested (objective)
- Why it matters (rationale)
- How to execute (step-by-step commands)
- Pass/Fail criteria with quantitative metrics
- Debugging procedures for failures
- Expected outcomes and validation methods

**Test Methodology:**
Each test follows structured format:
1. **WHAT:** Clear test objective
2. **WHY:** Technical rationale and importance
3. **HOW:** Executable command sequence with embedded bash/python scripts
4. **PASS Criteria:** Quantitative metrics and observable outcomes
5. **WHY this proves it:** Technical explanation of validation logic
6. **FAIL Indicators:** Common failure modes and symptoms
7. **If FAIL:** Debug procedures and resolution steps

**Status:** Testing procedure documentation in progress. Part 1/4 complete (Hardware Interface layer - 32 tests). Estimated total test time: 8-12 hours for complete validation suite.

---

## Hardware Specification

| Component | Model | Protocol/Interface | Notes |
|-----------|-------|-------------------|-------|
| **Actuators** | Dynamixel XW430-T200 (×2) | RS-485, Protocol 2.0 | Position/velocity control, encoder feedback |
| **IMU** | ICM-20948 (9-Axis) | I2C | Accelerometer, gyroscope, magnetometer |
| **Vision** | ELP Fisheye USB Camera (170° FOV) | USB 2.0/3.0 | Stereo vision capability |
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

## Dependencies

**System:**
- Ubuntu 24.04 LTS
- ROS 2 Jazzy

**Python Packages:**
```bash
pip install dynamixel-sdk numpy pandas matplotlib --break-system-packages
```

**ROS 2 Packages:**
```bash
sudo apt install ros-jazzy-ros-base
```

---

## Installation and Build

```bash
# Clone repository
cd ~/
git clone <repository-url> flapping-propulsors-control
cd flapping-propulsors-control

# Install dependencies
pip install dynamixel-sdk numpy pandas matplotlib --break-system-packages

# Build ROS2 workspace
colcon build
source install/setup.bash
```

---

## Launch

```bash
# Launch full control stack
ros2 launch flapping_foil_propulsors_control crab_launch.py

# In another terminal, send commands
ros2 topic pub --once /robot_cmd std_msgs/String \
  "data: 'cmd_id:[1] func:[sine_flap] freq:[0.5] amp:[1.0] cycles:[3] sets:[1]'"

# Record session data
python3 recorder.py test_session_1
```

---

## Project Status

**Completed:**
- ✅ Telemetry-driven control architecture (400 Hz)
- ✅ Servo-level differential motion control
- ✅ Position limits in motion_cmd wire format
- ✅ Emergency torque disable (SIGINT handler)
- ✅ Motion library with kwargs extensibility
- ✅ Automated recording and plotting pipeline
- ✅ Position mode testing and validation
- ✅ Velocity mode testing and validation
- ✅ Gait execution testing (sine_flap)

**In Progress:**
- 🔄 Data analysis from completed tests
- 🔄 Development of more rigorous testing procedures
- 🔄 Sensor fusion integration (IMU + vision)
- 🔄 Closed-loop state estimation
- 🔄 Underwater testing and characterization

**Future Work:**
- Vision-based state estimation
- IMU sensor fusion
- Autonomous navigation
- Multi-environment gait optimization
- Force/torque sensing integration

---

## License

[Specify License]

---

## Acknowledgments

This project implements the ROS2 Control Framework architecture for modular robot control system design.
