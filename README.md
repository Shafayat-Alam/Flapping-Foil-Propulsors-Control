# Flex Fin

`ROS 2 Jazzy` `Ubuntu 24.04` `Jetson Orin Nano` `Dynamixel Protocol 2.0`

Autonomous control stack for a bio-inspired underwater vehicle: mission dispatch, reactive state-machine control, motion generation, and hardware interfacing. ROS 2 package name: `soft_propulsors_control`.

## Design Considerations

| Property | Implementation |
|---|---|
| Node separation | Mission logic (`crab`) and real-time control (`controller`) run as separate nodes, separate failure domains |
| Interface scope | Hardware nodes perform bus-to-ROS 2 translation only, no mission logic |
| Startup ordering | `robot_config`, `mission_cmd`, `mission_status` use `TRANSIENT_LOCAL` (latched) QoS so a late-starting subscriber never misses the first message |
| Pre-motion safety gate | `controller` refuses to move any servo, mission or manual, until one full calibration has completed since boot |
| Inter-servo synchronization | `GroupSyncWrite`/`GroupSyncRead` commands and reads all servos in one bus transaction |
| Bus ownership | A single node owns the serial port, no shared serial access |
| Fault response | `SIGINT` disables torque on servo ids 1-10 within one bus transaction |
| Simulation parity | Real servos are detected via RS-485 ping and merged with Gazebo simulation per servo ID |
| Degraded operation | Missing IMU, camera, or servo bus logs once and retries on a timer, the rest of the stack keeps running |

## Node Organization

```
┌───────────────┐  mission_cmd (latched)   ┌──────────────────────┐
│     crab      │ ───────────────────────► │      controller       │
│ (deliberative)│ ◄─────────────────────── │      (reactive)        │
│ mission queue │  mission_status (latched)│  state machine          │
└───────────────┘                          └────┬─────────┬─────────┘
                              joint_cmd/fb  │    │imu_data │apriltag_detections
                                            ▼    ▼         │
                             ┌────────────────────────┐    │
                             │   hardware interfaces   │◄───┘
                             │   (thin, bus-only)      │
                             └────────────────────────┘
```

`crab` owns *what* to do: an unbounded mission queue, override/retry policy, and the one-shot robot configuration broadcast. `controller` owns *how*: an 11-state machine, sensor fusion, gait generation, and every real-time servo command. Hardware interface nodes sit under `controller` and only move bytes between a bus and a ROS 2 topic.

## Mission Dispatch (`crab.py`)

```
mission_input ──► ┌─────────────┐ ──► mission_cmd (one active mission)
                   │ deque queue │
   override:none ──┤  (FIFO)     │
   override:discard├─►  drop current, push front
   override:requeue├─►  save current to front, push new ahead of it
                   └─────────────┘
```

| Policy | Behavior |
|---|---|
| `override:none` (default) | Enqueue at the back of the queue |
| `override:discard` | Preempt the running mission, drop it, run the new one immediately |
| `override:requeue` | Preempt the running mission, save it to the front, run the new one first |
| STUCK retry | Auto re-dispatch up to `max_retries` (default 2, per-mission `retries:`) |
| Retry exhausted | Prompt on terminal, 10 s timeout: `y` grants 2 more retries, `n`/timeout abandons the mission |

## State Machine (`controller.py`)

```
BOOT/WAIT ──calibration──► CALIBRATION ──pose within 0.05 rad──► WAIT
   │
   ▼ heading/scan mission
SCANNING ──tag detected──► LOCKING ──5 stable frames──► HEADING ──arrived──► WAIT
   ▲                          │                            │
   └────── tag lost ──────────┘                            ├─ no progress 6s ─► STUCK
                                                             └─ tag lost ────► SCANNING

FLAPPING / PADDLING / LATERAL / DRIVING ──periods complete──► WAIT
STUCK / HOVERING ──── attitude-PID hold, crab decides next move
```

| Constant | Value | Meaning |
|---|---|---|
| `ARRIVE_DISTANCE` | 0.16 m | Range to reference tag counted as arrived |
| `ALIGN_BEARING` | 0.10 rad | Bearing error counted as facing the tag |
| `TURN_BEARING` | 0.30 rad | Bearing error that triggers differential (turning) flap over synchronous cruise |
| `STABLE_FRAMES` | 5 | Consecutive valid detections required to lock a tag |
| `STUCK_WINDOW` | 6.0 s | No progress improvement in this window forces STUCK |
| `POSE_TOLERANCE` | 0.05 rad | Encoder tolerance for calibration pose settled |

Progress toward a heading mission is a weighted blend of range closure and bearing alignment:

```
progress = 0.7 * (1 - range / range_0) + 0.3 * (1 - |bearing| / bearing_0)
```

Intercardinal headings (NE, SE, SW, NW) average the bearing of the two bounding cardinal tags and take the nearer tag's range.

## Motion Math (`motion_command.py`)

Pure, stateless functions. No ROS dependency, no shared state.

```
Waveform  (t, freq, amp, phase) -> value
   sine, cosine, square, triangle, sawtooth, trapezoid, shaped_sine (sine <-> square blend via exponent k)
        │
        ▼
Servo target   drive(id, val) / drive_multi({...}) / hold(ids, val)
        │
        ▼
Gait     flap(pitch_id, heave_id, waveform=...)      pitch oscillates, roll holds/follows IMU
         paddle(pitch_id, heave_id, pitch_amp, ...)  power stroke + feathered recovery
        │
        ▼
Search   sweep(id, rate, span)   triangle-wave scan for SCANNING state
```

`harmonic_wave` extends the sinusoid with 2nd/3rd harmonic terms and a bias, used by `paddle_harmonic` for waveform-shape experiments without changing stroke amplitude (peak-normalized on every parameter change).

## Simulation (Gazebo, Software-in-the-Loop)

Three hybrid interface nodes (`gazebo_dynamixel_interface`, `gazebo_icm20948_interface`, `gazebo_stellarhd_interface`) replace the three hardware interfaces one-for-one. `crab` and `controller` publish/subscribe the exact same topic names in both modes and never know which one is running.

```
joint_cmd ──► gazebo_dynamixel_interface ──ping at startup──► real servo responds? ──► write to real bus + mirror position to Gazebo
                                                          └──► no response? ──────────► drive Gazebo joint only

feedback merge (per servo ID, every tick):
  real servo present ──► real GroupSyncRead value
  real servo absent  ──► Gazebo /joint_states value
                              │
                              ▼
                   one joint_feedback message, same wire format either way
```

| Node | Real hardware detected via | Simulated fallback |
|---|---|---|
| `gazebo_dynamixel_interface` | One-shot RS-485 ping per servo ID at startup | Gazebo `JointPositionController` per joint, bridged through `ros_gz_bridge` |
| `gazebo_icm20948_interface` | I2C open + `ICM20948` init at startup | Gazebo `/imu` passed through; magnetometer is not simulated by gz-sim, so a fixed constant vector is published instead |
| `gazebo_stellarhd_interface` | `cv2.VideoCapture` test read at startup | Gazebo `/camera/image_raw` via `cv_bridge`, same mission-segmented recording logic as the real camera node |

Detection is one-shot at node startup, not continuous. A servo ID that doesn't respond stays simulated for the life of the node; partial hardware bring-up (2 of 4 servos wired, say) needs no config change; Gazebo fills in whichever IDs aren't present.

`worlds/ocean.sdf` sets gravity to `0 0 0` as a stand-in for neutral buoyancy: the vehicle floats and fin gaits drift the body. This is a kinematic sandbox, there is no buoyancy plugin and no hydrodynamic drag model, so it validates gait *logic and joint motion*, not thrust or swimming performance.

```bash
ros2 launch soft_propulsors_control gazebo_launch.py
```

## Physical Wiring

```
                         ┌─────────────────────────┐
                         │   Jetson Orin Nano 8GB   │
                         │      Ubuntu 24.04        │
                         │      ROS 2 Jazzy         │
                         └───┬───────┬───────┬──────┘
                   RS-485    │  I2C  │  USB  │
                   (U2D2)    │ 0x69  │ UVC   │
                             │       │       │
              ┌──────────────┘   ┌───┘   ┌───┘
              ▼                  ▼       ▼
   ┌────────────────────┐  ┌─────────┐ ┌───────────┐
   │  4x XW430-T200      │  │ICM-20948│ │ StellarHD │
   │  (2 pitch, 2 roll)  │  │  9-DOF  │ │  Camera   │
   └────────────────────┘  └─────────┘ └───────────┘
```

## Power Rails

```
3S LiPo 11.1-12.6V
  ├─ Rail 1 ──► U2D2 Power Hub ──► 4x XW430-T200 servos
  └─ Rail 2 ──► Jetson Orin Nano ──► Camera (USB) + IMU (I2C)
```

## Hardware and Pinout Mapping

| Component | Model | Bus | Address / Port | Voltage | Qty |
|---|---|---|---|---|---|
| Compute | Jetson Orin Nano 8GB | N/A | N/A | 5V reg | 1 |
| Actuator | Dynamixel XW430-T200 | RS-485, Protocol 2.0 | `/dev/ttyUSB0` @ 1 Mbaud | 11.1-12.6V | 4 |
| IMU | ICM-20948 | I2C | `0x69` | 3.3V | 1 |
| Camera | StellarHD | USB (UVC) | `index 0` | 5V (bus) | 1 |
| Battery | 3S LiPo | N/A | N/A | 11.1-12.6V | 1 |

**Servo control table addresses used (Protocol 2.0):**

| Register | Address | Register | Address |
|---|---|---|---|
| Torque Enable | 64 | Goal Velocity | 104 |
| Operating Mode | 11 | Goal Position | 116 |
| Min/Max Position Limit | 48 / 52 | Present data block | 122, 25 bytes |
| Current Limit | 38 | Hardware Error Status | 70 |
| Velocity I/P Gain | 76 / 78 | Profile Velocity | 112 |
| Position D/I/P Gain | 80 / 82 / 84 | Profile Acceleration | 108 |

Homing offset is calibrated once on the servo itself (Dynamixel Wizard) and is never read or written by this node; Present Position 0 is always trusted as home.

## Data Flow

```
┌──────────────┐  joint_cmd       ┌─────────────────────┐
│  controller  │ ───────────────► │ Dynamixel_XW430_T200 │
│              │ ◄─────────────── │      _interface       │  GroupSyncWrite/Read, 1 bus txn/tick
└──────────────┘  joint_feedback  └─────────────────────┘

┌──────────────┐  imu_data        ┌──────────────────┐
│  controller  │ ◄─────────────── │ icm20948_interface│  complementary filter, 100 Hz
│              │  mag_data        │                   │
└──────────────┘ ◄─────────────── └──────────────────┘

┌──────────────┐  camera/image_raw┌────────────────────┐
│  perception  │ ◄─────────────── │ stellarhd_interface │  30 Hz capture, 15 Hz publish cap
└──────────────┘                  └────────────────────┘
```

## Wire Formats

```
joint_cmd      [id0, id1, ..., mode0, mode1, ..., val0, val1, ...]
                 mode 3.0 = position (rad)   mode 1.0 = velocity (rad/s)

joint_feedback [id, mode, pos_rad, vel_rad_s, curr_A, volt_V]   per servo

imu_data       sensor_msgs/Imu           orientation (quaternion) + ang_vel (rad/s) + accel (m/s^2)
mag_data       sensor_msgs/MagneticField field strength (Tesla), uncalibrated
camera/image_raw  sensor_msgs/Image
```

| Topic | Type | QoS | Rate |
|---|---|---|---|
| `robot_config` | `String` | latched | once at startup |
| `mission_cmd` | `String` | latched | on dispatch |
| `mission_status` | `String` | latched | 5 Hz or on event |
| `joint_cmd` | `Float32MultiArray` | volatile | as commanded |
| `joint_feedback` | `Float32MultiArray` | volatile | up to 500 Hz |
| `imu_data` | `sensor_msgs/Imu` | volatile | 100 Hz |
| `mag_data` | `sensor_msgs/MagneticField` | volatile | 100 Hz |
| `camera/image_raw` | `sensor_msgs/Image` | volatile | 15 Hz |

## Dependencies

```bash
pip install dynamixel-sdk numpy opencv-python --break-system-packages
sudo apt install ros-jazzy-ros-base
```

## Build

```bash
git clone <repository-url> soft-propulsors-control
cd soft-propulsors-control
colcon build
source install/setup.bash
```

## Run

```bash
# full autonomous stack
ros2 launch soft_propulsors_control crab_launch.py

# or individual hardware nodes for bench testing
ros2 run soft_propulsors_control Dynamixel_XW430_T200_interface \
  --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=1000000

ros2 run soft_propulsors_control icm20948_interface \
  --ros-args -p i2c_address:=0x69 -p sample_rate:=100.0

ros2 run soft_propulsors_control stellarhd_interface \
  --ros-args -p camera_index:=0 -p fps:=30.0
```

## Pre-Flight Check

```bash
ls /dev/ttyUSB*      # U2D2 adapter enumerated
i2cdetect -y 1        # 0x69 responds
```

## Node Parameters

| Node | Parameter | Default |
|---|---|---|
| Actuator | `port` | `/dev/ttyUSB0` |
| Actuator | `baudrate` | `1000000` |
| Actuator | `hardware_rate` | `100.0` Hz |
| Actuator | `current_limit` | `800` mA |
| IMU | `i2c_address` | `0x69` |
| IMU | `sample_rate` | `100.0` |
| IMU | `comp_filter_alpha` | `0.98` |
| Camera | `camera_index` | `0` |
| Camera | `video_width` x `video_height` | `1920` x `1080` |
| Camera | `fps` | `30.0` |
| crab | `control_rate` | `400.0` Hz |
| crab | `default_retries` | `2` |
| controller | `control_rate` | `50.0` Hz |
| controller | `kp`/`ki`/`kd` | `0.0` (stabilization off by default) |

## Safety

```
SIGINT (Ctrl+C) ──► torque disabled on servo ids 1-10, one bus transaction
No calibration yet ──► controller rejects every mission and manual command
5 missed servo reads ──► servo flagged LOST, logged once
Serial port unplugged ──► reopened on a 2s retry timer, config replayed on reconnect
```

## Hardware-in-the-Loop (HIL)

An external script joins the live ROS graph as one more node and drives the real vehicle through the same `mission_input` path a normal mission uses, it is not a separate control path. Real-world sensor data (load cell) closes the loop back to that script.

```
external script                      crab / controller              load_cell_interface
      │  mission_input (mission line)      │                                │
      ├───────────────────────────────────►│                                │
      │                                     │── drives real servos ────────►│
      │  mission_status (ACHIEVED)          │                                │
      │◄────────────────────────────────────┤                                │
      │  load_cell_data + joint_feedback (buffered during the run)          │
      │◄─────────────────────────────────────────────────────────────────────┤
```

Requires three nodes running concurrently: the real (or hybrid) actuator interface, `load_cell_interface`, and the normal `crab`/`controller` mission chain. The external script joins as one more node, it does not replace or embed any of them.

`load_cell_interface.py` receives a 6-axis force/torque grid over UDP (port `5005`, big-endian float32, `rows x cols` = axes x samples) from an external DAQ, and republishes it on `load_cell_data` for anything on the ROS graph to consume. It is bench/research infrastructure, not part of the autonomous mission stack; `crab` and `controller` do not subscribe to it.

