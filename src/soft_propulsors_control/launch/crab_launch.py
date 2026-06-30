"""
crab_launch.py — Hardware System Launch File
============================================
Brings up the full autonomous stack on real hardware:

  crab        (mission dispatcher + config master)
      │  robot_config (latched)  →  controller
      │  mission_cmd             →  controller
      ▲  mission_status          ←  controller
  controller  (autonomous execution engine)
      │  joint_cmd               →  servo_actuator
      ▲  joint_feedback          ←  servo_actuator
      ▲  imu_data                ←  icm20948_imu
      ▲  apriltag_detections     ←  apriltag_interface (camera)

crab is the single source of truth for robot structure: the actuator map,
operating mode, rates, and nominal gait all live on the crab node here and are
broadcast once at startup.  Per-DOF servo settings unique to the hardware
(port, baudrate, servo PID gains) stay on the servo_actuator node.

Actuator map entry: [id, homing_offset, set_id, min_limit, max_limit, custom?]
  id                      — Dynamixel servo id
  homing_offset           — rad written to the servo's Homing Offset register by
                            the hardware node so it reads 0 at mechanical home
  set_id                  — servos sharing a set form one fin; within a set the
                            FIRST entry is roll, the SECOND is pitch
  min_limit, max_limit    — mechanical position limits (rad)
  custom (optional)       — spare per-servo value passed through for a
                            motion_command function (unused for now)

Missions are fed at runtime on the /mission_input topic — see
scripts/feed_missions.sh.
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PACKAGE = 'soft_propulsors_control'

# record_session.py lives in the workspace root, three levels up from this
# launch file (launch/ -> soft_propulsors_control/ -> src/ -> <root>).
WORKSPACE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..'))
RECORDER = os.path.join(WORKSPACE_ROOT, 'record_session.py')


def generate_launch_description():
    # Session recorder args: where to dump the run, and whether to record at all.
    session_arg = DeclareLaunchArgument(
        'session',
        default_value='session_' + datetime.now().strftime('%Y%m%d_%H%M%S'),
        description='Output folder for the recorded session (relative to cwd).')
    record_arg = DeclareLaunchArgument(
        'record', default_value='true',
        description='Record all topics and export per-mission CSVs on shutdown.')

    # Started with the stack; on Ctrl+C of the launch it gets SIGINT too, stops
    # the bag, and writes the CSVs.  sigterm/sigkill timeouts are stretched so a
    # large bag has time to finish exporting before launch force-kills it.
    recorder = ExecuteProcess(
        cmd=['python3', RECORDER, LaunchConfiguration('session')],
        output='screen',
        sigterm_timeout='60',
        sigkill_timeout='90',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('record'), "' == 'true'"])),
    )

    return LaunchDescription([
        session_arg,
        record_arg,
        recorder,

        # ------------------------------------------------------------------
        # Mission Dispatcher + Configuration Master
        # ------------------------------------------------------------------
        Node(
            package=PACKAGE,
            executable='crab',
            name='crab_mission_dispatcher',
            output='screen',
            parameters=[{
                # One fin: servo 4 = roll (±180°) first, servo 3 = pitch (±90°) second
                'actuator_map': '[[4, 0.0, 1, -3.14, 3.14], '
                                '[3, 0.0, 1, -1.57, 1.57]]',
                'operating_mode': 'position',   # 'position' or 'velocity'
                'control_rate': 400.0,          # Hz — must match controller
                'startup_delay': 10.0,          # s — deployment settling before motion
                'gait_velocity': 3.77,          # rad/s — nominal peak stroke rate (2π·f·A)
                'gait_effort': 0.6,             # rad — nominal stroke amplitude
                'default_retries': 2,           # auto-retries before asking a human
                # AprilTag id marking each compass heading (set to your printed tags)
                'cardinal_map': '{"N": 0, "E": 1, "S": 2, "W": 3}',
            }],
        ),

        # ------------------------------------------------------------------
        # Autonomous Execution Engine (state machine + outer-loop PID)
        # ------------------------------------------------------------------
        Node(
            package=PACKAGE,
            executable='controller',
            name='controller',
            output='screen',
            parameters=[{
                'kp': 0.0,
                'ki': 0.0,
                'kd': 0.0,
                'control_rate': 400.0,          # Hz — control loop rate
                'telemetry_decimation': 1,      # publish every sample
            }],
        ),

        # ------------------------------------------------------------------
        # AprilTag Perception (heading cues)
        # Consumes frames republished by stellarhd_camera — it does NOT open the
        # camera itself.  stellarhd_interface is the sole owner of camera hardware.
        # ------------------------------------------------------------------
        Node(
            package=PACKAGE,
            executable='apriltag_interface',
            name='apriltag_interface',
            output='screen',
            parameters=[{
                'source': 'topic',              # frames come from stellarhd
                'image_topic': 'camera/image_raw',
                'tag_family': 'tag36h11',
                'tag_size': 0.10,               # m — physical tag edge length
                # Intrinsics must match the published frame resolution.
                # Replace with your camera calibration.
                'fx': 1000.0, 'fy': 1000.0, 'cx': 960.0, 'cy': 540.0,
            }],
        ),

        # ------------------------------------------------------------------
        # Dynamixel XW430-T200 Hardware Interface with Inner-Loop PID
        # ------------------------------------------------------------------
        Node(
            package=PACKAGE,
            executable='Dynamixel_XW430_T200_interface',
            name='servo_actuator',
            output='screen',
            parameters=[{
                'port':       '/dev/ttyUSB0',
                'baudrate':   1000000,
                'hardware_rate': 500.0,  # Hz - write/read cycle rate
                'current_limit': 1200,

                'servo_position_p_gain': 800,
                'servo_position_i_gain': 0,
                'servo_position_d_gain': 0,
                'servo_velocity_p_gain': 100,
                'servo_velocity_i_gain': 1920,
            }],
        ),

        # ------------------------------------------------------------------
        # ICM20948 9-DOF IMU (Accelerometer + Gyroscope + Magnetometer)
        # ------------------------------------------------------------------
        Node(
            package=PACKAGE,
            executable='icm20948_interface',
            name='icm20948_imu',
            output='screen',
            parameters=[{
                'i2c_address': 0x69,      # Default I2C address
                'sample_rate': 100.0,     # Hz - IMU sampling rate
                'frame_id': 'imu_link',   # TF frame name
            }],
        ),

        # ------------------------------------------------------------------
        # StellarHD Camera — sole camera owner: records per-mission video AND
        # republishes frames on camera/image_raw for apriltag_interface.
        # ------------------------------------------------------------------
        Node(
            package=PACKAGE,
            executable='stellarhd_interface',
            name='stellarhd_camera',
            output='screen',
            parameters=[{
                'camera_index': 0,                           # /dev/video0
                'video_width': 1920,                         # Resolution
                'video_height': 1080,
                'fps': 30.0,                                 # Frames per second
                'output_directory': '/home/shafa/videos',    # Video save location
                'fourcc': 'mp4v',                            # Codec: 'mp4v', 'XVID', 'H264'
                'publish_images': True,                      # feed perception
                'image_topic': 'camera/image_raw',
                'publish_rate': 15.0,                        # Hz cap for republished frames
            }],
        ),

    ])
