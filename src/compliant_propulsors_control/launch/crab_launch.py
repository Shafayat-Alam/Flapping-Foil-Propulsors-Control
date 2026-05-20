"""
crab_launch.py — System Launch File
====================================
Starts all three nodes in the correct order:

  crab            (crab_gait_engine)
      ↓ motion_cmd
  crab_controller (crab_controller)
      ↓ joint_cmd
  servo_actuator  (DynamixelXW430Interface)

Actuator map format: [[id, offset_rad, set_id, min_limit, max_limit], ...]
  id         — Dynamixel servo ID (integer)
  offset_rad — Mechanical zero offset in radians
  set_id     — Logical group number; servos in the same set move together
               as a single DOF.  Multiple sets = multiple DOFs.
  min_limit  — Minimum position limit in radians (optional, default -3.14)
  max_limit  — Maximum position limit in radians (optional, default 3.14)
               
Example: [[3, 3.00, 1, -1.57, 1.57], [4, 3.42, 1, -3.14, 3.14]]
         Servo 3: ±90° limits (pitch), Servo 4: ±180° limits (roll)
"""

from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE = 'compliant_propulsors_control'


def generate_launch_description():
    return LaunchDescription([

        # ------------------------------------------------------------------
        # Gait Engine + Parameter Host
        # ------------------------------------------------------------------
        Node(
            package=PACKAGE,
            executable='crab',
            name='crab_gait_engine',
            output='screen',
            parameters=[{
                # Format: [[id, offset, set, min_limit, max_limit], ...]
                # Placeholder limits - update with your actual mechanical constraints
                'actuator_map': '[[3, 3.00, 1, -100, 100], [4, 3.42, 1, -100, 100]]',
                'operating_mode': 'position',  # 'position' or 'velocity'
            }],
        ),

        # ------------------------------------------------------------------
        # Controller with Outer-Loop PID (Telemetry-Driven)
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
                'telemetry_decimation': 1,  # Publish every sample
                'control_rate': 400.0,  # Hz - control loop rate
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
        # StellarHD Camera (Command-Synchronized Video Recording)
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
            }],
        ),

    ])