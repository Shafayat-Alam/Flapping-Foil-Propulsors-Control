"""
gazebo_launch.py — Gazebo Simulation Launch File (Harmonic/Jazzy)
===================================================================
Starts Gazebo Harmonic simulation with all simulated interfaces.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

CONTROL_PACKAGE = 'compliant_propulsors_control'
URDF_PACKAGE = 'prototype_description'


def generate_launch_description():
    
    # Get URDF and SDF file paths
    pkg_share = FindPackageShare(URDF_PACKAGE).find(URDF_PACKAGE)
    urdf_file = os.path.join(pkg_share, 'urdf', 'prototype.urdf')
    sdf_file = os.path.join(pkg_share, 'urdf', 'robot.sdf')
    
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        
        # ------------------------------------------------------------------
        # Gazebo Harmonic (gz sim)
        # ------------------------------------------------------------------
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),
        
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        
        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),
        
        # Spawn robot from SDF file (delayed to let Gazebo start)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['gz', 'service', '-s', '/world/empty/create',
                         '--reqtype', 'gz.msgs.EntityFactory',
                         '--reptype', 'gz.msgs.Boolean',
                         '--timeout', '1000',
                         '--req', f'sdf_filename: "{sdf_file}", name: "robot"'],
                    output='screen'
                )
            ]
        ),
        
        # ------------------------------------------------------------------
        # Core nodes (UNCHANGED from hardware)
        # ------------------------------------------------------------------
        Node(
            package=CONTROL_PACKAGE,
            executable='crab',
            name='crab_gait_engine',
            output='screen',
            parameters=[{
                'actuator_map': '[[1, 0.0, 1, -1.57, 1.57], [2, 0.0, 1, -3.14, 3.14], [3, 0.0, 2, -1.57, 1.57], [4, 0.0, 2, -3.14, 3.14]]',
                'operating_mode': 'position',
            }],
        ),
        
        Node(
            package=CONTROL_PACKAGE,
            executable='controller',
            name='controller',
            output='screen',
            parameters=[{
                'kp': 0.0,
                'ki': 0.0,
                'kd': 0.0,
                'telemetry_decimation': 1,
                'control_rate': 400.0,
            }],
        ),
        
        # ------------------------------------------------------------------
        # Simulated hardware interfaces (hybrid - auto-detect real hardware)
        # ------------------------------------------------------------------
        Node(
            package=CONTROL_PACKAGE,
            executable='gazebo_dynamixel_interface',
            name='gazebo_dynamixel_interface',
            output='screen',
            parameters=[{
                'joint_names': ['left_joint_1', 'left_joint_2', 'right_joint_1', 'right_joint_2'],
                'servo_ids': [1, 2, 3, 4],
            }],
        ),
        
        Node(
            package=CONTROL_PACKAGE,
            executable='gazebo_icm20948_interface',
            name='gazebo_icm20948_interface',
            output='screen',
            parameters=[{
                'frame_id': 'imu_link',
                'gazebo_imu_topic': '/imu',
            }],
        ),
        
        Node(
            package=CONTROL_PACKAGE,
            executable='gazebo_stellarhd_interface',
            name='gazebo_stellarhd_interface',
            output='screen',
            parameters=[{
                'gazebo_camera_topic': '/camera/image_raw',
                'output_directory': '/home/shafa/videos_sim',
                'fps': 30.0,
                'fourcc': 'mp4v',
            }],
        ),
        
    ])
