"""
gazebo_launch.py — Gazebo Harmonic Simulation Launch (gz-sim 8 / ROS 2 Jazzy)
=============================================================================
Brings up the full autonomous stack against a Gazebo Harmonic simulation.

Pipeline
--------
  xacro (robot_gz.xacro)   ->  robot_description (wraps prototype_description +
                               adds gz-sim systems/sensors; absolute mesh paths)
  robot_state_publisher    ->  /robot_description + TF
  gz sim (ocean.sdf)       ->  physics + sensors (camera, IMU)
  ros_gz_sim create        ->  spawns the model from robot_description
  ros_gz_bridge            ->  bridges gz <-> ROS topics (see config/ros_gz_bridge.yaml)
       /joint_states  <-  gz joint state           (-> gazebo_dynamixel_interface)
       /imu           <-  gz IMU                    (-> gazebo_icm20948_interface)
       /camera/image_raw <- gz camera              (-> apriltag + gazebo_stellarhd)
       /prototype/<Revolute_NN>/cmd_pos -> gz position controllers
  control stack            ->  crab + controller + apriltag + sim interfaces

prototype_description is a ROS1-style CAD export (joint names "Revolute 29/31/
33/34", no gz systems).  We do NOT modify it: the gz-sim integration lives in
soft_propulsors_control — the world (worlds/ocean.sdf), the bridge config
(config/ros_gz_bridge.yaml), and a wrapper xacro (urdf/robot_gz.xacro) that
includes the description and adds the gz systems/sensors.

This uses gz-sim system plugins + ros_gz_bridge (NOT gz_ros2_control).  Joint
actuation is per-joint position commands; the gazebo_dynamixel_interface
converts the controller's joint_cmd into those per-joint commands and reads
/joint_states back as feedback.

NOTE: requires a live bring-up pass to tune PID gains, the camera pose, and to
confirm gz topic names.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

CONTROL_PACKAGE = 'soft_propulsors_control'
MODEL_NAME = 'prototype'   # must match worlds/ocean.sdf + ros_gz_bridge.yaml


def generate_launch_description():

    # gz integration assets live in soft_propulsors_control (prototype_description
    # is a ROS1 export and is left untouched). The wrapper xacro includes it.
    xacro_path = PathJoinSubstitution(
        [FindPackageShare(CONTROL_PACKAGE), 'urdf', 'robot_gz.xacro'])
    world_path = PathJoinSubstitution(
        [FindPackageShare(CONTROL_PACKAGE), 'worlds', 'ocean.sdf'])
    bridge_config = PathJoinSubstitution(
        [FindPackageShare(CONTROL_PACKAGE), 'config', 'ros_gz_bridge.yaml'])

    # Process xacro -> URDF string (mesh paths resolve to absolute file:// paths)
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)

    return LaunchDescription([

        # ------------------------------------------------------------------
        # Gazebo Harmonic
        # ------------------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
            launch_arguments={'gz_args': ['-r ', world_path]}.items(),
        ),

        # Robot state publisher (publishes robot_description + TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Spawn the robot into gz from the robot_description topic (after gz is up)
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=['-topic', 'robot_description',
                               '-name', MODEL_NAME, '-z', '0.0'],
                    output='screen',
                )
            ],
        ),

        # ros_gz bridge (gz <-> ROS topics)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),

        # ------------------------------------------------------------------
        # Mission Dispatcher + Configuration Master
        # Two fins: set 1 = left (joints 1,2), set 2 = right (joints 3,4).
        # ------------------------------------------------------------------
        Node(
            package=CONTROL_PACKAGE,
            executable='crab',
            name='crab_mission_dispatcher',
            output='screen',
            parameters=[{
                # Within each set, roll first then pitch (positional convention):
                #   set 1 = left  (2 roll, 1 pitch), set 2 = right (4 roll, 3 pitch)
                'actuator_map': '[[2, 0.0, 1, -3.14, 3.14], '
                                '[1, 0.0, 1, -1.57, 1.57], '
                                '[4, 0.0, 2, -3.14, 3.14], '
                                '[3, 0.0, 2, -1.57, 1.57]]',
                'operating_mode': 'position',
                'control_rate': 400.0,
                'startup_delay': 10.0,
                'gait_velocity': 3.77,
                'gait_effort': 0.6,
                'default_retries': 2,
                'cardinal_map': '{"N": 0, "E": 1, "S": 2, "W": 3}',
            }],
        ),

        # Autonomous Execution Engine
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

        # AprilTag Perception — reads the bridged camera image
        Node(
            package=CONTROL_PACKAGE,
            executable='apriltag_interface',
            name='apriltag_interface',
            output='screen',
            parameters=[{
                'source': 'topic',
                'image_topic': '/camera/image_raw',
                'tag_family': 'tag36h11',
                'tag_size': 0.10,
                'fx': 1000.0, 'fy': 1000.0, 'cx': 960.0, 'cy': 540.0,
            }],
        ),

        # ------------------------------------------------------------------
        # Simulated hardware interfaces (hybrid — auto-detect real hardware)
        # ------------------------------------------------------------------
        Node(
            package=CONTROL_PACKAGE,
            executable='gazebo_dynamixel_interface',
            name='gazebo_dynamixel_interface',
            output='screen',
            parameters=[{
                # CAD joint names (with spaces). servo_ids map to crab's
                # actuator_map: set 1 = left (1,2), set 2 = right (3,4).
                #   1 -> Revolute 31 (left), 2 -> Revolute 33 (left)
                #   3 -> Revolute 29 (right), 4 -> Revolute 34 (right)
                'joint_names': ['Revolute 31', 'Revolute 33',
                                'Revolute 29', 'Revolute 34'],
                'servo_ids': [1, 2, 3, 4],
                'cmd_topic_prefix': '/prototype',   # matches robot_gz.xacro + bridge
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
