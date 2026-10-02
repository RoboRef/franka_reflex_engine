# Minimal real-hardware launch for a single Franka Panda arm.
# Default behaviour: gravity compensation (low-stiffness joint impedance).
# Reflexes: zero-g on collision, stop on joint limits.
# Recovery: haptic gesture (push the arm) or call /<arm_id>/recover service.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_param        = 'robot_ip'
    arm_id_param          = 'arm_id'
    load_gripper_param    = 'load_gripper'
    use_fake_hw_param     = 'use_fake_hardware'
    fake_sensor_param     = 'fake_sensor_commands'
    use_rviz_param        = 'use_rviz'
    config_file_param     = 'config_file'

    robot_ip     = LaunchConfiguration(robot_ip_param)
    arm_id       = LaunchConfiguration(arm_id_param)
    load_gripper = LaunchConfiguration(load_gripper_param)
    use_fake_hw  = LaunchConfiguration(use_fake_hw_param)
    fake_sensor  = LaunchConfiguration(fake_sensor_param)
    use_rviz     = LaunchConfiguration(use_rviz_param)

    # --- URDF / Xacro ---
    xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'real', 'panda_arm.urdf.xacro')
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' hand:=',                load_gripper,
        ' robot_ip:=',            robot_ip,
        ' arm_id:=',              arm_id,
        ' use_fake_hardware:=',   use_fake_hw,
        ' fake_sensor_commands:=', fake_sensor,
    ])

    # --- Controller config ---
    default_config = PathJoinSubstitution([
        FindPackageShare('reflex_bringup'),
        'config', 'real', 'interactive_example.yaml',
    ])
    config_file = LaunchConfiguration(config_file_param, default=default_config)

    # --- RViz ---
    rviz_file = os.path.join(
        get_package_share_directory('franka_description'),
        'rviz', 'visualize_franka.rviz')

    return LaunchDescription([
        # ── Launch arguments ──────────────────────────────────────────────
        DeclareLaunchArgument(robot_ip_param,
            default_value='172.16.0.2',
            description='Hostname or IP of the Franka robot.'),
        DeclareLaunchArgument(arm_id_param,
            default_value='panda',
            description='Arm ID used in all topic/frame names.'),
        DeclareLaunchArgument(load_gripper_param,
            default_value='true',
            description='Attach Franka Hand gripper.'),
        DeclareLaunchArgument(use_fake_hw_param,
            default_value='false',
            description='Use mock hardware (for offline testing).'),
        DeclareLaunchArgument(fake_sensor_param,
            default_value='false',
            description='Fake sensor commands (only valid with use_fake_hardware:=true).'),
        DeclareLaunchArgument(use_rviz_param,
            default_value='false',
            description='Launch RViz for visualisation.'),
        DeclareLaunchArgument(config_file_param,
            default_value=default_config,
            description='Path to controller YAML config.'),

        # ── ros2_control_node — 1 kHz control loop ────────────────────────
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, config_file],
            output='screen',
            prefix=['stdbuf -o L'],
            on_exit=Shutdown(),
        ),

        # ── Robot state publisher ─────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # ── Broadcasters ──────────────────────────────────────────────────
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_state_broadcaster'],
            output='screen',
            condition=UnlessCondition(use_fake_hw),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_model_broadcaster'],
            output='screen',
            condition=UnlessCondition(use_fake_hw),
        ),

        # ── Reflex MC controller (active) ─────────────────────────────────
        # Starts in gravity-comp mode (low-stiffness joint impedance).
        # The SimpleReflexEngine monitors for collisions and joint limits
        # and switches sub-controllers automatically.
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['reflex_multimode_controller'],
            output='screen',
            condition=UnlessCondition(use_fake_hw),
        ),

        # ── Gripper (optional) ────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={
                robot_ip_param:    robot_ip,
                use_fake_hw_param: use_fake_hw,
            }.items(),
            condition=IfCondition(load_gripper),
        ),

        # ── RViz (optional) ───────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['--display-config', rviz_file],
            condition=IfCondition(use_rviz),
        ),
    ])
