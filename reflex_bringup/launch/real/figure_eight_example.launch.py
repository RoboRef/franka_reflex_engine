# Figure-8 example on real hardware for a single Franka Panda arm.
#
# Brings up the same stack as interactive_example.launch.py (ros2_control,
# broadcasters, reflex_multimode_controller) with figure_eight_example.yaml,
# and starts figure_eight_publisher which:
#   1. moves the arm to the start configuration (franka_move_to_start_controller,
#      goal = panda.recovery_goal_position in figure_eight_example.yaml),
#   2. tracks a figure-8 in the base XY plane (Cartesian impedance).
#
# Task parameters (amplitude, period, etc.) are set as absolute values directly
# in the figure_eight_node configuration below.
#
# Reflexes: zero-g on collision, stop on joint position / velocity limits.
# Recovery: haptic gesture (push the arm) or
#             ros2 service call /panda/recover std_srvs/srv/Empty
#           After recovery the figure-8 task restarts automatically.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip_param = 'robot_ip'
    arm_id_param = 'arm_id'
    load_gripper_param = 'load_gripper'
    use_rviz_param = 'use_rviz'

    robot_ip = LaunchConfiguration(robot_ip_param)
    arm_id = LaunchConfiguration(arm_id_param)
    load_gripper = LaunchConfiguration(load_gripper_param)
    use_rviz = LaunchConfiguration(use_rviz_param)

    config_file = PathJoinSubstitution([
        FindPackageShare('reflex_bringup'), 'config', 'real', 'figure_eight_example.yaml',
    ])

    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('reflex_bringup'), 'launch', 'real',
            'interactive_example.launch.py'])),
        launch_arguments={
            robot_ip_param: robot_ip,
            arm_id_param: arm_id,
            load_gripper_param: load_gripper,
            use_rviz_param: use_rviz,
            'config_file': config_file,
        }.items(),
    )

    figure_eight_node = Node(
        package='reflex_bringup',
        executable='figure_eight_publisher.py',
        name='figure_eight_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'arm_id': arm_id,
            'amplitude_x': 0.20,
            'amplitude_y': 0.20,
            'period': 5.0,
            'num_loops': 0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(robot_ip_param, default_value='192.168.3.102',
                              description='Hostname or IP of the Franka robot.'),
        DeclareLaunchArgument(arm_id_param, default_value='panda',
                              description='Arm ID used in all topic/frame names.'),
        DeclareLaunchArgument(load_gripper_param, default_value='false',
                              description='Attach Franka Hand gripper.'),
        DeclareLaunchArgument(use_rviz_param, default_value='false',
                              description='Launch RViz for visualisation.'),

        robot_bringup,
        # Give the controller manager time to activate reflex_multimode_controller.
        TimerAction(period=2.0, actions=[figure_eight_node]),
    ])
