# Figure-8 simulation launch for a single Franka Panda arm in MuJoCo.
#
# Launches MuJoCo server, ros2_control, state broadcasters,
# reflex_multimode_controller, and starts figure_eight_publisher which:
#   1. moves the arm to the start configuration (franka_move_to_start_controller,
#      goal = panda.recovery_goal_position in figure_eight_sim_controllers.yaml),
#   2. tracks a figure-8 in the base XY plane (Cartesian impedance).
#
# Task parameters (amplitude, period, etc.) are set as absolute values directly
# in the figure_eight_node configuration below.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def concatenate_ns(ns1, ns2, absolute=False):
    if len(ns1) == 0:
        return ns2
    if len(ns2) == 0:
        return ns1
    if ns1[0] == '/':
        ns1 = ns1[1:]
    if ns1[-1] == '/':
        ns1 = ns1[:-1]
    if ns2[0] == '/':
        ns2 = ns2[1:]
    if ns2[-1] == '/':
        ns2 = ns2[:-1]
    if absolute:
        ns1 = '/' + ns1
    return ns1 + '/' + ns2


def generate_launch_description():
    arm_id_param = 'arm_id'
    initial_positions_param = 'initial_positions'
    use_rviz_param = 'use_rviz'
    config_file_param = 'config_file'

    default_config = os.path.join(
        get_package_share_directory('reflex_bringup'), 'config', 'sim',
        'figure_eight_sim_controllers.yaml'
    )

    arm_id = LaunchConfiguration(arm_id_param)
    initial_positions = LaunchConfiguration(initial_positions_param)
    use_rviz = LaunchConfiguration(use_rviz_param)
    config_file = LaunchConfiguration(config_file_param, default=default_config)

    load_gripper = False
    scene_file = 'scene.xml' if load_gripper else 'scene_ng.xml'
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'), 'robots', 'sim',
        'panda_arm_sim.urdf.xacro'
    )
    xml_file = os.path.join(
        get_package_share_directory('franka_description'), 'mujoco', 'franka', scene_file
    )
    franka_bringup_path = get_package_share_directory('reflex_bringup')
    ns = ''

    # Robot state publisher setup
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', franka_xacro_file,
        ' arm_id:=', arm_id,
        ' hand:=', str(load_gripper).lower(),
        ' initial_positions:=', initial_positions
    ])

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=ns,
        parameters=[{'robot_description': robot_description}]
    )

    # Joint state publisher setup
    jsp_source_list = [concatenate_ns(ns, 'joint_states', True)]
    if load_gripper:
        jsp_source_list.append(concatenate_ns(ns, 'panda/franka_gripper/joint_states', True))

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=ns,
        parameters=[{
            'source_list': jsp_source_list,
            'rate': 30
        }],
    )

    rviz_file = os.path.join(
        get_package_share_directory('franka_description'), 'rviz', 'visualize_franka.rviz'
    )

    figure_eight_node = Node(
        package='reflex_bringup',
        executable='figure_eight_publisher.py',
        name='figure_eight_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'arm_id': arm_id,
            'amplitude_x': 0.20,
            'amplitude_y': 0.20,
            'period': 5.0,
            'num_loops': 0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            use_rviz_param,
            default_value='false',
            description='Visualize the robot in RViz'
        ),
        DeclareLaunchArgument(
            arm_id_param,
            default_value='panda',
            description='The name of the robot. Defaults to panda.'
        ),
        DeclareLaunchArgument(
            initial_positions_param,
            default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
            description='Initial joint positions of the robot in MuJoCo (Franka home pose).'
        ),
        DeclareLaunchArgument(
            config_file_param,
            default_value=default_config,
            description='Path to the controller YAML configuration.'
        ),

        # MuJoCo ROS server launch
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(os.path.join(franka_bringup_path, 'launch', 'sim', 'launch_mujoco_ros_server.launch')),
            launch_arguments={
                'use_sim_time': 'true',
                'modelfile': xml_file,
                'verbose': 'true',
                'ns': ns,
                'unpause': 'true',
                'mujoco_plugin_config': config_file,
            }.items()
        ),

        node_robot_state_publisher,
        node_joint_state_publisher,

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_state_broadcaster', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_robot_model_broadcaster', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--display-config', rviz_file],
            condition=IfCondition(use_rviz)
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['reflex_multimode_controller', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),

        # Start the figure-8 task node after controller manager brings up the controllers
        TimerAction(period=5.0, actions=[figure_eight_node]),
    ])
