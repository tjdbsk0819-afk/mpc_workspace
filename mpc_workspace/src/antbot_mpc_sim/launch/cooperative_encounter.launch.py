"""Symmetric encounter with exchange of timestamped MPC plans."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_nodes(context):
    params = Path(get_package_share_directory('mpc_planner_jackal')) / 'config/ros2_params.yaml'
    share = LaunchConfiguration('share_trajectories').perform(context).lower() == 'true'
    diagnostics = LaunchConfiguration('diagnostics').perform(context).lower() == 'true'
    length = float(LaunchConfiguration('path_length').perform(context))
    timeout = float(LaunchConfiguration('peer_trajectory_timeout').perform(context))
    velocity = float(LaunchConfiguration('reference_velocity').perform(context))
    weight = float(LaunchConfiguration('velocity_weight').perform(context))
    nodes = []
    for robot, peer in [('robot_a', 'robot_b'), ('robot_b', 'robot_a')]:
        nodes.append(Node(
            package='antbot_mpc_sim', executable='publish_test_path',
            name='test_reference_path', namespace=robot, output='screen',
            parameters=[{'use_sim_time': True, 'path_length': length}],
        ))
        nodes.append(Node(
            package='mpc_planner_jackal', executable='jackal_planner',
            name='jackal_planner', namespace=robot, output='screen',
            parameters=[str(params), {
                'use_sim_time': True, 'empty_world_test': False,
                'reference_velocity': velocity, 'velocity_weight': weight,
                'mpc_diagnostics': diagnostics,
                'peer_odom_topic': f'/{peer}/odom',
                'peer_trajectory_topic': f'/{peer}/predicted_trajectory' if share else '',
                'peer_trajectory_timeout': timeout,
                'peer_offset_x': 6.0, 'peer_offset_y': 0.0,
                'peer_yaw': 3.141592653589793, 'peer_radius': 0.5,
            }],
            remappings=[
                ('~/input/state', f'/{robot}/odom'),
                ('~/input/reference_path', f'/{robot}/reference_path'),
                ('~/output/command', f'/{robot}/cmd_vel'),
                ('~/output/predicted_trajectory', f'/{robot}/predicted_trajectory'),
                ('~/debug/peer_prediction', f'/{robot}/peer_prediction'),
            ],
        ))
    return nodes


def generate_launch_description():
    package = Path(get_package_share_directory('antbot_mpc_sim'))
    return LaunchDescription([
        DeclareLaunchArgument('path_length', default_value='8.0'),
        DeclareLaunchArgument('velocity_weight', default_value='1.0'),
        DeclareLaunchArgument('reference_velocity', default_value='0.15'),
        DeclareLaunchArgument('peer_trajectory_timeout', default_value='0.5'),
        DeclareLaunchArgument('share_trajectories', default_value='true'),
        DeclareLaunchArgument('diagnostics', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch/two_robots.launch.py')),
            launch_arguments={'encounter': 'true'}.items(),
        ),
        OpaqueFunction(function=start_nodes),
    ])
