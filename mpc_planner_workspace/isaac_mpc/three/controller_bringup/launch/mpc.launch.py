"""Controllers only: use when an external plant supplies /clock and /odom."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('centralized', 'distributed'):
        raise ValueError('Invalid mode')
    common = {'use_sim_time': True, 'mode': mode,
              'scenario': LaunchConfiguration('scenario').perform(context),
              'config_path': LaunchConfiguration('config_path').perform(context),
              'world_frame': LaunchConfiguration('world_frame').perform(context)}
    params = str(Path(get_package_share_directory('antbot_mpc_ros'))/'config'/'mpc.yaml')
    return [Node(package='antbot_mpc_ros', executable='mpc_node',
                 name='centralized_mpc' if mode == 'centralized' else 'mpc',
                 namespace='' if mode == 'centralized' else ['robot_a', 'robot_b'][m],
                 parameters=[params, common, {'robot_index': m}], output='screen')
            for m in ([0] if mode == 'centralized' else [0, 1])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='centralized'),
        DeclareLaunchArgument('scenario', default_value='A'),
        DeclareLaunchArgument('config_path', default_value=''),
        DeclareLaunchArgument('world_frame', default_value='world'),
        OpaqueFunction(function=nodes),
    ])
