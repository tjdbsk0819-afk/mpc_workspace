from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def nodes(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('centralized', 'distributed'):
        raise ValueError('mode must be centralized or distributed')
    common = {'scenario': LaunchConfiguration('scenario').perform(context),
              'config_path': LaunchConfiguration('config_path').perform(context)}
    plant = Node(package='antbot_mpc_ros', executable='kinematic_plant',
                 parameters=[common, {'use_sim_time': False, 'mode': mode,
                                      'output': LaunchConfiguration('output').perform(context)}], output='screen')
    controllers = []
    params = str(Path(get_package_share_directory('antbot_mpc_ros'))/'config'/'mpc.yaml')
    for index in ([0] if mode == 'centralized' else [0, 1]):
        controllers.append(Node(package='antbot_mpc_ros', executable='mpc_node',
                                namespace='' if mode == 'centralized' else ['robot_a', 'robot_b'][index],
                                name='centralized_mpc' if mode == 'centralized' else 'mpc',
                                parameters=[params, common, {'use_sim_time': True, 'mode': mode, 'robot_index': index}],
                                output='screen'))
    return [plant, *controllers, RegisterEventHandler(OnProcessExit(target_action=plant,
            on_exit=[EmitEvent(event=Shutdown(reason='Kinematic benchmark finished'))]))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='centralized'),
        DeclareLaunchArgument('scenario', default_value='A'),
        DeclareLaunchArgument('config_path', default_value=''),
        DeclareLaunchArgument('output', default_value='antbot_results/ros'),
        OpaqueFunction(function=nodes),
    ])
