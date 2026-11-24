"""GT -> original C++ MPC -> RPP adapter -> existing Swerve -> Isaac."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup = Path(get_package_share_directory('antbot_mpc_ros'))
    planner = Path(get_package_share_directory('mpc_planner_antbot'))
    navigation = Path(get_package_share_directory('antbot_navigation'))
    goals = {f'robot_{r}_goal_{axis}': str(value)
             for r, x in [('a', 3.0), ('b', -3.0)] for axis, value in [('x', x), ('y', 0.0)]}
    actions = [DeclareLaunchArgument(k, default_value=v) for k, v in goals.items()]
    actions.append(DeclareLaunchArgument('enabled', default_value='true'))
    actions.append(DeclareLaunchArgument('recovery_enabled', default_value='true'))
    for launch_file in ['two_antbot_controllers.launch.py', 'gt_peer_cv.launch.py']:
        actions.append(GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup/'launch'/launch_file)))]))
    actions.extend([
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(planner/'launch/two_antbot_mpc.launch.py')),
                                 launch_arguments={**{k: LaunchConfiguration(k) for k in goals},
                                                   'recovery_enabled': LaunchConfiguration('recovery_enabled')}.items())]),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation/'launch/isaac_rpp.launch.py')))]),
    ])
    actions.extend(Node(package='antbot_mpc_ros', executable='mpc_rpp_adapter', namespace=ego,
        parameters=[str(bringup/'config/mpc_rpp.yaml'), {'ego_robot': ego, 'peer_robot': peer,
            'enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool)}], output='screen')
        for ego, peer in [('robot_a', 'robot_b'), ('robot_b', 'robot_a')])
    return LaunchDescription(actions)
