"""GT -> original C++ MPC -> MPC-native RPP -> existing Swerve -> Isaac."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from simulation.helpers.scenario_config import DEFAULT_SCENARIO, GOAL_ARGUMENTS, DRIVING_SCENARIOS


def generate_launch_description():
    bringup = Path(get_package_share_directory('antbot_mpc_ros'))
    planner = Path(get_package_share_directory('mpc_planner_antbot'))
    navigation = Path(get_package_share_directory('antbot_navigation'))
    actions = [DeclareLaunchArgument('scenario', default_value=DEFAULT_SCENARIO, choices=DRIVING_SCENARIOS)]
    actions.extend(DeclareLaunchArgument(k, default_value='',
                   description='Optional override; otherwise use the selected scenario goal')
                   for k in GOAL_ARGUMENTS)
    actions.append(DeclareLaunchArgument('enabled', default_value='true'))
    actions.append(DeclareLaunchArgument('recovery_enabled', default_value='true'))
    for launch_file in ['two_antbot_controllers.launch.py', 'gt_peer_cv.launch.py']:
        actions.append(GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup/'launch'/launch_file)))]))
    actions.extend([
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(planner/'launch/two_antbot_mpc.launch.py')),
                                 launch_arguments={**{k: LaunchConfiguration(k) for k in GOAL_ARGUMENTS},
                                                   'scenario': LaunchConfiguration('scenario'),
                                                   'recovery_enabled': LaunchConfiguration('recovery_enabled')}.items())]),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(navigation/'launch/isaac_rpp.launch.py')),
            launch_arguments={'enabled': LaunchConfiguration('enabled'),
                              'scenario': LaunchConfiguration('scenario')}.items())]),
    ])
    return LaunchDescription(actions)
