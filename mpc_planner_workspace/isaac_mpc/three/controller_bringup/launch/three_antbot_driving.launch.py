"""Three identical decentralized planners; each observes both other robots."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from simulation.robot_layout import DEFAULT_LAYOUT,load_layout


def nodes(context):
    layout=load_layout(LaunchConfiguration('layout_file').perform(context),LaunchConfiguration('scenario').perform(context))
    enabled=LaunchConfiguration('enabled').perform(context).lower()=='true'
    recovery=LaunchConfiguration('recovery_enabled').perform(context).lower()=='true'
    bringup=Path(get_package_share_directory('antbot_mpc_ros'))
    planner=Path(get_package_share_directory('mpc_planner_antbot'))
    nav=Path(get_package_share_directory('antbot_navigation'))
    actions=[GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup/'launch/three_antbot_controllers.launch.py')))]),
             GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav/'launch/three_antbot_rpp.launch.py')),
                         launch_arguments={'enabled':str(enabled).lower()}.items())])]
    for ego,configuration in layout.items():
        peers=[p for p in layout if p!=ego]
        for peer in peers:
            actions.append(Node(package='antbot_mpc_ros',executable='gt_peer_predictor',namespace=ego,
                name='gt_'+peer,parameters=[str(bringup/'config/gt_peer_cv.yaml'),
                    {'ego_robot':ego,'peer_robot':peer,'peer_odom_topic':f'/{peer}/odom'}],
                remappings=[(f'/{ego}/peer_state',f'/{ego}/peers/{peer}/state'),
                            (f'/{ego}/peer_prediction',f'/{ego}/peers/{peer}/prediction')],output='screen'))
        actions.append(Node(package='mpc_planner_antbot',executable='antbot_planner',namespace=ego,
            parameters=[str(planner/'config/recovery.yaml'),{'use_sim_time':True,'ego_robot':ego,
                'peer_robots':peers,'goal':[float(x) for x in configuration['goal']],
                'recovery_enabled':recovery}],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('layout_file',default_value=str(DEFAULT_LAYOUT)),
        DeclareLaunchArgument('scenario',default_value='staggered'),
        DeclareLaunchArgument('enabled',default_value='true'),
        DeclareLaunchArgument('recovery_enabled',default_value='true'),
        OpaqueFunction(function=nodes)])
