"""Two independent GT-to-CV observers; no robot control commands are produced."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    params = LaunchConfiguration('params_file').perform(context)
    return [
        Node(
            package='antbot_mpc_ros', executable='gt_peer_predictor',
            namespace=ego, parameters=[params,
                str(Path(get_package_share_directory('mpc_planner_antbot'))/'config/peer_prediction.yaml'), {
                'ego_robot': ego, 'peer_robot': peer, 'peer_odom_topic': f'/{peer}/odom',
            }], output='screen',
        )
        for ego, peer in (('robot_a', 'robot_b'), ('robot_b', 'robot_a'))
    ]


def generate_launch_description():
    package = Path(get_package_share_directory('antbot_mpc_ros'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(package / 'config/gt_peer_cv.yaml')),
        OpaqueFunction(function=nodes),
    ])
