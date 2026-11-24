"""Two independent original C++ planners; output paths without driving the robots."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nodes(context):
    goals = {r: [float(LaunchConfiguration(f'{r}_goal_x').perform(context)),
                 float(LaunchConfiguration(f'{r}_goal_y').perform(context))]
             for r in ('robot_a', 'robot_b')}
    return [
        Node(package='mpc_planner_antbot', executable='antbot_planner', namespace=ego,
             parameters=[str(Path(get_package_share_directory('mpc_planner_antbot'))/'config/recovery.yaml'),
                         {'use_sim_time': True, 'ego_robot': ego, 'peer_robot': peer,
                          'goal': goals[ego], 'recovery_enabled': LaunchConfiguration('recovery_enabled').perform(context).lower() == 'true'}], output='screen')
        for ego, peer in [('robot_a', 'robot_b'), ('robot_b', 'robot_a')]
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_a_goal_x', default_value='3.0'),
        DeclareLaunchArgument('robot_a_goal_y', default_value='0.0'),
        DeclareLaunchArgument('robot_b_goal_x', default_value='-3.0'),
        DeclareLaunchArgument('robot_b_goal_y', default_value='0.0'),
        DeclareLaunchArgument('recovery_enabled', default_value='true'),
        OpaqueFunction(function=nodes),
    ])
