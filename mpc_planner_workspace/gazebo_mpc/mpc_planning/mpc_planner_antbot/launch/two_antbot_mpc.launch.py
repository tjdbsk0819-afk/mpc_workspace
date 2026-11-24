"""Two independent original C++ planners; output paths without driving the robots."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from simulation.helpers.scenario_config import (DEFAULT_SCENARIO, GOAL_ARGUMENTS, DRIVING_SCENARIOS,
                                               planner_goals, workspace_parameters)


def nodes(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    goals = planner_goals(scenario, {key: LaunchConfiguration(key).perform(context)
                                    for key in GOAL_ARGUMENTS})
    return [LogInfo(msg=f'Two-AntBot scenario {scenario}; goals: {goals}')] + [
        Node(package='mpc_planner_antbot', executable='antbot_planner', namespace=ego,
             parameters=[str(Path(get_package_share_directory('mpc_planner_antbot'))/'config/recovery.yaml'),
                         str(Path(get_package_share_directory('mpc_planner_antbot'))/'config/peer_prediction.yaml'),
                         {**workspace_parameters(scenario),
                          'use_sim_time': True, 'ego_robot': ego, 'peer_robot': peer,
                          'goal': [goals[f'{ego}_goal_x'], goals[f'{ego}_goal_y']],
                          'goal_yaw': goals[f'{ego}_goal_yaw'],
                          'recovery_enabled': LaunchConfiguration('recovery_enabled').perform(context).lower() == 'true'}], output='screen')
        for ego, peer in [('robot_a', 'robot_b'), ('robot_b', 'robot_a')]
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value=DEFAULT_SCENARIO, choices=DRIVING_SCENARIOS),
        *[DeclareLaunchArgument(key, default_value='',
              description='Optional override; otherwise use the selected scenario goal')
          for key in GOAL_ARGUMENTS],
        DeclareLaunchArgument('recovery_enabled', default_value='true'),
        OpaqueFunction(function=nodes),
    ])
