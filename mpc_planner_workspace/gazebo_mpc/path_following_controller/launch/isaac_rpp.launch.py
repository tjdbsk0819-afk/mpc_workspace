"""Run one MPC-native RPP controller per AntBot using Isaac's world-frame TF."""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from simulation.helpers.rpp_config import mpc_rpp_parameters
from simulation.helpers.scenario_config import DEFAULT_SCENARIO, DRIVING_SCENARIOS, workspace_parameters


def nodes(context):
    config_path = Path(LaunchConfiguration('params_file').perform(context))
    template = yaml.safe_load(config_path.read_text())
    execution = yaml.safe_load(Path(LaunchConfiguration('execution_params_file').perform(context)).read_text())['/**']['ros__parameters']
    execution.update(yaml.safe_load((Path(get_package_share_directory('mpc_planner_antbot'))/'config/peer_prediction.yaml').read_text())['/**']['ros__parameters'])
    execution.update(workspace_parameters(LaunchConfiguration('scenario').perform(context)))
    enabled = LaunchConfiguration('enabled').perform(context).lower() == 'true'
    output = Path(LaunchConfiguration('output_directory').perform(context))
    output.mkdir(parents=True, exist_ok=True)
    actions = []
    for robot in ('robot_a', 'robot_b'):
        with tempfile.NamedTemporaryFile(
            mode='w', dir=output, prefix=f'{robot}_', suffix='.yaml', delete=False
        ) as stream:
            peers = [p for p in ('robot_a', 'robot_b') if p != robot]
            yaml.safe_dump(mpc_rpp_parameters(template, execution, robot, peers, enabled), stream)
            params = stream.name
        actions.extend([
            Node(
                package='antbot_navigation', executable='mpc_rpp_controller',
                namespace=robot, parameters=[params], output='screen',
                remappings=[('peers/'+peers[0]+'/state', 'peer_state')],
                # Keep the shared /tf and /tf_static from the Isaac runner.
                # Do not apply a global node-name remap to the child costmap.
                # MPC time sampling and final command checks run inside this node.
            ),
        ])
    return actions


def generate_launch_description():
    package = Path(get_package_share_directory('antbot_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value=DEFAULT_SCENARIO, choices=DRIVING_SCENARIOS),
        DeclareLaunchArgument(
            'params_file', default_value=str(package / 'config/isaac/rpp.yaml'),
        ),
        DeclareLaunchArgument('execution_params_file', default_value=str(package / 'config/isaac/mpc_execution.yaml')),
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument(
            'output_directory', default_value='/tmp/isaac_mpc_two/rpp_parameters',
        ),
        OpaqueFunction(function=nodes),
    ])
