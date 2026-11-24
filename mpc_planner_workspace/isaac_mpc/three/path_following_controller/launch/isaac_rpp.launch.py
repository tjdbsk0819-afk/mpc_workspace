"""Run one RPP controller server per AntBot using Isaac's world-frame TF."""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

from simulation.helpers.rpp_config import namespaced_rpp_parameters


def nodes(context):
    config_path = Path(LaunchConfiguration('params_file').perform(context))
    template = yaml.safe_load(config_path.read_text())
    output = Path(LaunchConfiguration('output_directory').perform(context))
    output.mkdir(parents=True, exist_ok=True)
    actions = []
    for robot in ('robot_a', 'robot_b'):
        with tempfile.NamedTemporaryFile(
            mode='w', dir=output, prefix=f'{robot}_', suffix='.yaml', delete=False
        ) as stream:
            yaml.safe_dump(namespaced_rpp_parameters(template, robot), stream)
            params = stream.name
        actions.extend([
            Node(
                package='nav2_controller', executable='controller_server',
                namespace=robot, parameters=[params], output='screen',
                # Keep the shared /tf and /tf_static from the Isaac runner.
                # Do not apply a global node-name remap to the child costmap.
                # The adapter checks MPC/GT freshness before forwarding to Swerve.
                remappings=[('cmd_vel', f'/{robot}/rpp_cmd_vel')],
            ),
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                namespace=robot, name='lifecycle_manager_rpp',
                parameters=[{
                    'use_sim_time': True, 'autostart': True,
                    'node_names': ['controller_server'],
                }], output='screen',
            ),
        ])
    return actions


def generate_launch_description():
    package = Path(get_package_share_directory('antbot_navigation'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=str(package / 'config/isaac/rpp.yaml'),
        ),
        DeclareLaunchArgument(
            'output_directory', default_value='/tmp/isaac_mpc/rpp_parameters',
        ),
        OpaqueFunction(function=nodes),
    ])
