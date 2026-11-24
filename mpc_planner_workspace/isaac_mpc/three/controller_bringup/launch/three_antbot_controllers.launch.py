"""Three namespaced instances of the existing AntBot Swerve/IsaacTopicSystem."""
from pathlib import Path
import tempfile
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from simulation.helpers.controller_config import namespaced_parameters, namespaced_control_description


def nodes(context):
    root = Path(get_package_share_directory('antbot_isaac_control'))
    template = yaml.safe_load((root/'config/swerve_controller.yaml').read_text())
    xml = xacro.process_file(str(root/'urdf/antbot_control_only.urdf.xacro')).toxml()
    output = Path(LaunchConfiguration('output_directory').perform(context))
    output.mkdir(parents=True, exist_ok=True)
    actions = []
    for robot in ('robot_a', 'robot_b', 'robot_c'):
        description = namespaced_control_description(xml, robot)
        with tempfile.NamedTemporaryFile(mode='w', dir=output, prefix=f'{robot}_', suffix='.yaml', delete=False) as file:
            yaml.safe_dump(namespaced_parameters(template, robot), file)
            params = file.name
        actions.extend([
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 namespace=robot, name='control_description',
                 parameters=[{'robot_description': description, 'use_sim_time': True,
                              'frame_prefix': f'{robot}/control_model/'}],
                 remappings=[('/tf', f'/{robot}/control_model/tf'),
                             ('/tf_static', f'/{robot}/control_model/tf_static')], output='screen'),
            # No global __node remap: the hardware plugin creates another node
            # in this process, and it should retain its own node name.
            Node(package='controller_manager', executable='ros2_control_node', namespace=robot,
                 parameters=[params, {'use_sim_time': True}],
                 remappings=[('robot_description', f'/{robot}/robot_description'),
                             ('~/robot_description', f'/{robot}/robot_description'),
                             ('odom', f'/{robot}/wheel_odom')], output='screen'),
            Node(package='controller_manager', executable='spawner', namespace=robot,
                 arguments=['antbot_swerve_controller', '--controller-manager', f'/{robot}/controller_manager',
                            '--controller-manager-timeout', '60', '--param-file', params], output='screen'),
        ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_directory', default_value='/tmp/isaac_mpc_three/controller_parameters'),
        OpaqueFunction(function=nodes),
    ])
