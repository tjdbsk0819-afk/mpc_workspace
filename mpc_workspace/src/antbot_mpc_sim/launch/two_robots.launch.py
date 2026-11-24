"""Two isolated Antbots in parallel lanes; no mutual avoidance yet."""
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, SetEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def build_scene(context):
    pkg = Path(get_package_share_directory('antbot_gazebo'))
    description = Path(get_package_share_directory('antbot_description'))
    world = pkg / 'worlds' / 'empty.sdf'
    actions = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', str(description.parent) + ':' + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '/opt/ros/humble/lib:' + os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')),
        ExecuteProcess(cmd=['ign', 'gazebo', '-r', str(world)], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'], output='screen'),
    ]
    encounter = LaunchConfiguration('encounter').perform(context).lower() == 'true'
    robots = [('robot_a', -3.0, 0.0, 0.0), ('robot_b', 3.0, 0.0, 3.141592653589793)] if encounter else [('robot_a', 0.0, -2.0, 0.0), ('robot_b', 0.0, 2.0, 0.0)]
    for name, x, y, yaw in robots:
        config = yaml.safe_load((pkg / 'config/swerve_controller_gazebo.yaml').read_text())
        config['antbot_swerve_controller']['ros__parameters'].update(
            cmd_vel_topic='cmd_vel', odom_frame_id='odom', base_frame_id='base_link')
        # Absolute node names prevent parameters leaking between controller managers.
        namespaced = {f'/{name}/{key}': value for key, value in config.items()}
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', prefix=name, delete=False) as file:
            yaml.safe_dump(namespaced, file)
            params = file.name
        root = ET.fromstring(xacro.process_file(str(pkg / 'urdf/antbot_sim.xacro')).toxml())
        for plugin in root.findall('.//plugin'):
            if plugin.get('name') == 'ign_ros2_control::IgnitionROS2ControlPlugin':
                plugin.find('parameters').text = params
                ros = ET.SubElement(plugin, 'ros')
                ET.SubElement(ros, 'namespace').text = '/' + name
                ET.SubElement(ros, 'remapping').text = '/tf:=/' + name + '/tf'
                ET.SubElement(ros, 'remapping').text = '/tf_static:=/' + name + '/tf_static'
                ET.SubElement(plugin, 'robot_param_node').text = 'robot_state_publisher'
        # Sensors are not bridged in this first two-robot test; isolate their Gazebo topics.
        for topic in root.findall('.//sensor/topic'):
            topic.text = '/' + name + '/' + topic.text.lstrip('/')
        xml = ET.tostring(root, encoding='unicode')
        state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                     name='robot_state_publisher', namespace=name,
                     parameters=[{'robot_description': xml, 'use_sim_time': True}],
                     remappings=[('/tf', '/' + name + '/tf'), ('/tf_static', '/' + name + '/tf_static')])
        spawn = Node(package='ros_gz_sim', executable='create', namespace=name,
                     arguments=['-name', name, '-string', xml, '-x', str(x), '-y', str(y), '-z', '0.15', '-Y', str(yaw)], output='screen')
        joint = Node(package='controller_manager', executable='spawner', namespace=name,
                     arguments=['joint_state_broadcaster', '-c', '/' + name + '/controller_manager',
                                '--param-file', params, '--controller-manager-timeout', '60'], output='screen')
        drive = Node(package='controller_manager', executable='spawner', namespace=name,
                     arguments=['antbot_swerve_controller', '-c', '/' + name + '/controller_manager',
                                '--param-file', params, '--controller-manager-timeout', '60'], output='screen')
        actions.extend([state, spawn,
                        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[joint])),
                        RegisterEventHandler(OnProcessExit(target_action=joint, on_exit=[drive]))])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('encounter', default_value='false'),
        OpaqueFunction(function=build_scene),
    ])
