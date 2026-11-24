#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    package_share = FindPackageShare("antbot_isaac_control")
    control_xacro = PathJoinSubstitution(
        [package_share, "urdf", "antbot_control_only.urdf.xacro"]
    )
    controller_config = PathJoinSubstitution(
        [package_share, "config", "swerve_controller.yaml"]
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", control_xacro]),
        value_type=str,
    )

    description_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        remappings=[
            ("/tf", "/antbot_control/tf"),
            ("/tf_static", "/antbot_control/tf_static"),
        ],
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    swerve_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "antbot_swerve_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            description_publisher,
            controller_manager,
            swerve_spawner,
        ]
    )
