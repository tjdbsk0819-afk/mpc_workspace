from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config = Path(get_package_share_directory("mpc_planner_jackal")) / "config" / "ros2_params.yaml"
    return LaunchDescription([
        Node(
            package="mpc_planner_jackal",
            executable="jackal_planner",
            name="jackal_planner",
            output="screen",
            parameters=[str(config)],
        ),
    ])
