"""State relay -> independent IPOPT planners -> RPP -> Swerve -> Isaac."""
from pathlib import Path
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from simulation.helpers.rpp_config import mpc_rpp_parameters
from simulation.helpers.scenario_config import DRIVING_SCENARIOS, GOAL_ARGUMENTS, planner_goals, workspace_parameters


def nodes(context):
    share = Path(get_package_share_directory("antbot_dmpc_ros"))
    navigation = Path(get_package_share_directory("antbot_navigation"))
    params = LaunchConfiguration("params_file").perform(context)
    planner = yaml.safe_load(Path(params).read_text())["/**"]["ros__parameters"]
    execution = yaml.safe_load((navigation/"config/isaac/mpc_execution.yaml").read_text())["/**"]["ros__parameters"]
    template = yaml.safe_load((navigation/"config/isaac/rpp.yaml").read_text())
    execution.update({key: value for key, value in planner.items() if key.startswith("peer_prediction_")})
    execution.update(max_speed=planner["max_speed"], max_yaw_rate=planner["max_yaw_rate"],
                     max_acceleration=planner["max_acceleration"], max_yaw_acceleration=planner["max_yaw_acceleration"],
                     minimum_separation=planner["separation"], state_timeout=planner["state_timeout"],
                     plan_timeout=planner["plan_timeout"])
    scenario = LaunchConfiguration("scenario").perform(context)
    workspace = workspace_parameters(scenario)
    execution.update(workspace)
    enabled = LaunchConfiguration("enabled").perform(context).lower() == "true"
    goals = planner_goals(scenario, {key: LaunchConfiguration(key).perform(context) for key in GOAL_ARGUMENTS})
    output = Path("/tmp/dmpc_two/rpp_parameters")
    output.mkdir(parents=True, exist_ok=True)
    actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/"launch/two_antbot_controllers.launch.py")))]
    for ego, peer in (("robot_a", "robot_b"), ("robot_b", "robot_a")):
        actions.extend([
            Node(package="antbot_dmpc_ros", executable="peer_state_relay", namespace=ego,
                 parameters=[{"use_sim_time": True, "ego_robot": ego, "peer_robot": peer,
                              "state_timeout": planner["state_timeout"]}], output="screen"),
            Node(package="antbot_dmpc_ros", executable="dmpc_planner", namespace=ego,
                 parameters=[params, workspace, {"ego_robot": ego, "peer_robot": peer,
                     "goal": [goals[f"{ego}_goal_{axis}"] for axis in ("x", "y", "yaw")]}], output="screen"),
        ])
        with tempfile.NamedTemporaryFile(mode="w", dir=output, prefix=f"{ego}_", suffix=".yaml", delete=False) as stream:
            yaml.safe_dump(mpc_rpp_parameters(template, execution, ego, [peer], enabled), stream)
            rpp_params = stream.name
        actions.append(Node(package="antbot_navigation", executable="mpc_rpp_controller", namespace=ego,
                            parameters=[rpp_params], output="screen",
                            remappings=[(f"peers/{peer}/state", "peer_state")]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory("antbot_dmpc_ros"))
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="B", choices=DRIVING_SCENARIOS),
        DeclareLaunchArgument("enabled", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=str(share/"config/dmpc.yaml")),
        *[DeclareLaunchArgument(key, default_value="") for key in GOAL_ARGUMENTS],
        OpaqueFunction(function=nodes),
    ])
