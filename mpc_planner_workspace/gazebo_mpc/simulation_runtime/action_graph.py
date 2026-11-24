"""Build the runner-owned ROS 2 clock and odometry Action Graph."""

from __future__ import annotations

from dataclasses import dataclass

from robot_model.asset import ANTBOT_DEFAULT_ENV_NAMESPACE
from robot_model.sensors import RAYCASTER_CFGS


@dataclass(frozen=True)
class AntbotActionGraphCfg:
    """Paths and topic names for the always-on Antbot Action Graph."""

    graph_path: str = "/World/ROS2_Antbot"
    env_namespace: str = ANTBOT_DEFAULT_ENV_NAMESPACE
    robot_prim_path: str = "/World/envs/env_0/Robot"
    articulation_root_path: str = "/World/envs/env_0/Robot/base_link"
    ros_domain_id: int = 0
    rebuild_existing_graph: bool = True
    clock_topic: str = "/clock"
    odom_topic: str = "/odom"
    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"


@dataclass(frozen=True)
class AntbotActionGraphResult:
    """Graph handle and the RayCaster anchors validated during construction."""

    graph_handle: object
    raycaster_prim_paths: tuple[str, ...]


def _require_prim(stage, prim_path: str, description: str):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"{description} prim does not exist: {prim_path}")
    return prim


def build_antbot_action_graph(
    config: AntbotActionGraphCfg = AntbotActionGraphCfg(),
    stage=None,
) -> AntbotActionGraphResult:
    """Validate the complete scene and create clock and odometry output."""

    import omni.graph.core as og
    import omni.usd
    from pxr import Sdf, UsdPhysics

    stage = stage or omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open.")
    _require_prim(stage, config.robot_prim_path, "Antbot robot")
    articulation = _require_prim(
        stage, config.articulation_root_path, "Antbot articulation root"
    )
    if not articulation.HasAPI(UsdPhysics.ArticulationRootAPI):
        raise RuntimeError(
            "PhysicsArticulationRootAPI is missing from "
            f"{config.articulation_root_path}."
        )

    namespace = config.env_namespace.rstrip("/")
    raycaster_paths = tuple(
        sensor_cfg.prim_path.replace("{ENV_REGEX_NS}", namespace)
        for sensor_cfg in RAYCASTER_CFGS
    )
    for path in raycaster_paths:
        _require_prim(stage, path, "RayCaster anchor")

    existing_graph = stage.GetPrimAtPath(config.graph_path)
    if existing_graph and existing_graph.IsValid():
        if not config.rebuild_existing_graph:
            raise RuntimeError(f"Action Graph already exists: {config.graph_path}")
        stage.RemovePrim(config.graph_path)

    keys = og.Controller.Keys
    target = [Sdf.Path(config.articulation_root_path)]
    nodes = [
        ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
        ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
    ]
    values = [
        ("ROS2Context.inputs:domain_id", config.ros_domain_id),
        ("PublishClock.inputs:topicName", config.clock_topic),
        ("ComputeOdometry.inputs:chassisPrim", target),
        ("PublishOdometry.inputs:topicName", config.odom_topic),
        ("PublishOdometry.inputs:odomFrameId", config.odom_frame_id),
        ("PublishOdometry.inputs:chassisFrameId", config.base_frame_id),
    ]
    links = [
        ("OnPhysicsStep.outputs:step", "PublishClock.inputs:execIn"),
        ("ROS2Context.outputs:context", "PublishClock.inputs:context"),
        ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ("OnPhysicsStep.outputs:step", "ComputeOdometry.inputs:execIn"),
        ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
        ("ROS2Context.outputs:context", "PublishOdometry.inputs:context"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
        (
            "ComputeOdometry.outputs:angularVelocity",
            "PublishOdometry.inputs:angularVelocity",
        ),
        (
            "ComputeOdometry.outputs:linearVelocity",
            "PublishOdometry.inputs:linearVelocity",
        ),
        ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
        ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
    ]

    graph_handle, _, _, _ = og.Controller.edit(
        {
            "graph_path": config.graph_path,
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
        },
        {
            keys.CREATE_NODES: nodes,
            keys.SET_VALUES: values,
            keys.CONNECT: links,
        },
    )
    print(
        "[ActionGraph] Python publishes RayCaster data; validated anchors: "
        f"{list(raycaster_paths)}"
    )
    return AntbotActionGraphResult(graph_handle, raycaster_paths)


__all__ = [
    "AntbotActionGraphCfg",
    "AntbotActionGraphResult",
    "build_antbot_action_graph",
]
