"""Fixed Isaac Lab sensor configuration for the complete Antbot sensor suite."""

import isaaclab.sim as sim_utils
from isaaclab.sensors import CameraCfg, RayCasterCfg, patterns


CAMERA_RGB_NAMES = ("back", "front", "left", "right")
CAMERA_NAMES = CAMERA_RGB_NAMES + ("front_depth",)
LIDAR_2D_NAMES = ("front", "back")


def _camera_cfg(prim_path: str, data_types: list[str]) -> CameraCfg:
    """Build a camera at the identity pose of its USD optical frame."""

    return CameraCfg(
        prim_path=prim_path,
        update_period=0.033,
        height=480,
        width=640,
        data_types=data_types,
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0,
            horizontal_aperture=20.955,
            clipping_range=(0.1, 100.0),
        ),
        offset=CameraCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 1.0),
            convention="ros",
        ),
    )


CAMERA_BACK_CFG = _camera_cfg(
    "{ENV_REGEX_NS}/Robot/base_link/camera_mono_back_link/"
    "camera_mono_back_optical_link/sim_camera",
    ["rgb"],
)
CAMERA_FRONT_RGB_CFG = _camera_cfg(
    "{ENV_REGEX_NS}/Robot/base_link/camera_mono_front_link/"
    "camera_mono_front_optical_link/sim_camera",
    ["rgb"],
)
CAMERA_LEFT_CFG = _camera_cfg(
    "{ENV_REGEX_NS}/Robot/base_link/camera_mono_left_link/"
    "camera_mono_left_optical_link/sim_camera",
    ["rgb"],
)
CAMERA_RIGHT_CFG = _camera_cfg(
    "{ENV_REGEX_NS}/Robot/base_link/camera_mono_right_link/"
    "camera_mono_right_optical_link/sim_camera",
    ["rgb"],
)
CAMERA_FRONT_DEPTH_CFG = _camera_cfg(
    "{ENV_REGEX_NS}/Robot/base_link/camera_stereo_front_depth_link/"
    "camera_stereo_front_depth_optical_link/sim_camera",
    ["rgb", "distance_to_image_plane"],
)


_LIDAR_2D_PATTERN_CFG = patterns.LidarPatternCfg(
    channels=1,
    vertical_fov_range=(0.0, 0.0),
    horizontal_fov_range=(-180.0, 180.0),
    horizontal_res=0.5,
)


def _lidar_2d_cfg(prim_path: str) -> RayCasterCfg:
    """Build one planar LiDAR that tracks its authored USD frame."""

    return RayCasterCfg(
        prim_path=prim_path,
        update_period=0.1,
        mesh_prim_paths=["/World/ground"],
        offset=RayCasterCfg.OffsetCfg(
            pos=(0.0, 0.0, 0.0),
            rot=(0.0, 0.0, 0.0, 1.0),
        ),
        ray_alignment="base",
        pattern_cfg=_LIDAR_2D_PATTERN_CFG,
        max_distance=30.0,
        debug_vis=False,
    )


LIDAR_2D_FRONT_CFG = _lidar_2d_cfg(
    "{ENV_REGEX_NS}/Robot/base_link/lidar_2d_front_link/sim_lidar"
)
LIDAR_2D_BACK_CFG = _lidar_2d_cfg(
    "{ENV_REGEX_NS}/Robot/base_link/lidar_2d_back_link/sim_lidar"
)

LIDAR_3D_CFG = RayCasterCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base_link/lidar_3d_link/sim_lidar",
    update_period=0.1,
    mesh_prim_paths=["/World/ground"],
    offset=RayCasterCfg.OffsetCfg(
        pos=(0.0, 0.0, 0.0),
        rot=(0.0, 0.0, 0.0, 1.0),
    ),
    ray_alignment="base",
    pattern_cfg=patterns.LidarPatternCfg(
        channels=32,
        vertical_fov_range=(-45.0, 45.0),
        horizontal_fov_range=(-180.0, 180.0),
        horizontal_res=0.4,
    ),
    max_distance=100.0,
    debug_vis=False,
)

RAYCASTER_CFGS = (LIDAR_2D_FRONT_CFG, LIDAR_2D_BACK_CFG, LIDAR_3D_CFG)


__all__ = [
    "CAMERA_BACK_CFG",
    "CAMERA_FRONT_RGB_CFG",
    "CAMERA_FRONT_DEPTH_CFG",
    "CAMERA_RGB_NAMES",
    "CAMERA_LEFT_CFG",
    "CAMERA_NAMES",
    "CAMERA_RIGHT_CFG",
    "LIDAR_2D_BACK_CFG",
    "LIDAR_2D_FRONT_CFG",
    "LIDAR_2D_NAMES",
    "LIDAR_3D_CFG",
    "RAYCASTER_CFGS",
]
