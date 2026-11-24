'''Run the complete Antbot scene and ROS 2 runtime on CUDA device zero.'''

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from simulation_runtime.environment import (
    Ros2EnvironmentError,
    WebRtcEnvironmentError,
    WorkspaceSettings,
    configure_ros2_environment,
    configure_webrtc,
    load_workspace_settings,
    reserve_webrtc_instance,
)

CUDA_DEVICE = 'cuda:0'

class AntbotRos2Runtime:
    '''Own the shared ROS context while composing publisher and subscriber.'''

    def __init__(self, robot, ros_domain_id: int) -> None:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions

        from simulation_runtime.ros2 import AntbotPublisher, JointCommandSubscriber

        self.context = Context()
        self.node = None
        self.executor = None
        self.publisher = None
        self.subscriber = None

        # Kit owns process signals; embedded rclpy must not replace them.
        rclpy.init(
            context=self.context,
            domain_id=ros_domain_id,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        try:
            self.node = Node('antbot_ros2_io', context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.publisher = AntbotPublisher(self.node, robot)
            self.subscriber = JointCommandSubscriber(self.node, robot)
            self.node.get_logger().info(
                f'Antbot ROS 2 ready: command={self.subscriber.topic}, '
                f'state={self.publisher.state_topic}, '
                f'sensors={list(self.publisher.sensor_topics)}'
            )
        except Exception:
            self.close()
            raise

    def spin_once(self) -> None:
        self.executor.spin_once(timeout_sec=0.0)

    def close(self) -> None:
        if self.executor is not None:
            if self.node is not None:
                self.executor.remove_node(self.node)
            self.executor.shutdown()
            self.executor = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.context.ok():
            self.context.shutdown()


def make_antbot_scene_cfg(debug_vis: bool):
    '''Build the complete, always-on Antbot scene configuration.'''

    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.terrains import TerrainImporterCfg
    from isaaclab.utils import configclass

    from robot_model.asset import ANTBOT_CFG
    from robot_model.sensors import (
        CAMERA_BACK_CFG,
        CAMERA_FRONT_RGB_CFG,
        CAMERA_FRONT_DEPTH_CFG,
        CAMERA_LEFT_CFG,
        CAMERA_RIGHT_CFG,
        LIDAR_2D_BACK_CFG,
        LIDAR_2D_FRONT_CFG,
        LIDAR_3D_CFG,
    )

    @configclass
    class AntbotSceneCfg(InteractiveSceneCfg):
        terrain = TerrainImporterCfg(prim_path='/World/ground', terrain_type='plane')
        robot = ANTBOT_CFG.replace(prim_path='{ENV_REGEX_NS}/Robot')
        light = AssetBaseCfg(
            prim_path='/World/light',
            spawn=sim_utils.DomeLightCfg(
                intensity=1500.0,
                color=(0.9, 0.9, 0.9),
            ),
        )
        camera_back = CAMERA_BACK_CFG
        camera_front = CAMERA_FRONT_RGB_CFG
        camera_left = CAMERA_LEFT_CFG
        camera_right = CAMERA_RIGHT_CFG
        camera_front_depth = CAMERA_FRONT_DEPTH_CFG
        lidar_2d_front = LIDAR_2D_FRONT_CFG.replace(debug_vis=debug_vis)
        lidar_2d_back = LIDAR_2D_BACK_CFG.replace(debug_vis=debug_vis)
        lidar_3d = LIDAR_3D_CFG.replace(debug_vis=debug_vis)

    return AntbotSceneCfg


def _tensor_view(value):
    return value.torch if hasattr(value, 'torch') else value


def _print_camera_statistics(scene) -> None:
    from robot_model.sensors import CAMERA_NAMES

    for name in CAMERA_NAMES:
        outputs = scene[f'camera_{name}'].data.output
        shapes = {
            data_type: tuple(_tensor_view(value).shape)
            for data_type, value in outputs.items()
        }
        ranges = {
            data_type: (
                str(_tensor_view(value).dtype),
                _tensor_view(value).min().item(),
                _tensor_view(value).max().item(),
            )
            for data_type, value in outputs.items()
        }
        print(f'[camera/{name}] shapes={shapes} ranges={ranges}')


def _print_lidar_statistics(name: str, sensor) -> None:
    import torch

    hits = sensor.data.ray_hits_w.torch[0]
    origin = sensor.data.pos_w.torch[0]
    valid = torch.isfinite(hits).all(dim=-1)
    valid_count = int(valid.sum().item())
    total_count = int(valid.numel())
    if valid_count == 0:
        print(f'[{name}] valid_hits=0/{total_count}')
        return
    ranges = torch.linalg.vector_norm(hits[valid] - origin, dim=-1)
    print(
        f'[{name}] valid_hits={valid_count}/{total_count} '
        f'min={ranges.min().item():.3f} m '
        f'mean={ranges.mean().item():.3f} m '
        f'max={ranges.max().item():.3f} m'
    )


def print_sensor_statistics(scene) -> None:
    '''Print compact tensor-shape and valid-hit diagnostics.'''

    _print_camera_statistics(scene)
    _print_lidar_statistics('lidar_2d/front', scene['lidar_2d_front'])
    _print_lidar_statistics('lidar_2d/back', scene['lidar_2d_back'])
    _print_lidar_statistics('lidar_3d/main', scene['lidar_3d'])


def launch_simulation_app(settings: WorkspaceSettings):
    '''Configure WebRTC and ROS, then boot Kit through AppLauncher.'''

    from isaaclab.app import AppLauncher

    launcher_overrides, webrtc_kit_args = configure_webrtc(settings)
    try:
        bridge_kit_args = configure_ros2_environment(
            settings,
            publishes_cameras=True,
            enable_bridge=True,
        )
    except Ros2EnvironmentError as error:
        raise SystemExit(str(error)) from error

    kit_args = f'{webrtc_kit_args} {bridge_kit_args}'.strip()
    launch_config = {
        'device': CUDA_DEVICE,
        'enable_cameras': True,
        'kit_args': kit_args,
        'width': 1280,
        'height': 720,
        'window_width': 1920,
        'window_height': 1080,
        'hide_ui': False,
        'renderer': 'RealTimePathTracing',
        'display_options': 3286,
        'visualizer': ['kit'],
    }
    launch_config.update(launcher_overrides)
    app_launcher = AppLauncher(launch_config)
    simulation_app = app_launcher.app
    simulation_app.set_setting('/app/window/drawMouse', True)
    return simulation_app


def create_ros2_graph(ros_domain_id: int) -> None:
    '''Create the runner-owned clock and odometry graph.'''

    from simulation_runtime.action_graph import (
        AntbotActionGraphCfg,
        build_antbot_action_graph,
    )

    graph_cfg = AntbotActionGraphCfg(ros_domain_id=ros_domain_id)
    build_antbot_action_graph(graph_cfg)
    print(f'[ROS2] Physics-step Action Graph ready: {graph_cfg.graph_path}')


def run_simulation(
    simulation_app,
    settings: WorkspaceSettings,
    debug_vis: bool,
) -> None:
    '''Create the complete scene, ROS components, and simulation loop.'''

    import isaaclab.sim as sim_utils
    from isaaclab.scene import InteractiveScene
    import omni.timeline

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=settings.physics_dt, device=CUDA_DEVICE)
    )
    sim.set_camera_view(eye=(4.0, 4.0, 2.5), target=(0.0, 0.0, 0.5))
    scene_cfg = make_antbot_scene_cfg(debug_vis)
    scene = InteractiveScene(scene_cfg(num_envs=1, env_spacing=2.0))
    sim.reset()
    if not sim.is_rendering:
        raise RuntimeError(
            'WebRTC is enabled, but the Isaac Lab Kit render loop is inactive.'
        )

    timeline = omni.timeline.get_timeline_interface()
    timeline.pause()
    create_ros2_graph(settings.ros_domain_id)
    timeline.play()
    ros2 = AntbotRos2Runtime(scene['robot'], settings.ros_domain_id)

    print('[INFO] Antbot scene is ready: all cameras and LiDARs enabled')
    print(f'[ROS2] ROS_DOMAIN_ID={settings.ros_domain_id}')
    print(f'[ROS2] GPU joint command topic: {ros2.subscriber.topic}')
    print(f'[ROS2] Joint state topic: {ros2.publisher.state_topic}')
    print(f'[ROS2] Sensor topics: {list(ros2.publisher.sensor_topics)}')
    print(f'[WebRTC] Open http://{settings.webrtc_public_ip}:{settings.web_viewer_port}')
    print(
        f'[WebRTC] signal={settings.webrtc_public_ip}:'
        f'{settings.webrtc_signal_port}/tcp '
        f'stream={settings.webrtc_public_ip}:'
        f'{settings.webrtc_stream_port}/udp'
    )
    print(
        '[INFO] RayCaster targets /World/ground. Add scene mesh prim paths '
        'to scan obstacles.'
    )

    simulation_time = 0.0
    step = 0
    try:
        while simulation_app.is_running():
            ros2.spin_once()
            ros2.subscriber.apply_latest()
            scene.write_data_to_sim()
            sim.step()
            scene.update(settings.physics_dt)
            simulation_time += settings.physics_dt
            ros2.publisher.publish(scene, simulation_time)
            if step % settings.print_interval_steps == 0:
                print_sensor_statistics(scene)
            step += 1
    finally:
        ros2.close()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run all Antbot sensors and ROS 2 I/O on cuda:0.'
    )
    parser.add_argument(
        '--debug-vis',
        action='store_true',
        help='Show all LiDAR ray/hit markers in the viewport.',
    )
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    settings = load_workspace_settings()
    try:
        with reserve_webrtc_instance(settings):
            simulation_app = launch_simulation_app(settings)
            try:
                run_simulation(simulation_app, settings, arguments.debug_vis)
            finally:
                simulation_app.close()
    except WebRtcEnvironmentError as error:
        raise SystemExit(f'[WebRTC] {error}') from error


if __name__ == '__main__':
    main()
