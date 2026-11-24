#!/usr/bin/env python3
"""Run two AntBots using this project's simulation runtime and robot model.

Structure follows simulation_runtime/run_antbot.py: environment setup -> AppLauncher
-> scene -> ROS runtime -> command/physics/publication loop -> cleanup.
Use Isaac Sim's /isaac-sim/python.sh, not the MPC ROS container's Python.
"""
from __future__ import annotations

import argparse
from contextlib import nullcontext
from dataclasses import replace
import math
import os
from pathlib import Path
import sys
import time
import traceback

# Imports of the external AntBot modules must not create __pycache__ there.
sys.dont_write_bytecode = True
DMPC_ROOT = Path(__file__).resolve().parents[2]
if str(DMPC_ROOT) not in sys.path:
    sys.path.insert(0, str(DMPC_ROOT))
CUDA_DEVICE = 'cuda:0'
ROBOT_NAMES = ('robot_a', 'robot_b')


def resolve_isaac_workspace(path=None):
    candidates = [Path(path)] if path else [
        Path(os.environ.get('ANTBOT_ISAAC_WORKSPACE', '/workspace')),
        DMPC_ROOT,
    ]
    for root in candidates:
        if (root/'simulation_runtime/run_antbot.py').is_file() and (root/'robot_model/asset.py').is_file():
            return root.resolve()
    raise FileNotFoundError('Cannot locate simulation_runtime and robot_model. Pass --project-root PATH.')


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project-root', '--isaac-workspace', dest='isaac_workspace',
                        help='Project root containing robot_model and simulation_runtime')
    parser.add_argument('--scenario', choices=list('ABCD'), default='A')
    parser.add_argument('--robot-a-start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--robot-b-start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'))
    parser.add_argument('--spawn-height', type=float, help='Override root Z; otherwise preserve ANTBOT_CFG height')
    parser.add_argument('--ros-domain-id', type=int)
    parser.add_argument('--physics-dt', type=float)
    parser.add_argument('--state-period', type=float, default=.02, help='ROS odom/joint state period, simulation seconds')
    parser.add_argument('--command-timeout', type=float, default=.5, help='Joint command watchdog, wall seconds')
    parser.add_argument('--headless', action='store_true', help='Physics without WebRTC viewer')
    parser.add_argument('--duration', type=float, default=0., help='Exit after this simulation duration; 0 runs until closed')
    parser.add_argument('--settle-time', type=float, default=.5, help='Initial physics settling with wheels stopped')
    parser.add_argument('--real-time-factor', type=float, default=1., help='Wall pacing factor; 0 runs unrestricted')
    parser.add_argument('--check-config', action='store_true', help='Validate references and print configuration without starting Kit')
    args = parser.parse_args(argv)
    for key in ('state_period', 'command_timeout'):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            parser.error(f'--{key.replace("_", "-")} must be positive and finite')
    for key in ('duration', 'settle_time', 'real_time_factor'):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) < 0:
            parser.error(f'--{key.replace("_", "-")} must be non-negative and finite')
    if args.physics_dt is not None and (not math.isfinite(args.physics_dt) or args.physics_dt <= 0):
        parser.error('--physics-dt must be positive and finite')
    if args.ros_domain_id is not None and not 0 <= args.ros_domain_id <= 232:
        parser.error('--ros-domain-id must be in [0, 232]')
    for start in (args.robot_a_start, args.robot_b_start):
        if start and not all(math.isfinite(v) for v in start):
            parser.error('Start pose must be finite')
    if args.spawn_height is not None and not math.isfinite(args.spawn_height):
        parser.error('--spawn-height must be finite')
    return args


def configure_reference(root, args):
    # Environment module is stdlib-only and safe to import before Kit.
    sys.path.insert(0, str(root))
    from simulation_runtime.environment import load_workspace_settings
    settings = load_workspace_settings()
    overrides = {}
    if args.ros_domain_id is not None:
        overrides['ros_domain_id'] = args.ros_domain_id
    if args.physics_dt is not None:
        overrides['physics_dt'] = args.physics_dt
    return replace(settings, **overrides)


def launch_simulation_app(settings, headless):
    if not headless:
        # Reuse the working single-AntBot stream initialization verbatim:
        # cameras/render loop, ROS bridge, WebRTC ports, UI and mouse settings.
        from simulation_runtime.run_antbot import launch_simulation_app as launch_antbot_app
        return launch_antbot_app(settings)
    from isaaclab.app import AppLauncher
    from simulation_runtime.environment import configure_ros2_environment
    configure_ros2_environment(settings, publishes_cameras=False, enable_bridge=False)
    config = {'device': CUDA_DEVICE, 'enable_cameras': False, 'headless': True, 'livestream': 0,
              'width': 1280, 'height': 720}
    return AppLauncher(config).app


def make_two_antbot_scene_cfg(spec, args):
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.terrains import TerrainImporterCfg
    from isaaclab.utils import configclass
    from robot_model.asset import ANTBOT_CFG
    from simulation.helpers.state_utils import yaw_quaternion

    def robot_cfg(index):
        start = spec.starts[index]
        height = ANTBOT_CFG.init_state.pos[2] if args.spawn_height is None else args.spawn_height
        initial = ANTBOT_CFG.init_state.replace(pos=(float(start[0]), float(start[1]), height),
                                               rot=yaw_quaternion(float(start[2])))
        return ANTBOT_CFG.replace(prim_path=f'{{ENV_REGEX_NS}}/Robot{index}', init_state=initial)

    @configclass
    class TwoAntbotSceneCfg(InteractiveSceneCfg):
        terrain = TerrainImporterCfg(prim_path='/World/ground', terrain_type='plane')
        robot_a = robot_cfg(0)
        robot_b = robot_cfg(1)
        light = AssetBaseCfg(prim_path='/World/light',
                            spawn=sim_utils.DomeLightCfg(intensity=1500., color=(.9, .9, .9)))

    cfg = TwoAntbotSceneCfg(num_envs=1, env_spacing=10., filter_collisions=False)
    # Both robots share env_0, so their collision/contact interaction is enabled.
    for index, (x, y, radius) in enumerate(spec.obstacles):
        setattr(cfg, f'obstacle_{index}', AssetBaseCfg(
            prim_path=f'/World/obstacle_{index}',
            spawn=sim_utils.CylinderCfg(radius=radius, height=.6,
                                       collision_props=sim_utils.CollisionPropertiesCfg()),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(x, y, .3))))
    if spec.bounds:
        xmin, xmax, ymin, ymax = spec.bounds
        walls = [((xmax-xmin+.2, .1, .6), ((xmin+xmax)/2, ymin-.05, .3)),
                 ((xmax-xmin+.2, .1, .6), ((xmin+xmax)/2, ymax+.05, .3)),
                 ((.1, ymax-ymin, .6), (xmin-.05, (ymin+ymax)/2, .3)),
                 ((.1, ymax-ymin, .6), (xmax+.05, (ymin+ymax)/2, .3))]
        for index, (size, pos) in enumerate(walls):
            setattr(cfg, f'wall_{index}', AssetBaseCfg(
                prim_path=f'/World/wall_{index}',
                spawn=sim_utils.CuboidCfg(size=size, collision_props=sim_utils.CollisionPropertiesCfg()),
                init_state=AssetBaseCfg.InitialStateCfg(pos=pos)))
    return cfg


class TwoAntbotRos2Runtime:
    """Shared ROS context, per-robot nodes and existing JointCommandSubscriber."""
    def __init__(self, robots, settings, args):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        from antbot_interfaces.msg import JointCommandFeedback
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        from simulation_runtime.ros2.subscriber import JointCommandSubscriber
        from robot_model.asset import ANTBOT_STEERING_JOINT_NAMES, ANTBOT_WHEEL_JOINT_NAMES
        from simulation.helpers.state_utils import CommandNodeAdapter
        self.context = Context()
        self.nodes, self.subscribers, self.states, self.odoms = [], [], [], []
        self.feedback_publishers = []
        self.applied_commands = [None, None]
        self.applied_times = [0., 0.]
        self.last_commands = [-float('inf'), -float('inf')]
        self.command_timeout, self.period = args.command_timeout, args.state_period
        self.robots, self.executor = robots, None
        self.next_state = 0.
        # Nav2 Jazzy's OdomSubscriber uses SystemDefaultsQoS (normally reliable).
        # A reliable publisher also serves the existing best-effort subscribers.
        odom_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        rclpy.init(context=self.context, domain_id=settings.ros_domain_id,
                   signal_handler_options=SignalHandlerOptions.NO)
        try:
            self.executor = SingleThreadedExecutor(context=self.context)
            for index, (name, robot) in enumerate(zip(ROBOT_NAMES, robots)):
                if robot.num_instances != 1:
                    raise RuntimeError('Each AntBot must be a separate articulation with one instance')
                # The existing subscriber uses an absolute /joint_command topic.
                # A per-node remap reuses it without changing its source/constants.
                node = Node('isaac_io', namespace=name, context=self.context,
                            use_global_arguments=False,
                            cli_args=['--ros-args', '-r', f'/joint_command:=/{name}/joint_command'])
                self.nodes.append(node)
                self.executor.add_node(node)
                adapter = CommandNodeAdapter(node, ANTBOT_STEERING_JOINT_NAMES, ANTBOT_WHEEL_JOINT_NAMES,
                    lambda m=index: self.last_commands.__setitem__(m, time.monotonic()))
                self.subscribers.append(JointCommandSubscriber(adapter, robot))
                self.states.append(node.create_publisher(JointState, 'joint_states', qos_profile_sensor_data))
                self.feedback_publishers.append(node.create_publisher(
                    JointCommandFeedback, 'joint_feedback', qos_profile_sensor_data))
                self.odoms.append(node.create_publisher(Odometry, 'odom', odom_qos))
                node.get_logger().info(f'joint_command={self.subscribers[-1].topic}; odom=/{name}/odom; frame=world')
            self.clock = self.nodes[0].create_publisher(Clock, '/clock', 10)
            self.tf = self.nodes[0].create_publisher(TFMessage, '/tf', 10)
        except Exception:
            self.close()
            raise

    def spin_once(self):
        # Drain the small state/command queue without blocking physics.
        for _ in range(8):
            self.executor.spin_once(timeout_sec=0.)

    def stop_wheels(self, index):
        import torch
        from robot_model.asset import ANTBOT_WHEEL_JOINT_NAMES
        robot = self.robots[index]
        ids = [robot.joint_names.index(name) for name in ANTBOT_WHEEL_JOINT_NAMES]
        target = torch.zeros((1, len(ids)), device=robot.device)
        robot.set_joint_velocity_target(target, joint_ids=ids)

    def apply_latest(self, simulation_time, allow_motion=True):
        import copy
        wall = time.monotonic()
        for index, subscriber in enumerate(self.subscribers):
            if subscriber.apply_latest():
                self.applied_commands[index] = copy.deepcopy(subscriber.applied_command)
                self.applied_times[index] = simulation_time
            if not allow_motion or wall-self.last_commands[index] > self.command_timeout:
                self.stop_wheels(index)
                command = self.applied_commands[index]
                if command is not None:
                    from robot_model.asset import ANTBOT_WHEEL_JOINT_NAMES
                    indices = [command.name.index(name) for name in ANTBOT_WHEEL_JOINT_NAMES]
                    if any(command.velocity[k] != 0. for k in indices):
                        command = copy.deepcopy(command)
                        for k in indices:
                            command.velocity[k] = 0.
                        self.applied_commands[index] = command
                        self.applied_times[index] = simulation_time

    def publish(self, simulation_time):
        from antbot_interfaces.msg import JointCommandFeedback
        from builtin_interfaces.msg import Time
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        from rosgraph_msgs.msg import Clock
        from tf2_msgs.msg import TFMessage
        from simulation.helpers.state_utils import as_numpy, root_state, time_parts
        sec, ns = time_parts(simulation_time)
        timestamp = Time(sec=sec, nanosec=ns)
        self.clock.publish(Clock(clock=timestamp))
        if simulation_time+1e-9 < self.next_state:
            return
        self.next_state = simulation_time+self.period
        transforms = []
        for index, (name, robot) in enumerate(zip(ROBOT_NAMES, self.robots)):
            pose, linear, angular = root_state(robot)
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = timestamp, 'world'
            odom.child_frame_id = f'{name}/base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, pose[:3])
            q = odom.pose.pose.orientation
            q.x, q.y, q.z, q.w = map(float, pose[3:])
            v, w = odom.twist.twist.linear, odom.twist.twist.angular
            v.x, v.y, v.z = map(float, linear)
            w.x, w.y, w.z = map(float, angular)
            self.odoms[index].publish(odom)
            joints = JointState()
            joints.header.stamp = timestamp
            joints.name = list(robot.joint_names)
            joints.position = as_numpy(robot.data.joint_pos)[0].tolist()
            joints.velocity = as_numpy(robot.data.joint_vel)[0].tolist()
            self.states[index].publish(joints)
            feedback = JointCommandFeedback()
            feedback.state = joints
            if self.applied_commands[index] is not None:
                feedback.applied_command = self.applied_commands[index]
                sec_applied, ns_applied = time_parts(self.applied_times[index])
                feedback.applied_at = Time(sec=sec_applied, nanosec=ns_applied)
                feedback.command_valid = True
            self.feedback_publishers[index].publish(feedback)
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            t = transform.transform.translation
            t.x, t.y, t.z = map(float, pose[:3])
            transform.transform.rotation = q
            transforms.append(transform)
        self.tf.publish(TFMessage(transforms=transforms))

    def close(self):
        for index in range(len(self.subscribers)):
            self.stop_wheels(index)
        if self.executor is not None:
            for node in self.nodes:
                self.executor.remove_node(node)
            self.executor.shutdown()
        for node in self.nodes:
            node.destroy_node()
        self.nodes.clear()
        if self.context.ok():
            self.context.shutdown()


def run_simulation(simulation_app, settings, args):
    print('[Isaac] Loading scene modules and scenario', flush=True)
    import isaaclab.sim as sim_utils
    from isaaclab.scene import InteractiveScene
    import omni.timeline
    from legacy_mpc_reference.scenarios import scenario
    from simulation.helpers.state_utils import as_numpy
    spec = scenario(args.scenario)
    for index, value in enumerate((args.robot_a_start, args.robot_b_start)):
        if value is not None:
            spec.starts[index] = value
    print('[Isaac] Creating simulation context', flush=True)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=settings.physics_dt, device=CUDA_DEVICE))
    sim.set_camera_view(eye=(7., 7., 5.), target=(0., 0., .4))
    print('[Isaac] Creating two AntBot articulations', flush=True)
    scene = InteractiveScene(make_two_antbot_scene_cfg(spec, args))
    print('[Isaac] Initializing physics and resetting scene', flush=True)
    sim.reset()
    if not args.headless and not sim.is_rendering:
        raise RuntimeError('WebRTC needs an active Isaac Lab rendering loop')
    timeline = omni.timeline.get_timeline_interface()
    timeline.pause()
    robots = [scene[name] for name in ROBOT_NAMES]
    print('[Isaac] Starting ROS interfaces', flush=True)
    runtime = TwoAntbotRos2Runtime(robots, settings, args)
    timeline.play()
    print('[Isaac] Timeline playing; physics and viewport updates enabled', flush=True)
    simulation_time, step_count, paused = 0., 0, False
    print(f'[Isaac] Two AntBots share /World/envs/env_0; scenario={args.scenario}', flush=True)
    print(f'[ROS2] Jazzy/Zenoh domain={settings.ros_domain_id}; /clock and world-frame /odom', flush=True)
    print('[Control] Receives per-robot /joint_command; start the Swerve Controllers separately', flush=True)
    if not args.headless:
        print(f'[WebRTC] http://{settings.webrtc_public_ip}:{settings.web_viewer_port}', flush=True)
        print(f'[WebRTC] signal={settings.webrtc_public_ip}:{settings.webrtc_signal_port}/tcp '
              f'stream={settings.webrtc_public_ip}:{settings.webrtc_stream_port}/udp', flush=True)
    wall_origin = time.monotonic()
    try:
        runtime.publish(0.)
        while simulation_app.is_running():
            runtime.spin_once()
            if not sim.is_playing():
                runtime.apply_latest(simulation_time, allow_motion=False)
                scene.write_data_to_sim()
                sim.render()
                paused = True
                time.sleep(.01)
                continue
            if paused:
                runtime.last_commands[:] = [-float('inf'), -float('inf')]
                wall_origin = time.monotonic()-simulation_time/max(args.real_time_factor, 1e-9)
                paused = False
            runtime.apply_latest(simulation_time, allow_motion=simulation_time >= args.settle_time)
            scene.write_data_to_sim()
            sim.step()
            scene.update(settings.physics_dt)
            simulation_time += settings.physics_dt
            runtime.publish(simulation_time)
            if step_count % settings.print_interval_steps == 0:
                positions = [as_numpy(robot.data.root_link_pose_w)[0, :2].tolist() for robot in robots]
                print(f'[State] t={simulation_time:.2f}s robot_a={positions[0]} robot_b={positions[1]}', flush=True)
            step_count += 1
            if args.duration and simulation_time >= args.duration:
                break
            if args.real_time_factor:
                wait = wall_origin+simulation_time/args.real_time_factor-time.monotonic()
                if wait > 0:
                    time.sleep(min(wait, settings.physics_dt/args.real_time_factor))
    finally:
        runtime.close()
        scene.write_data_to_sim()


def main(argv=None):
    args = parse_arguments(argv)
    root = resolve_isaac_workspace(args.isaac_workspace)
    settings = configure_reference(root, args)
    if args.check_config:
        print(f'Isaac reference: {root}\nScenario: {args.scenario}\nROS domain: {settings.ros_domain_id}\nPhysics dt: {settings.physics_dt}')
        print('Robots: robot_a, robot_b; odom frame: world; transport: rmw_zenoh_cpp')
        return
    from simulation_runtime.environment import Ros2EnvironmentError, WebRtcEnvironmentError, reserve_webrtc_instance
    guard = nullcontext() if args.headless else reserve_webrtc_instance(settings)
    try:
        with guard:
            app = launch_simulation_app(settings, args.headless)
            try:
                run_simulation(app, settings, args)
            except Exception:
                # Kit may terminate Python while closing. Print the original
                # exception before shutdown so Docker logs retain the cause.
                print('[Isaac] Two-AntBot runner failed; traceback follows:', file=sys.stderr, flush=True)
                traceback.print_exc(file=sys.stderr)
                sys.stderr.flush()
                raise
            finally:
                app.close()
    except (Ros2EnvironmentError, WebRtcEnvironmentError) as error:
        raise SystemExit(str(error)) from error
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
