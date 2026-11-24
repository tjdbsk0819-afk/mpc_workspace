"""ROS 2 adapter: state-only peers -> local IPOPT -> timestamped RPP plans."""
from concurrent.futures import ThreadPoolExecutor
import math
import json
import signal
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from antbot_interfaces.msg import MpcPlan, PeerState
from std_msgs.msg import String

from .observations import body_to_world_velocity
from .planner import LocalObjectMPC, PlannerConfig, integrate, yaw_command_reference
from .prediction import ObjectState
from .ros_time import seconds, ros_stamp, valid_stamp


class DmpcNode(Node):
    def __init__(self):
        super().__init__("dmpc_planner")
        defaults = dict(ego_robot="robot_a", peer_robot="robot_b", peer_robots=["legacy"], horizon=30, dt=0.2,
                        control_frequency=10.0, state_timeout=0.35, plan_timeout=0.35, plan_timeout_enabled=True,
                        allow_reverse=False, max_speed=0.5, max_acceleration=0.5, max_yaw_rate=0.8,
                        max_yaw_acceleration=0.5, separation=1.15, tracking_margin=0.05, current_clearance_check_enabled=True,
                        solve_seconds=0.08, ipopt_mu_strategy="monotone", goal_tolerance=0.2,
                        goal_braking_deceleration=0.25, goal_reaction_time=0.3, goal_stop_reserve=0.08,
                        peer_prediction_model="cv_gaussian", peer_prediction_noise=0.3,
                        peer_prediction_risk=0.05, peer_prediction_dt=0.2,
                        goal=[3.0, 0.0, 0.0], workspace_bounds_enabled=False,
                        workspace_bounds=[-2000.0, 2000.0, -2000.0, 2000.0],
                        wall_robot_radius=0.5, wall_safety_margin=0.05, wall_planning_margin=0.001,
                        static_obstacle_enabled=False, static_obstacle=[0.0, 0.0, 0.4],
                        static_robot_radius=0.5, static_safety_margin=0.15)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        self.ego, self.peer_id = p("ego_robot"), p("peer_robot")
        if not self.ego or not self.peer_id or self.ego == self.peer_id or "/" in self.ego+self.peer_id:
            raise ValueError("Robot names must be distinct and unqualified")
        configured = p("peer_robots")
        self.peer_ids = [self.peer_id] if configured == ["legacy"] else list(configured)
        if (not 1 <= len(self.peer_ids) <= 3 or len(set(self.peer_ids)) != len(self.peer_ids) or
                any(not peer or "/" in peer or peer == self.ego for peer in self.peer_ids)):
            raise ValueError("Expected 1 to 3 distinct unqualified peers")
        self.multi = configured != ["legacy"]
        self.peers, self.peer_walls = {}, {}
        self.goal = np.asarray(p("goal"), dtype=float)
        if self.goal.shape != (3,) or not np.isfinite(self.goal).all():
            raise ValueError("Goal must be finite [x,y,yaw]")
        self.frequency, self.timeout, self.plan_timeout = p("control_frequency"), p("state_timeout"), p("plan_timeout")
        self.plan_timeout_enabled=p("plan_timeout_enabled")
        if not all(math.isfinite(v) and v > 0 for v in (self.frequency, self.timeout, self.plan_timeout)):
            raise ValueError("Frequency and timeouts must be positive")
        if abs(p("peer_prediction_dt")-p("dt")) > 1e-9:
            raise ValueError("Executor and planner prediction intervals must match")
        self.config = PlannerConfig(
            allow_reverse=p("allow_reverse"), peer_count=len(self.peer_ids), horizon=p("horizon"), dt=p("dt"), max_speed=p("max_speed"),
            max_acceleration=p("max_acceleration"), max_yaw_rate=p("max_yaw_rate"),
            max_yaw_acceleration=p("max_yaw_acceleration"), command_dt=1/self.frequency,
            separation=p("separation"), tracking_margin=p("tracking_margin"),
            current_clearance_check_enabled=p("current_clearance_check_enabled"),
            solve_seconds=p("solve_seconds"), mu_strategy=p("ipopt_mu_strategy"),
            goal_tolerance=p("goal_tolerance"),
            goal_braking_deceleration=p("goal_braking_deceleration"),
            goal_reaction_time=p("goal_reaction_time"), goal_stop_reserve=p("goal_stop_reserve"),
            prediction_model=p("peer_prediction_model"), prediction_noise=p("peer_prediction_noise"),
            prediction_risk=p("peer_prediction_risk"),
            bounds=tuple(p("workspace_bounds")) if p("workspace_bounds_enabled") else None,
            obstacles=(tuple(p("static_obstacle")),) if p("static_obstacle_enabled") else (),
            wall_inset=p("wall_robot_radius")+p("wall_safety_margin"),
            wall_planning_margin=p("wall_planning_margin"),
            static_padding=p("static_robot_radius")+p("static_safety_margin")+p("tracking_margin"))
        self.planner = LocalObjectMPC(self.config)
        # Solving on a worker keeps state callbacks, clock-reset handling and
        # invalidation responsive even while IPOPT is running.
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="dmpc_ipopt")
        self.pending = None
        self.epoch = 0
        self.reset_before_solve = True
        self.odom = self.peer = None
        self.odom_wall = self.peer_wall = -math.inf
        self.command_rate, self.command_wall = None, -math.inf
        self.last_clock = None
        self.next_solve = -math.inf
        self.last_status = None
        self.plan_pub = self.create_publisher(MpcPlan, "mpc_plan", 1)
        self.path_pub = self.create_publisher(Path, "mpc_path", 1)
        self.forecast_pub = self.create_publisher(Path, "mpc_peer_prediction", 1)
        self.forecast_pubs = {peer: self.create_publisher(Path, f"mpc_peer_prediction/{peer}", 1) for peer in self.peer_ids}
        self.distribution_pubs = {peer: self.create_publisher(String, f"mpc_peer_distribution/{peer}", 1) for peer in self.peer_ids}
        self.status_pub = self.create_publisher(String, "mpc_status", 1)
        self.timing_pub = self.create_publisher(String, "planning_timing",
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timing_samples = self.timing_accepted = self.timing_deadline_misses = 0
        self.timing_solver_sum = self.timing_planning_sum = 0.
        self.timing_solver_max = self.timing_planning_max = 0.
        self.timing_last_log = -math.inf
        self.create_subscription(Odometry, "odom", self.on_odom, qos_profile_sensor_data)
        for peer_id in self.peer_ids:
            self.create_subscription(PeerState, f"peers/{peer_id}/state" if self.multi else "peer_state",
                                     lambda msg, peer_id=peer_id: self.on_peer(msg, peer_id), 1)
        self.create_subscription(Twist, 'cmd_vel', self.on_command, 1)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.02, self.tick, clock=self.wall_clock)
        self.get_logger().info(
            f"{self.ego}: IPOPT + local {self.config.prediction_model}; peers={self.peer_ids}; "
            f"noise={self.config.prediction_noise}, dt={self.config.dt}, risk={self.config.prediction_risk}; "
            "peer mean/uncertainty applied to constraints; output=mpc_plan")

    def observe_clock(self):
        now = self.get_clock().now().nanoseconds*1e-9
        if self.last_clock is not None and now < self.last_clock-1e-9:
            self.epoch += 1
            self.odom = self.peer = None
            self.peers.clear(); self.peer_walls.clear()
            self.command_rate, self.command_wall = None, -math.inf
            self.reset_before_solve = True
            self.next_solve = -math.inf
        self.last_clock = now
        return now

    def on_odom(self, msg):
        now = self.observe_clock()
        try:
            q, v = msg.pose.pose.orientation, msg.twist.twist.linear
            position = msg.pose.pose.position
            body_to_world_velocity((v.x, v.y, v.z), (q.x, q.y, q.z, q.w))
            if (msg.header.frame_id != "world" or msg.child_frame_id != f"{self.ego}/base_link" or
                    abs(v.y) > 0.05 or not all(math.isfinite(x) for x in
                    (position.x, position.y, msg.twist.twist.angular.z)) or
                    not valid_stamp(msg.header.stamp) or seconds(msg.header.stamp) > now+self.timeout):
                raise ValueError("Invalid ego odometry")
            if self.odom is not None and seconds(msg.header.stamp) <= seconds(self.odom.header.stamp):
                return
            self.odom, self.odom_wall = msg, time.monotonic()
        except ValueError:
            self.odom = None

    def on_command(self, msg):
        self.observe_clock()
        rate = msg.angular.z
        if math.isfinite(rate) and abs(rate) <= self.config.max_yaw_rate+1e-3:
            self.command_rate, self.command_wall = rate, time.monotonic()

    def on_peer(self, msg, peer_id=None):
        peer_id = peer_id or self.peer_id
        now = self.observe_clock()
        if (not msg.valid or msg.robot_id != peer_id or msg.header.frame_id != "world" or
                not all(math.isfinite(v) for v in (msg.x, msg.y, msg.vx, msg.vy)) or
                not valid_stamp(msg.header.stamp) or seconds(msg.header.stamp) > now+self.timeout):
            self.peers.pop(peer_id, None)
            return
        previous = self.peers.get(peer_id)
        if previous is not None and seconds(msg.header.stamp) <= seconds(previous.header.stamp):
            return
        self.peers[peer_id], self.peer_walls[peer_id] = msg, time.monotonic()

    def fresh(self, msg, wall, now):
        return (msg is not None and 0 <= now-seconds(msg.header.stamp) <= self.timeout and
                time.monotonic()-wall <= self.timeout)

    def set_status(self, status):
        self.status_pub.publish(String(data=status))
        if status != self.last_status:
            self.get_logger().info(f"{self.ego}: {status}")
            self.last_status = status

    def invalidate(self, status, now):
        plan = MpcPlan()
        plan.header.frame_id, plan.header.stamp = "world", ros_stamp(now)
        plan.path.header = plan.header
        plan.valid, plan.status = False, status
        self.plan_pub.publish(plan)
        self.path_pub.publish(plan.path)
        self.forecast_pub.publish(plan.path)
        for publisher in self.forecast_pubs.values(): publisher.publish(plan.path)
        self.reset_before_solve = True
        self.set_status(status)

    def publish_timing(self, measurement, accepted, outcome):
        if measurement['attempts']==0:
            return
        completed_wall=time.monotonic()
        planning_ms=(completed_wall-measurement['started'])*1000.
        solver_ms=measurement['solver_ms']
        deadline_ms=1000./self.frequency
        self.timing_samples+=1
        self.timing_accepted+=int(accepted)
        self.timing_deadline_misses+=int(planning_ms>deadline_ms)
        self.timing_solver_sum+=solver_ms;self.timing_planning_sum+=planning_ms
        self.timing_solver_max=max(self.timing_solver_max,solver_ms)
        self.timing_planning_max=max(self.timing_planning_max,planning_ms)
        record=dict(planner='dmpc',robot=self.ego,outcome=outcome,accepted=accepted,
            stamp=self.get_clock().now().nanoseconds*1e-9,attempts=measurement['attempts'],
            start_wall_sec=measurement['started'],completion_wall_sec=completed_wall,
            solver_ms=solver_ms,planning_ms=planning_ms,deadline_ms=deadline_ms,
            samples=self.timing_samples,accepted_samples=self.timing_accepted,
            mean_solver_ms=self.timing_solver_sum/self.timing_samples,max_solver_ms=self.timing_solver_max,
            mean_planning_ms=self.timing_planning_sum/self.timing_samples,max_planning_ms=self.timing_planning_max,
            deadline_misses=self.timing_deadline_misses)
        self.timing_pub.publish(String(data=json.dumps(record)))
        if time.monotonic()-self.timing_last_log>=5.:
            self.get_logger().info(
                f"timing: solver {solver_ms:.1f} ms, planning {planning_ms:.1f} ms "
                f"(mean {record['mean_planning_ms']:.1f}, max {self.timing_planning_max:.1f}), "
                f">{deadline_ms:.0f} ms {self.timing_deadline_misses}/{self.timing_samples}")
            self.timing_last_log=time.monotonic()

    def tick(self):
        tick_started=time.monotonic()
        now = self.observe_clock()
        valid = self.fresh(self.odom, self.odom_wall, now) and all(self.fresh(self.peers.get(peer), self.peer_walls.get(peer, -math.inf), now) for peer in self.peer_ids)
        # Drain a finished worker even when inputs have since become invalid.
        if self.pending is not None and self.pending[0].done():
            future, epoch = self.pending
            self.pending = None
            measurement=None
            accepted=False
            outcome='solver_failed'
            try:
                plan, measurement, error = future.result()
                if error is not None:
                    raise error
                if valid and epoch == self.epoch and now-plan.stamp >= 0 and (not self.plan_timeout_enabled or now-plan.stamp <= self.plan_timeout):
                    self.publish_plan(plan)
                    accepted=True;outcome='tracking'
                else:
                    outcome='discarded_stale_plan'
                    self.invalidate("discarded_stale_plan", now)
            except (RuntimeError, ValueError) as error:
                self.invalidate("solver_failed", now)
                self.get_logger().warning(f"MPC solve failed: {str(error).splitlines()[-1]}",
                                          throttle_duration_sec=5.)
            finally:
                if measurement is not None:
                    self.publish_timing(measurement,accepted,outcome)
        if not valid:
            self.invalidate("waiting_for_fresh_states", now)
            return
        odom = self.odom
        pos, q = odom.pose.pose.position, odom.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        speed, rate = odom.twist.twist.linear.x, odom.twist.twist.angular.z
        state = np.asarray(integrate([pos.x, pos.y, yaw, speed], [0, rate], now-seconds(odom.header.stamp))).ravel()
        if np.linalg.norm(state[:2]-self.goal[:2]) <= self.config.goal_tolerance and abs(speed) <= 0.05:
            self.invalidate("goal_reached", now)
            return
        if self.pending is None and now >= self.next_solve:
            peer = tuple(ObjectState(msg.robot_id, seconds(msg.header.stamp), (msg.x, msg.y, 0), (msg.vx, msg.vy, 0))
                         for msg in (self.peers[key] for key in self.peer_ids))
            reset = self.reset_before_solve
            self.reset_before_solve = False
            # Keep the moving-path policy unchanged. Near the goal, low-speed
            # steering response must not reset the command ramp on every solve.
            command_rate = (yaw_command_reference(rate, self.command_rate,
                time.monotonic()-self.command_wall, self.timeout, self.config.max_yaw_rate)
                if np.linalg.norm(state[:2]-self.goal[:2]) < 1. and abs(speed) < .05 else rate)
            # Capture request time: include queue/polling, validation and publish
            # latency in planning_ms, not only worker/solver execution time.
            request_started=tick_started
            def work():
                result=error=None
                self.planner.last_solver_ms=0.;self.planner.last_solver_calls=0
                try:
                    if reset:
                        self.planner.reset()
                    result=self.planner.solve(state,self.goal,peer,now,command_rate)
                except Exception as caught:
                    error=caught
                measurement=dict(started=request_started,solver_ms=self.planner.last_solver_ms,
                                 attempts=self.planner.last_solver_calls)
                return result,measurement,error
            self.pending = (self.pool.submit(work), self.epoch)
            self.next_solve = now+1/self.frequency

    def publish_plan(self, result):
        c = self.config
        plan = MpcPlan()
        plan.header.frame_id, plan.header.stamp = "world", ros_stamp(result.stamp)
        plan.path.header = plan.header
        for k in range(c.horizon+1):
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = "world", ros_stamp(result.stamp+k*c.dt)
            pose.pose.position.x, pose.pose.position.y = map(float, result.states[:2, k])
            yaw = result.states[2, k]
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            plan.path.poses.append(pose)
        plan.linear_velocities = np.clip(result.states[3], -c.max_speed if c.allow_reverse else 0., c.max_speed).tolist()
        plan.linear_accelerations = np.r_[result.controls[0], 0].tolist()
        plan.angular_velocities = np.r_[result.controls[1], 0].tolist()
        plan.target_speed, plan.target_yaw_rate = float(plan.linear_velocities[1]), float(result.controls[1, 0])
        plan.max_acceleration, plan.max_yaw_rate = c.max_acceleration, c.max_yaw_rate
        plan.minimum_separation = c.separation
        plan.valid, plan.status = True, "tracking"
        self.plan_pub.publish(plan)
        self.path_pub.publish(plan.path)
        for peer_id, prediction in zip(self.peer_ids, result.predictions or (result.prediction,)):
            forecast = Path()
            forecast.header = plan.header
            for stamp, xyz in zip(prediction.times, prediction.positions):
                pose = PoseStamped()
                pose.header.frame_id, pose.header.stamp = "world", ros_stamp(stamp)
                pose.pose.position.x, pose.pose.position.y = xyz[:2]
                pose.pose.orientation.w = 1.0
                forecast.poses.append(pose)
            self.distribution_pubs[peer_id].publish(String(data=json.dumps(dict(
                model=c.prediction_model,frame="world",stamp=result.stamp,
                steps=[dict(t=float(t-result.stamp),x=float(p[0]),y=float(p[1]),radius=float(r),sigma=float(r)/math.sqrt(-2*math.log(c.prediction_risk)))
                       for t,p,r in zip(prediction.times,prediction.positions,prediction.radii)]))))
            self.forecast_pubs[peer_id].publish(forecast)
            if peer_id == self.peer_ids[0]: self.forecast_pub.publish(forecast)
        self.set_status("tracking")

    def destroy_node(self):
        self.timer.cancel()
        try:
            if self.context.ok():
                self.invalidate("shutdown", self.get_clock().now().nanoseconds*1e-9)
        finally:
            self.pool.shutdown(wait=True, cancel_futures=True)
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = None
    try:
        node = DmpcNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch and the test process group can both deliver SIGINT. Finish
        # joining the IPOPT worker before Python tears down its native state.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
