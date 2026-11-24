"""ROS state/command adapter. IPOPT runs in a separately killable child process."""
import json
import os
import time
os.environ.setdefault('OMP_NUM_THREADS', '1')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from legacy_mpc_reference.config import MPCConfig
from legacy_mpc_reference.planner import command_is_safe
from legacy_mpc_reference.ros_contract import fresh_stamp, local_command_is_safe
from legacy_mpc_reference.scenarios import scenario, RouteTracker
from legacy_mpc_reference.trajectory import Prediction, hold_prediction
from legacy_mpc_reference.worker import SolverWorker
from .messages import seconds, yaw, twist, prediction_message, parse_prediction


class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc')
        defaults = {'mode': 'centralized', 'robot_index': 0, 'scenario': 'A',
                    'config_path': '', 'world_frame': 'world',
                    'robot_names': ['robot_a', 'robot_b'], 'solver_startup_timeout': 15.}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.mode = self.get_parameter('mode').value
        self.index = self.get_parameter('robot_index').value
        self.frame = self.get_parameter('world_frame').value
        self.names = self.get_parameter('robot_names').value
        if self.mode not in ('centralized', 'distributed') or self.index not in (0, 1) or len(self.names) != 2:
            raise ValueError('Invalid mode, robot_index, or robot_names')
        self.cfg = MPCConfig.load(self.get_parameter('config_path').value or None)
        self.spec = scenario(self.get_parameter('scenario').value)
        self.tracker = RouteTracker(self.spec, self.cfg)
        self.controlled = [0, 1] if self.mode == 'centralized' else [self.index]
        self.states = [None, None]
        self.peer_prediction, self.peer_received = None, 0.
        self.command = np.zeros((len(self.controlled), 3))
        self.cmd_pub, self.plan_pub = {}, {}
        self._input_subscriptions = []
        for m, robot in enumerate(self.names):
            self._input_subscriptions.append(self.create_subscription(Odometry, f'/{robot}/odom',
                                      lambda message, m=m: self.on_odom(message, m), qos_profile_sensor_data))
            if m in self.controlled:
                self.cmd_pub[m] = self.create_publisher(Twist, f'/{robot}/cmd_vel', 1)
                self.plan_pub[m] = self.create_publisher(Path, f'/{robot}/predicted_trajectory', 1)
        if self.mode == 'distributed':
            self._input_subscriptions.append(self.create_subscription(Path,
                f'/{self.names[1-self.index]}/predicted_trajectory', self.on_prediction, 1))
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.worker = None
        self.start_worker()
        self.submitted = None
        self.last_solve = -float('inf')
        self.last_result_wall = -float('inf')
        self.last_sim = None
        self.last_clock_advance = time.monotonic()
        self.last_hold = -float('inf')
        self.last_status = None
        self.timer = self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f'{self.mode} MPC controls {[self.names[m] for m in self.controlled]} in frame {self.frame}; '
            f'dt={self.cfg.dt}s, horizon={self.cfg.horizon}, IPOPT budget={self.cfg.solver_max_seconds}s')

    def start_worker(self):
        if self.worker:
            self.worker.close()
        self.worker = SolverWorker(self.cfg, len(self.controlled), self.spec.obstacles, self.spec.bounds)

    def on_odom(self, message, m):
        try:
            if message.header.frame_id != self.frame or message.child_frame_id != f'{self.names[m]}/base_link':
                raise ValueError('Odometry must use common world frame and robot-specific base_link')
            state = np.array([message.pose.pose.position.x, message.pose.pose.position.y, yaw(message.pose.pose.orientation)])
            velocity = np.array([message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.angular.z])
            if not np.isfinite(state).all() or not np.isfinite(velocity).all():
                raise ValueError('Non-finite odometry')
            epoch = seconds(message.header.stamp)
            if self.states[m] is not None and epoch < self.states[m][2]:
                return
            self.states[m] = state, velocity, epoch, time.monotonic()
        except ValueError as error:
            self.states[m] = None
            self.report(str(error))

    def on_prediction(self, message):
        try:
            prediction = parse_prediction(message, self.frame)
            if self.peer_prediction and prediction.stamp < self.peer_prediction.stamp:
                return
            self.peer_prediction = prediction
            self.peer_received = time.monotonic()
        except ValueError as error:
            self.peer_prediction = None
            self.report(str(error))

    def report(self, reason, **metrics):
        message = String(data=json.dumps({'reason': reason, 'mode': self.mode,
                                         'robot_index': self.index, **metrics}))
        self.status_pub.publish(message)
        if reason != self.last_status:
            self.get_logger().info(reason)
            self.last_status = reason

    def send_commands(self):
        for k, m in enumerate(self.controlled):
            self.cmd_pub[m].publish(twist(self.command[k]))

    def stop(self, reason, now, **metrics):
        self.command[:] = 0.
        self.send_commands()
        self.report(reason, **metrics)
        if now-self.last_hold >= self.cfg.dt:
            for m in self.controlled:
                if self.states[m] is not None:
                    self.plan_pub[m].publish(prediction_message(hold_prediction(self.states[m][0], now, self.cfg, self.frame)))
            self.last_hold = now

    def valid_states(self, now, wall):
        return all(s is not None and fresh_stamp(s[2], now, s[3], wall, self.cfg.odom_timeout) for s in self.states)

    def peer_plan(self, now, wall):
        if self.peer_prediction is None or not fresh_stamp(self.peer_prediction.stamp, now,
                self.peer_received, wall, self.cfg.stale_timeout):
            raise ValueError('waiting_for_fresh_peer_prediction')
        aligned, inflation = self.peer_prediction.align(now, self.cfg, self.frame)
        actual = self.states[1-self.index][0]
        if np.linalg.norm(actual[:2]-aligned[0, :2]) > self.cfg.margin+inflation[0]:
            raise ValueError('peer_prediction_disagrees_with_odometry')
        return aligned, inflation

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        wall = time.monotonic()
        if self.last_sim is not None and now < self.last_sim-1e-6:
            self.states = [None, None]
            self.peer_prediction = None
            self.tracker = RouteTracker(self.spec, self.cfg)
            self.submitted = None
            self.last_solve = self.last_hold = -float('inf')
            self.start_worker()
        if self.last_sim is None or now > self.last_sim+1e-9:
            self.last_clock_advance = wall
        self.last_sim = now
        # Drain the result pipe before testing busy/deadline. A ROS callback
        # delayed by scheduling must not kill a child whose result is ready.
        # Late or stale results are still rejected below.
        event = self.worker.poll()
        if event:
            kind, plan = event
            if kind == 'error':
                self.start_worker()
                self.submitted = None
                self.stop(f'solver_process_error: {plan}', now)
                return
            if kind == 'result':
                epoch = self.submitted
                self.submitted = None
                if epoch is None or not plan.success or now-epoch > self.cfg.dt or not self.valid_states(now, wall):
                    self.stop('invalid_or_late_solution' if plan.success else plan.status, now,
                              solver_seconds=plan.seconds, budget_seconds=self.cfg.solver_max_seconds)
                    return
                current = np.array([s[0] for s in self.states])
                proposed = plan.command.copy()
                if self.mode == 'centralized':
                    safe = command_is_safe(current, proposed, self.cfg, self.spec.obstacles, self.spec.bounds)
                else:
                    try:
                        peer, inflation = self.peer_plan(now, wall)
                        safe = local_command_is_safe(current[self.index], proposed[0], peer, inflation,
                                                     self.cfg, self.spec.obstacles, self.spec.bounds)
                    except ValueError:
                        safe = False
                if not safe:
                    self.stop('first_command_safety_veto', now)
                    return
                self.command = proposed
                self.last_result_wall = wall
                for k, m in enumerate(self.controlled):
                    self.plan_pub[m].publish(prediction_message(Prediction(epoch, self.cfg.dt, plan.states[k], self.frame)))
                self.report('tracking', solver_seconds=plan.seconds)
        if self.worker.busy and wall-self.worker.started > self.cfg.dt:
            self.start_worker()
            self.submitted = None
            self.stop('solver_deadline_exceeded', now)
            return
        if now <= 0 or wall-self.last_clock_advance > self.cfg.odom_timeout or not self.valid_states(now, wall):
            self.stop('waiting_for_fresh_clock_and_odometry', now)
            return
        if all(self.tracker.reached(m, self.states[m][0]) for m in self.controlled):
            self.stop('goal_reached', now)
            return
        if self.mode == 'distributed':
            try:
                self.peer_plan(now, wall)
            except ValueError as error:
                self.stop(str(error), now)
                return
        if wall-self.last_result_wall > 2*self.cfg.dt:
            self.stop('waiting_for_valid_solution', now)
        else:
            self.send_commands()
        if not self.worker.ready:
            if wall-self.worker.started > self.get_parameter('solver_startup_timeout').value:
                self.start_worker()
                self.stop('solver_startup_timeout', now)
            return
        if not self.worker.busy and now-self.last_solve >= self.cfg.dt-1e-6:
            states = np.array([self.states[m][0] for m in self.controlled])
            previous = np.array([self.states[m][1] for m in self.controlled])
            targets = np.array([self.tracker.target(m, self.states[m][0]) for m in self.controlled])
            request = {'states': states, 'targets': targets, 'previous': previous}
            if self.mode == 'distributed':
                try:
                    request['peer'], request['inflation'] = self.peer_plan(now, wall)
                except ValueError as error:
                    self.stop(str(error), now)
                    return
            if self.worker.submit(request):
                self.submitted, self.last_solve = now, now

    def close(self):
        self.command[:] = 0.
        if rclpy.ok():
            self.send_commands()
        self.worker.close()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = MPCNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.close()
        if rclpy.ok():
            rclpy.shutdown()
