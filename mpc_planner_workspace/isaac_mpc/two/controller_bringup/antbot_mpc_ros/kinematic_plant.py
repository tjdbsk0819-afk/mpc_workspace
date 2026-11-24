"""Two ideal omnidirectional robots driven by actual ROS Twist messages."""
import csv
import json
from pathlib import Path as FilePath
import time
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String
from legacy_mpc_reference.config import MPCConfig
from legacy_mpc_reference.model import step, segment_distance
from legacy_mpc_reference.scenarios import scenario, RouteTracker
from .messages import stamp, set_pose


class KinematicPlant(Node):
    def __init__(self):
        super().__init__('kinematic_plant')
        for key, value in {'scenario': 'A', 'mode': 'centralized', 'config_path': '',
                           'output': 'antbot_results/ros', 'physics_dt': .02,
                           'command_timeout': .5, 'robot_names': ['robot_a', 'robot_b']}.items():
            self.declare_parameter(key, value)
        self.cfg = MPCConfig.load(self.get_parameter('config_path').value or None)
        self.spec = scenario(self.get_parameter('scenario').value)
        self.mode = self.get_parameter('mode').value
        self.names = self.get_parameter('robot_names').value
        self.dt = self.get_parameter('physics_dt').value
        self.timeout = self.get_parameter('command_timeout').value
        if len(self.names) != 2 or self.dt <= 0 or self.timeout <= 0:
            raise ValueError('Invalid plant configuration')
        self.states = self.spec.starts.copy()
        self.commands = np.zeros((2, 3))
        self.received = [-float('inf'), -float('inf')]
        self.clock_pub = self.create_publisher(ClockMessage, '/clock', 10)
        self.odom_pub = []
        self.inputs = []
        for m, name in enumerate(self.names):
            self.odom_pub.append(self.create_publisher(Odometry, f'/{name}/odom', qos_profile_sensor_data))
            self.inputs.append(self.create_subscription(Twist, f'/{name}/cmd_vel',
                               lambda message, m=m: self.on_command(message, m), 1))
        status_topics = ['/centralized_mpc/status'] if self.mode == 'centralized' else [f'/{name}/mpc/status' for name in self.names]
        self.status_inputs = [self.create_subscription(String, topic, self.on_status, 10) for topic in status_topics]
        self.status_events = []
        self.tracker = RouteTracker(self.spec, self.cfg)
        self.arrivals = [None, None]
        self.lengths = np.zeros(2)
        self.minimum = float('inf')
        self.collisions, self.colliding = 0, False
        self.obstacle_collisions, self.obstacle_colliding = 0, False
        self.safety_violations = 0
        self.now, self.done_at = 0., None
        self.rows = []
        self.saved = False
        self.timer = self.create_timer(self.dt, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Scenario {self.spec.name}: publishing common world-frame ground-truth odometry')

    def on_command(self, message, m):
        command = np.array([message.linear.x, message.linear.y, message.angular.z])
        if not np.isfinite(command).all() or np.any(np.abs(command) > np.array(self.cfg.velocity)+1e-4):
            self.commands[m] = 0.
            return
        self.commands[m] = command
        self.received[m] = time.monotonic()

    def on_status(self, message):
        try:
            self.status_events.append({'simulation_time': self.now, **json.loads(message.data)})
        except (ValueError, TypeError):
            pass

    def tick(self):
        wall = time.monotonic()
        applied = self.commands.copy()
        for m in range(2):
            if wall-self.received[m] > self.timeout or self.done_at is not None:
                applied[m] = 0.
        before = self.states.copy()
        self.states = np.array([step(x, u, self.dt) for x, u in zip(self.states, applied)])
        self.now += self.dt
        distance = segment_distance(before[0], self.states[0], before[1], self.states[1])
        self.minimum = min(self.minimum, distance)
        collision = distance < 2*self.cfg.radius-1e-6
        self.collisions += int(collision and not self.colliding)
        self.colliding = collision
        obstacle_collision = any(segment_distance(before[m], self.states[m], [ox, oy], [ox, oy]) < self.cfg.radius+radius-1e-6
                                 for m in range(2) for ox, oy, radius in self.spec.obstacles)
        self.obstacle_collisions += int(obstacle_collision and not self.obstacle_colliding)
        self.obstacle_colliding = obstacle_collision
        self.safety_violations += int(distance < self.cfg.separation-1e-5)
        self.lengths += np.linalg.norm(self.states[:, :2]-before[:, :2], axis=1)
        stamp_now = stamp(self.now)
        self.clock_pub.publish(ClockMessage(clock=stamp_now))
        row = {'time': self.now, 'min_distance': distance}
        for m in range(2):
            odom = Odometry()
            odom.header.stamp = stamp_now
            odom.header.frame_id = 'world'
            odom.child_frame_id = f'{self.names[m]}/base_link'
            set_pose(odom.pose.pose, self.states[m])
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = map(float, applied[m])
            self.odom_pub[m].publish(odom)
            for j, name in enumerate(('x', 'y', 'yaw')):
                row[f'robot_{m}_{name}'] = self.states[m, j]
            for j, name in enumerate(('vx', 'vy', 'wz')):
                row[f'robot_{m}_{name}'] = applied[m, j]
            if self.tracker.reached(m, self.states[m]) and self.arrivals[m] is None:
                self.arrivals[m] = self.now
        self.rows.append(row)
        if self.done_at is None and (all(self.tracker.reached(m, self.states[m]) for m in range(2)) or self.now >= self.cfg.max_time):
            self.done_at = wall
            self.save()
        if self.done_at is not None and wall-self.done_at > 1.:
            self.timer.cancel()
            rclpy.try_shutdown()

    def save(self):
        if self.saved or not self.rows:
            return
        self.saved = True
        output = FilePath(self.get_parameter('output').value)/f'{self.spec.name}_{self.mode}'
        output.mkdir(parents=True, exist_ok=True)
        with (output/'timeseries.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.rows[0]))
            writer.writeheader()
            writer.writerows(self.rows)
        tracking = [s for s in self.status_events if s.get('reason') == 'tracking']
        summary = {'mode': self.mode, 'scenario': self.spec.name,
                   'reached': all(self.tracker.reached(m, self.states[m]) for m in range(2)),
                   'duration_seconds': self.now, 'arrival_seconds': self.arrivals,
                   'collision_count': self.collisions, 'obstacle_collision_count': self.obstacle_collisions,
                   'min_distance': self.minimum, 'required_distance': self.cfg.separation,
                   'safety_violation_steps': self.safety_violations, 'path_length': self.lengths.tolist(),
                   'solver_seconds_mean': float(np.mean([s['solver_seconds'] for s in tracking])) if tracking else None,
                   'solver_deadline_events': sum(s.get('reason') == 'solver_deadline_exceeded' for s in self.status_events),
                   'solver_failure_events': sum(str(s.get('reason', '')).startswith('solver_failed') for s in self.status_events),
                   'first_command_safety_vetoes': sum(s.get('reason') == 'first_command_safety_veto' for s in self.status_events),
                   'final_goal_errors': np.linalg.norm(self.states[:, :2]-self.spec.goals[:, :2], axis=1).tolist()}
        (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        with (output/'events.jsonl').open('w') as stream:
            for event in self.status_events:
                stream.write(json.dumps(event)+'\n')
        self.get_logger().info(json.dumps(summary))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KinematicPlant()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node:
            node.save()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
