"""Relay world-frame peer GT and publish the configured predictive distribution."""
import json
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from antbot_interfaces.msg import PeerState
from std_msgs.msg import String

from .cv_model import PeerStateCache, state_from_body_odometry
from .peer_prediction import distribution


def stamp_message(nanoseconds):
    sec, nanosec = divmod(nanoseconds, 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


class GtPeerPredictor(Node):
    def __init__(self, **kwargs):
        super().__init__('gt_peer_predictor', **kwargs)
        defaults = {
            'ego_robot': 'robot_a', 'peer_robot': 'robot_b', 'world_frame': 'world',
            'peer_odom_topic': '', 'prediction_steps': 30, 'prediction_dt': 0.2,
            'publish_frequency': 20.0, 'max_state_age': 0.5, 'max_receive_age': 0.5,
            'peer_prediction_model': 'cv', 'peer_prediction_noise': .3,
            'peer_prediction_dt': .2, 'peer_prediction_risk': .05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.ego = self.get_parameter('ego_robot').value
        self.peer = self.get_parameter('peer_robot').value
        self.frame = self.get_parameter('world_frame').value
        if not self.ego or not self.peer or self.ego == self.peer or '/' in self.ego or '/' in self.peer:
            raise ValueError('ego_robot and peer_robot must be distinct unqualified robot names')
        self.steps = self.get_parameter('prediction_steps').value
        if not 2 <= self.steps <= 1000:
            raise ValueError('prediction_steps must be between 2 and 1000')
        values = {name: self.get_parameter(name).value for name in (
            'prediction_dt', 'publish_frequency', 'max_state_age', 'max_receive_age')}
        if not all(math.isfinite(value) and value > 0 for value in values.values()):
            raise ValueError('Prediction interval, rate, and age limits must be positive and finite')
        self.dt_ns = round(values['prediction_dt']*1e9)
        self.model=self.get_parameter('peer_prediction_model').value
        self.noise=self.get_parameter('peer_prediction_noise').value
        self.risk=self.get_parameter('peer_prediction_risk').value
        if self.model not in ('cv','cv_gaussian','gaussian_random_walk') or not math.isfinite(self.noise) or self.noise<=0 or not 0<self.risk<1:
            raise ValueError('Invalid peer prediction model/configuration')
        if abs(self.get_parameter('peer_prediction_dt').value-values['prediction_dt'])>1e-9:
            raise ValueError('Diagnostic and configured peer prediction intervals must match')
        if self.dt_ns < 1:
            raise ValueError('prediction_dt must be at least one nanosecond')
        self.max_state_age_ns = round(values['max_state_age']*1e9)
        self.max_receive_age_ns = round(values['max_receive_age']*1e9)
        topic = self.get_parameter('peer_odom_topic').value or f'/{self.peer}/odom'
        self.cache = PeerStateCache()
        self.invalid_reason = 'waiting_for_peer'
        self.last_status = None
        self.state_pub = self.create_publisher(PeerState, f'/{self.ego}/peer_state', QoSProfile(depth=1))
        self.prediction_pub = self.create_publisher(Path, f'/{self.ego}/peer_prediction', QoSProfile(depth=1))
        self.distribution_pub=self.create_publisher(String,f'/{self.ego}/peer_prediction_distribution',QoSProfile(depth=1))
        # Accept both the old best-effort Isaac odom and the reliable RPP-compatible one.
        self.peer_sub = self.create_subscription(Odometry, topic, self.on_odom, qos_profile_sensor_data)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0/values['publish_frequency'], self.tick, clock=self.wall_clock)
        self.get_logger().info(
            f'{self.ego} observes {self.peer}: {topic} -> /{self.ego}/peer_state '
            f'and /{self.ego}/peer_prediction; frame={self.frame}, '
            f'steps={self.steps}, dt={self.dt_ns/1e9}s, model={self.model}')

    def observe_clock(self):
        now = self.get_clock().now().nanoseconds
        if self.cache.observe_clock(now):
            self.invalid_reason = 'clock_reset'
        return now

    def on_odom(self, message):
        now = self.observe_clock()
        try:
            if message.header.frame_id != self.frame:
                raise ValueError(f'Expected pose frame {self.frame}')
            if message.child_frame_id != f'{self.peer}/base_link':
                raise ValueError(f'Expected twist frame {self.peer}/base_link')
            stamp = message.header.stamp
            if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
                raise ValueError('Invalid odometry timestamp')
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            velocity = message.twist.twist.linear
            state = state_from_body_odometry(
                stamp.sec*1_000_000_000 + stamp.nanosec, self.peer, self.frame,
                (position.x, position.y, position.z),
                (orientation.x, orientation.y, orientation.z, orientation.w),
                (velocity.x, velocity.y, velocity.z),
            )
            if state.stamp_ns > now + self.max_state_age_ns:
                raise ValueError('Odometry is too far ahead of the simulation clock')
            if self.cache.update(state, time.monotonic_ns()):
                self.invalid_reason = 'waiting_for_peer'
        except (ValueError, TypeError) as error:
            self.cache.invalidate()
            self.invalid_reason = f'invalid_odometry: {error}'

    def tick(self):
        now = self.observe_clock()
        status = self.cache.validity(
            now, time.monotonic_ns(), self.max_state_age_ns, self.max_receive_age_ns)
        if status == 'waiting_for_peer':
            status = self.invalid_reason
        valid = status == 'ok'
        sample = self.cache.state
        state = PeerState()
        state.header.frame_id = self.frame
        state.header.stamp = stamp_message(sample.stamp_ns if sample else now)
        state.robot_id = self.peer
        state.valid, state.status = valid, status
        if sample:
            state.x, state.y, state.vx, state.vy = sample.x, sample.y, sample.vx, sample.vy
        self.state_pub.publish(state)

        prediction = Path()
        prediction.header.frame_id = self.frame
        prediction.header.stamp = stamp_message(now)
        points=[]
        if valid:
            points=distribution(sample,now,self.dt_ns,self.steps,self.model,self.noise,self.risk)
            for p in points:
                pose = PoseStamped()
                pose.header.frame_id = self.frame
                pose.header.stamp = stamp_message(now+round(p['t']*1e9))
                pose.pose.position.x, pose.pose.position.y = p['x'], p['y']
                # No future body orientation is predicted by either model.
                pose.pose.orientation.w = 1.0
                prediction.poses.append(pose)
        # Empty paths explicitly invalidate a previously published prediction.
        self.prediction_pub.publish(prediction)
        msg=String();msg.data=json.dumps(dict(model=self.model,frame=self.frame,stamp=now/1e9,
                                             risk=self.risk,steps=points),allow_nan=False)
        self.distribution_pub.publish(msg)
        if status != self.last_status:
            # rclpy caches severity per call site. Separate call sites keep
            # waiting -> valid -> stale transitions from raising ValueError.
            if valid:
                self.get_logger().info(f'{self.ego} peer={self.peer}: {status}')
            else:
                self.get_logger().warning(f'{self.ego} peer={self.peer}: {status}')
            self.last_status = status


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GtPeerPredictor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
