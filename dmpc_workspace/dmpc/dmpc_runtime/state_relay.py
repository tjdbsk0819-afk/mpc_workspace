"""Observe peer odometry and publish only world-frame position/velocity/time."""
import time
import math
import signal

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from antbot_interfaces.msg import PeerState

from .observations import PeerStateCache, state_from_body_odometry
from .ros_time import ros_stamp, valid_stamp


class StateRelay(Node):
    def __init__(self):
        super().__init__("peer_state_relay")
        for key, value in dict(ego_robot="robot_a", peer_robot="robot_b", state_timeout=0.35).items():
            self.declare_parameter(key, value)
        self.ego, self.peer = (self.get_parameter(key).value for key in ("ego_robot", "peer_robot"))
        self.timeout = self.get_parameter("state_timeout").value
        if (not self.ego or not self.peer or self.ego == self.peer or "/" in self.ego+self.peer or
                not math.isfinite(self.timeout) or self.timeout <= 0):
            raise ValueError("Distinct robot names and a positive timeout are required")
        self.cache = PeerStateCache()
        self.pub = self.create_publisher(PeerState, "peer_state", 1)
        self.create_subscription(Odometry, f"/{self.peer}/odom", self.on_odom, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self, msg):
        now = self.get_clock().now().nanoseconds
        self.cache.observe_clock(now)
        try:
            if msg.header.frame_id != "world" or msg.child_frame_id != f"{self.peer}/base_link":
                raise ValueError("Odometry coordinate frame mismatch")
            p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
            stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if not valid_stamp(msg.header.stamp) or stamp > now+self.timeout*1e9:
                raise ValueError("Invalid odometry timestamp")
            state = state_from_body_odometry(stamp, self.peer, "world", (p.x, p.y, p.z),
                                            (q.x, q.y, q.z, q.w), (v.x, v.y, v.z))
            self.cache.update(state, time.monotonic_ns())
        except ValueError:
            self.cache.invalidate()

    def tick(self):
        now = self.get_clock().now().nanoseconds
        self.cache.observe_clock(now)
        status = self.cache.validity(now, time.monotonic_ns(), self.timeout*1e9, self.timeout*1e9)
        msg = PeerState()
        msg.header.frame_id = "world"
        state = self.cache.state
        msg.header.stamp = ros_stamp((state.stamp_ns if state else now)*1e-9)
        msg.robot_id, msg.valid, msg.status = self.peer, status == "ok", status
        if state is not None:
            msg.x, msg.y, msg.vx, msg.vy = state.x, state.y, state.vx, state.vy
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = None
    try:
        node = StateRelay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
