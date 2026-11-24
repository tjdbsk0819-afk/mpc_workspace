#!/usr/bin/env python3
"""Publish a configurable straight path ahead of the first received Antbot pose."""
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class TestPath(Node):
    def __init__(self):
        super().__init__('test_reference_path')
        self.length = self.declare_parameter('path_length', 5.0).value
        if not math.isfinite(self.length) or self.length < 0.75:
            raise ValueError('path_length must be finite and at least 0.75 m')
        self.path = None
        self.publisher = self.create_publisher(Path, 'reference_path', 1)
        self.subscription = self.create_subscription(
            Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0, self.publish_path)

    def on_odom(self, msg):
        if self.path is not None:
            return
        if msg.header.frame_id != 'odom':
            self.get_logger().error('Expected odometry in odom frame')
            return
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        self.path = Path()
        self.path.header.frame_id = 'odom'
        steps = max(3, math.ceil(self.length / 0.25))
        for i in range(steps + 1):
            distance = self.length * i / steps
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = msg.pose.pose.position.x + distance * math.cos(yaw)
            pose.pose.position.y = msg.pose.pose.position.y + distance * math.sin(yaw)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)
            self.path.poses.append(pose)
        self.get_logger().info(f'Fixed {self.length:g} m reference path prepared')
        self.publish_path()

    def publish_path(self):
        if self.path is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.path.header.stamp = stamp
        for pose in self.path.poses:
            pose.header.stamp = stamp
        self.publisher.publish(self.path)


if __name__ == '__main__':
    rclpy.init()
    node = TestPath()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
