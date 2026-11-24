#!/usr/bin/env python3
"""RViz markers for the current parallel-lane two-Antbot experiment."""
import copy
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import qos_profile_sensor_data


class Paths(Node):
    def __init__(self):
        super().__init__('two_robot_path_visualizer')
        self.paths = {}
        self.poses = {}
        self.offsets = {'robot_a': -2.0, 'robot_b': 2.0}
        self.colors = {'robot_a': (0.1, 0.6, 1.0), 'robot_b': (1.0, 0.4, 0.1)}
        self.pub = self.create_publisher(MarkerArray, '/multi_robot/path_markers', 1)
        self.subs = []
        for robot in self.offsets:
            self.subs.append(self.create_subscription(
                Path, f'/{robot}/reference_path',
                lambda msg, name=robot: self.paths.__setitem__(name, msg), 1))
            self.subs.append(self.create_subscription(
                Odometry, f'/{robot}/odom',
                lambda msg, name=robot: self.poses.__setitem__(name, msg),
                qos_profile_sensor_data))
        self.timer = self.create_timer(0.2, self.publish)

    def marker(self, robot, ident, kind):
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = robot
        m.id = ident
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = self.colors[robot]
        m.color.a = 1.0
        return m

    def publish(self):
        output = MarkerArray()
        for robot, offset in self.offsets.items():
            if robot in self.paths:
                line = self.marker(robot, 0, Marker.LINE_STRIP)
                line.scale.x = 0.05
                for pose in self.paths[robot].poses:
                    point = Point()
                    point.x = pose.pose.position.x
                    point.y = pose.pose.position.y + offset
                    point.z = 0.08
                    line.points.append(point)
                output.markers.append(line)
            if robot in self.poses:
                body = self.marker(robot, 1, Marker.ARROW)
                body.pose = copy.deepcopy(self.poses[robot].pose.pose)
                body.pose.position.y += offset
                body.pose.position.z = 0.2
                body.scale.x, body.scale.y, body.scale.z = 0.7, 0.2, 0.2
                output.markers.append(body)
        self.pub.publish(output)


if __name__ == '__main__':
    rclpy.init()
    node = Paths()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
