"""Optional ROS transport test with synthetic GT and an isolated ROS domain.

Run after building/sourcing the interfaces and ROS package, with a Zenoh router:
RUN_GT_PEER_ROS_TESTS=1 python -m unittest discover -s tests -p test_gt_peer_ros.py -v
No Isaac instance or velocity-command publisher is used.
"""
import math
import os
from pathlib import Path
import sys
import time
import unittest


@unittest.skipUnless(os.environ.get('RUN_GT_PEER_ROS_TESTS') == '1',
                     'Enable explicitly in the built ROS environment')
class GroundTruthRosTests(unittest.TestCase):
    def test_two_independent_receivers_and_stale_prediction_clear(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'controller_bringup'))
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from nav_msgs.msg import Odometry, Path as RosPath
        from rosgraph_msgs.msg import Clock
        from antbot_interfaces.msg import PeerState
        from antbot_mpc_ros.gt_peer_predictor import GtPeerPredictor

        context = Context()
        rclpy.init(args=[], context=context, domain_id=231)
        executor = SingleThreadedExecutor(context=context)
        nodes = []
        try:
            driver = Node('gt_cv_test_driver', context=context)
            nodes.append(driver)
            states, paths, subscriptions = {}, {}, []
            odom_publishers = {}
            clock_pub = driver.create_publisher(Clock, '/clock', 10)
            for ego, peer in (('robot_a', 'robot_b'), ('robot_b', 'robot_a')):
                node = GtPeerPredictor(context=context, namespace=ego, parameter_overrides=[
                    Parameter('use_sim_time', value=True),
                    Parameter('ego_robot', value=ego), Parameter('peer_robot', value=peer),
                    Parameter('prediction_steps', value=4),
                ])
                nodes.append(node)
                odom_publishers[ego] = driver.create_publisher(Odometry, f'/{ego}/odom', 1)
                subscriptions.extend([
                    driver.create_subscription(PeerState, f'/{ego}/peer_state',
                        lambda msg, ego=ego: states.__setitem__(ego, msg), 1),
                    driver.create_subscription(RosPath, f'/{ego}/peer_prediction',
                        lambda msg, ego=ego: paths.__setitem__(ego, msg), 1),
                ])
            for node in nodes:
                executor.add_node(node)

            def publish_gt(stamp_ns):
                clock = Clock()
                clock.clock.sec, clock.clock.nanosec = divmod(stamp_ns, 1_000_000_000)
                clock_pub.publish(clock)
                for name, yaw, x, y in (('robot_a', math.pi, -2., -3.),
                                         ('robot_b', math.pi/2, 2., 3.)):
                    odom = Odometry()
                    odom.header.frame_id = 'world'
                    odom.header.stamp = clock.clock
                    odom.child_frame_id = f'{name}/base_link'
                    odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
                    odom.pose.pose.orientation.z = math.sin(yaw/2)
                    odom.pose.pose.orientation.w = math.cos(yaw/2)
                    odom.twist.twist.linear.x = 1.0
                    odom_publishers[name].publish(odom)

            stamp_ns = 1_000_000_000
            deadline = time.monotonic() + 8
            ready = False
            while time.monotonic() < deadline:
                publish_gt(stamp_ns)
                for _ in range(12):
                    executor.spin_once(timeout_sec=0.005)
                ready = all(name in states and states[name].valid and
                            name in paths and len(paths[name].poses) == 4
                            for name in ('robot_a', 'robot_b'))
                if ready:
                    break
                stamp_ns += 20_000_000
            self.assertTrue(ready, 'No valid GT/CV output; check ROS messages and the Zenoh router')
            self.assertEqual(states['robot_a'].robot_id, 'robot_b')
            self.assertEqual(states['robot_b'].robot_id, 'robot_a')
            self.assertAlmostEqual(states['robot_a'].vx, 0, places=6)
            self.assertAlmostEqual(states['robot_a'].vy, 1, places=6)
            self.assertAlmostEqual(states['robot_b'].vx, -1, places=6)
            self.assertAlmostEqual(states['robot_b'].vy, 0, places=6)
            self.assertAlmostEqual(paths['robot_a'].poses[1].pose.position.y -
                                   paths['robot_a'].poses[0].pose.position.y, .2)
            self.assertAlmostEqual(paths['robot_b'].poses[1].pose.position.x -
                                   paths['robot_b'].poses[0].pose.position.x, -.2)

            # Stop both clock and odom: wall-time freshness must still expire.
            deadline = time.monotonic() + 3
            expired = False
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.02)
                expired = all(not states[name].valid and not paths[name].poses
                              for name in ('robot_a', 'robot_b'))
                if expired:
                    break
            self.assertTrue(expired, 'A paused/stopped GT source retained a valid prediction')
        finally:
            for node in reversed(nodes):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            rclpy.shutdown(context=context)


if __name__ == '__main__':
    unittest.main()
