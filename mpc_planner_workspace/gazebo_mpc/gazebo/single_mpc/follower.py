"""Track three timed references using live odometry; never publish fake odometry."""
import argparse
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool
from controller_manager_msgs.srv import ListControllers
from reference import load_reference, Trajectory, tracking_command, motion_start_time


class ReferenceFollower(Node):
    def __init__(self, args):
        super().__init__('fixed_reference_followers', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.data = load_reference(args.reference, args.scenario)
        self.ego = args.ego
        self.source_offsets = {r:motion_start_time(rows) for r,rows in self.data['robots'].items()}
        self.ego_enabled_stamp = None
        self.trajectories = {r: Trajectory(rows) for r, rows in self.data['robots'].items() if r != self.ego}
        self.duration = max(t.times[-1]-self.source_offsets[r] for r,t in self.trajectories.items())
        self.states = {}; self.received = {}; self.commands = {r:(0.,0.,0.) for r in self.trajectories}
        self.publishers_cmd = {r:self.create_publisher(Twist, f'/{r}/cmd_vel', 1) for r in self.trajectories}
        self.path_pubs = {r:self.create_publisher(Path, f'/{r}/mpc_path', 1) for r in self.trajectories}
        self.status_pub = self.create_publisher(String, '/fixed_reference/status', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.phase = 'waiting'; self.reason = ''; self.epoch = None; self.last_sim = None
        self.enable_future = None
        self.enable = self.create_client(SetBool, f'/{self.ego}/mpc_rpp_controller/set_enabled')
        self.controllers = {r:self.create_client(ListControllers, f'/{r}/controller_manager/list_controllers') for r in self.data['robots']}
        self.controller_requests = {}; self.active = set(); self.last_control_check = -math.inf
        self.last_status_wall = -math.inf
        for r in self.data['robots']:
            self.create_subscription(Odometry, f'/{r}/odom', lambda msg, robot=r:self.on_odom(msg, robot), qos_profile_sensor_data)
        self.create_timer(.05, self.tick)

    def on_odom(self, msg, robot):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        values = (p.x,p.y,yaw,math.hypot(v.x,v.y),stamp)
        if msg.header.frame_id == 'world' and msg.child_frame_id == f'{robot}/base_link' and all(math.isfinite(v) for v in values):
            self.states[robot] = values; self.received[robot] = time.monotonic()

    def stop(self):
        for robot, pub in self.publishers_cmd.items():
            pub.publish(Twist()); self.commands[robot] = (0.,0.,0.)

    def check_controllers(self):
        wall = time.monotonic()
        for robot, (future, sent) in list(self.controller_requests.items()):
            if future.done():
                response = future.result()
                if response and any(c.name == 'antbot_swerve_controller' and c.state == 'active' for c in response.controller):
                    self.active.add(robot)
                del self.controller_requests[robot]
            elif wall-sent > 2.:
                self.controllers[robot].remove_pending_request(future)
                del self.controller_requests[robot]
        if wall-self.last_control_check > .5:
            for robot, client in self.controllers.items():
                if robot not in self.active and robot not in self.controller_requests and client.service_is_ready():
                    self.controller_requests[robot] = (client.call_async(ListControllers.Request()), wall)
            self.last_control_check = wall

    def publish_reference_paths(self, now):
        # Visualization only: these are recorded reference paths, not MPC outputs.
        elapsed = 0. if self.epoch is None else now-self.epoch
        for robot, trajectory in self.trajectories.items():
            reference_time = self.source_offsets[robot]+elapsed
            path = Path(); path.header.frame_id = 'world'
            path.header.stamp = self.get_clock().now().to_msg()
            remaining = max(0.,trajectory.times[-1]-reference_time)
            samples = min(200,max(2,math.ceil(remaining/.2)+1))
            for step in range(samples):
                future = remaining*step/(samples-1)
                x,y,yaw,_,_ = trajectory.at(reference_time+future)
                pose = PoseStamped(); pose.header.frame_id = 'world'
                pose.header.stamp = rclpy.time.Time(seconds=now+future).to_msg()
                pose.pose.position.x = x; pose.pose.position.y = y
                pose.pose.orientation.z = math.sin(yaw/2.)
                pose.pose.orientation.w = math.cos(yaw/2.)
                path.poses.append(pose)
            self.path_pubs[robot].publish(path)

    def publish_status(self, now):
        if time.monotonic()-self.last_status_wall < .2:
            return
        self.publish_reference_paths(now)
        record = dict(phase=self.phase, reason=self.reason, ego=self.ego, epoch=self.epoch,
                      elapsed=0. if self.epoch is None else now-self.epoch,
                      reference_duration=self.duration,
                      start_mode='simultaneous', source_offsets=self.source_offsets, ego_release_delay=0., ego_enabled_stamp=self.ego_enabled_stamp,
                      automatic_abort=False)
        self.status_pub.publish(String(data=json.dumps(record)))
        self.last_status_wall = time.monotonic()

    def tick(self):
        now = self.get_clock().now().nanoseconds*1e-9
        previous_sim = self.last_sim; self.last_sim = now
        if previous_sim is not None and now < previous_sim and self.epoch is not None:
            # Keep the reference progress continuous across a simulation clock reset.
            self.epoch = now-max(0.,previous_sim-self.epoch)
            self.ego_enabled_stamp = min(self.ego_enabled_stamp,now)
        if self.phase == 'waiting':
            self.stop(); self.check_controllers()
            if len(self.states) == 4 and len(self.active) == 4 and self.enable.service_is_ready():
                if self.enable_future is None:
                    self.enable_future = self.enable.call_async(SetBool.Request(data=True))
                if self.enable_future.done():
                    if self.enable_future.result() is not None and self.enable_future.result().success:
                        self.phase = 'running'; self.epoch = now; self.ego_enabled_stamp = now
                        self.get_logger().info('All four robots released together; run until user stops')
                    else:
                        self.enable_future = None
        elif self.phase in ('running', 'holding'):
            elapsed = now-self.epoch
            dt = min(.1,max(.001,now-(previous_sim if previous_sim is not None else now-.05)))
            for robot, trajectory in self.trajectories.items():
                reference_time = elapsed+self.source_offsets[robot]
                reference = trajectory.at(reference_time)
                v,vy,w = tracking_command(self.states[robot],reference,self.commands[robot],dt,
                                           trajectory.world_velocity(reference_time))
                msg = Twist(); msg.linear.x = v; msg.linear.y = vy; msg.angular.z = w
                self.publishers_cmd[robot].publish(msg); self.commands[robot] = (v,vy,w)
            if elapsed >= self.duration:
                self.phase = 'holding'
        self.publish_status(now)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--reference', required=True); parser.add_argument('--scenario', required=True)
    parser.add_argument('--ego', choices=[f'robot_{r}' for r in 'abcd'], default='robot_a')
    args = parser.parse_args(); rclpy.init(); node = ReferenceFollower(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok(): node.stop()
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
