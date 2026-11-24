"""Legacy adapter for regression comparison; production uses native mpc_rpp_controller."""
import copy
import math
import json
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from antbot_interfaces.msg import MpcPlan, PeerState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .timed_tracking import Motion, TimedTrajectory, ProximityReleaseGate, step, approach, filter_command
from .rpp_path import rotation_prefix_count, alignment_heading


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


class MpcRppAdapter(Node):
    def __init__(self, **kwargs):
        super().__init__('mpc_rpp_adapter', **kwargs)
        defaults = dict(ego_robot='robot_a', peer_robot='robot_b', enabled=True,
                        plan_timeout=.35, state_timeout=.35, command_timeout=.15,
                        goal_frequency=10., output_frequency=20., max_speed=.3,
                        max_yaw_rate=.8, minimum_separation=1.15,
                        braking_deceleration=.5, reaction_time=.25,
                        max_acceleration=.5, max_yaw_acceleration=.5,
                        tracking_horizon=.6, path_sample_period=.025,
                        minimum_tracking_length=.05, maximum_tracking_horizon=2.0,
                        execution_buffer=.005, execution_gap_fraction=.9,
                        proximity_release_duration=.15, alignment_horizon=.4)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        for key in defaults:
            setattr(self, key, self.get_parameter(key).value)
        numeric = [getattr(self, k) for k in defaults if k not in (
            'ego_robot', 'peer_robot', 'enabled')]
        if self.ego_robot == self.peer_robot or not all(math.isfinite(x) and x > 0 for x in numeric):
            raise ValueError('Invalid adapter configuration')
        if self.execution_gap_fraction >= 1.:
            raise ValueError('execution_gap_fraction must be below one')
        self.proximity_release = ProximityReleaseGate(
            round(self.proximity_release_duration*1e9), round(2e9/self.output_frequency))
        self.plan = self.odom = self.peer = self.raw_command = None
        self.trajectory = None
        self.plan_wall = self.odom_wall = self.peer_wall = self.raw_wall = 0.
        self.last_clock = None
        self.goal_handle = None
        self.goal_pending = False
        self.epoch = self.sequence = self.active_sequence = 0
        self.active_since = 0.
        self.last_goal_stamp = -1
        self.last_send = -float('inf')
        self.last_status = None
        self.tracking_mode = 'tracking'
        self.invalid_reason = 'waiting_for_fresh_mpc_plan'
        self.diagnostics = {}
        self.alignment_yaw = None
        self.execution_state = None
        self.last_output = Twist()
        self.active_tracking_mode = None
        self.client = ActionClient(self, FollowPath, 'follow_path')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.speed_pub = self.create_publisher(SpeedLimit, 'speed_limit', 1)
        self.status_pub = self.create_publisher(String, 'control_status', 1)
        self.diagnostic_pub = self.create_publisher(String,'control_diagnostics',1)
        self.subscriptions_ = [
            self.create_subscription(MpcPlan, 'mpc_plan', self.on_plan, 1),
            self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data),
            self.create_subscription(PeerState, 'peer_state', self.on_peer, 1),
            self.create_subscription(Twist, 'rpp_cmd_vel', self.on_command, 1),
        ]
        self.enable_service = self.create_service(SetBool, '~/set_enabled', self.on_enable)
        self.timer = self.create_timer(1./self.output_frequency, self.tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def report(self, text):
        self.status_pub.publish(String(data=text))
        self.diagnostic_pub.publish(String(data=json.dumps(dict(self.diagnostics,status=text),allow_nan=False)))
        if text != self.last_status:
            self.get_logger().info(text)
            self.last_status = text

    def clear_goal(self):
        # Invalidate in-flight goal responses too; late accepted goals are canceled.
        self.epoch += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.raw_command = None

    def emit(self, command):
        self.last_output = copy.deepcopy(command)
        self.cmd_pub.publish(command)

    def hold(self, reason, cancel=True):
        self.proximity_release.interrupt()
        self.emit(Twist())
        if cancel and (self.goal_handle is not None or self.goal_pending):
            self.clear_goal()
        self.report(reason)

    def on_enable(self, request, response):
        self.enabled = request.data
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('enabled', value=self.enabled)])
        self.plan = self.trajectory = None
        self.alignment_yaw = None
        self.invalid_reason = 'waiting_for_new_plan'
        self.clear_goal()
        self.hold('waiting_for_new_plan' if self.enabled else 'disabled')
        response.success = True
        response.message = 'New valid MPC plan required' if self.enabled else 'Zero command and FollowPath cancellation requested'
        return response

    def on_plan(self, msg):
        if not msg.valid:
            self.plan = self.trajectory = None
            self.alignment_yaw = None
            self.invalid_reason = 'mpc_' + msg.status
            self.hold(self.invalid_reason)
            return
        try:
            path = msg.path
            epoch = stamp_ns(msg.header.stamp)
            if (msg.header.frame_id != 'world' or path.header.frame_id != 'world' or
                epoch != stamp_ns(path.header.stamp) or len(path.poses) < 2 or
                len(path.poses) > 1000 or msg.status != 'tracking' or
                not all(math.isfinite(v) for v in (msg.target_speed, msg.target_yaw_rate, msg.minimum_separation)) or
                msg.target_speed < 0 or msg.minimum_separation <= 0):
                raise ValueError('Invalid MPC metadata')
            previous = None
            for pose in path.poses:
                p, q = pose.pose.position, pose.pose.orientation
                if pose.header.frame_id not in ('', 'world') or not all(
                        math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)):
                    raise ValueError('Invalid MPC pose')
                if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .1:
                    raise ValueError('Invalid MPC orientation')
                current = stamp_ns(pose.header.stamp)
                if (previous is None and current != epoch) or (previous is not None and current <= previous):
                    raise ValueError('Invalid MPC time grid')
                previous = current
            now = self.get_clock().now().nanoseconds
            if epoch > now + 50_000_000:
                raise ValueError('Future MPC plan')
            if self.plan and epoch <= stamp_ns(self.plan.header.stamp):
                return
            if not all(math.isfinite(v) and v > 0 for v in (msg.max_acceleration,msg.max_yaw_rate)):
                raise ValueError('Missing MPC motion limits')
            poses = []
            for pose in path.poses:
                p,q = pose.pose.position,pose.pose.orientation
                poses.append((p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
            trajectory = TimedTrajectory(
                tuple((stamp_ns(p.header.stamp)-epoch)*1e-9 for p in path.poses),tuple(poses),
                tuple(msg.linear_velocities),tuple(msg.linear_accelerations),tuple(msg.angular_velocities))
            if (any(abs(a)>msg.max_acceleration+.001 for a in trajectory.accelerations) or
                any(abs(w)>msg.max_yaw_rate+.001 for w in trajectory.yaw_rates) or
                any(v < -.011 or v > 2. for v in trajectory.velocities)):
                raise ValueError('MPC motion limits violated')
            self.plan, self.plan_wall, self.trajectory = msg, time.monotonic(), trajectory
        except ValueError as error:
            self.plan = self.trajectory = None
            self.alignment_yaw = None
            self.invalid_reason = str(error)
            self.hold(self.invalid_reason)

    def on_odom(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        w = msg.twist.twist.angular.z
        if (msg.header.frame_id != 'world' or msg.child_frame_id != self.ego_robot+'/base_link' or
            not all(math.isfinite(x) for x in (p.x,p.y,q.x,q.y,q.z,q.w,v.x,v.y,w)) or
            abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .1):
            self.odom = None
            self.hold('invalid_ego_odometry')
            return
        if self.odom and stamp_ns(msg.header.stamp) <= stamp_ns(self.odom.header.stamp):
            return
        self.odom, self.odom_wall = msg, time.monotonic()

    def on_peer(self, msg):
        if (not msg.valid or msg.header.frame_id != 'world' or msg.robot_id != self.peer_robot or
            not all(math.isfinite(x) for x in (msg.x,msg.y,msg.vx,msg.vy))):
            self.peer = None
            self.hold('invalid_peer_state')
            return
        if self.peer and stamp_ns(msg.header.stamp) <= stamp_ns(self.peer.header.stamp):
            return
        self.peer, self.peer_wall = msg, time.monotonic()

    def on_command(self, msg):
        if not all(math.isfinite(v) for v in (msg.linear.x,msg.linear.y,msg.linear.z,
                                            msg.angular.x,msg.angular.y,msg.angular.z)):
            self.raw_command = None
            self.hold('invalid_rpp_command')
            return
        self.raw_command, self.raw_wall = msg, time.monotonic()

    def fresh(self, msg, wall, now, timeout):
        if msg is None:
            return False
        age = (now-stamp_ns(msg.header.stamp))*1e-9
        return -1e-6 <= age <= timeout and time.monotonic()-wall <= timeout

    def request_path(self, now):
        self.sequence += 1
        sequence, epoch = self.sequence, self.epoch
        goal = FollowPath.Goal()
        goal.path = copy.deepcopy(self.plan.path)
        goal.path.poses.clear()
        age = max(0.,(now-stamp_ns(self.plan.header.stamp))*1e-9)
        horizon = self.tracking_horizon
        window = self.trajectory.window(age,horizon,self.path_sample_period)
        # Native RPP suppresses curvature below a ~3.2 cm carrot distance. Keep
        # enough optimized geometry available while it still follows the near
        # (4 cm minimum) lookahead, rather than the distant path endpoint.
        while (horizon < self.maximum_tracking_horizon and
               math.hypot(window[-1].x-window[0].x,window[-1].y-window[0].y) < self.minimum_tracking_length and
               age+horizon < self.trajectory.times[-1]):
            horizon = min(self.maximum_tracking_horizon,horizon+.2)
            window = self.trajectory.window(age,horizon,self.path_sample_period)
        self.tracking_mode = 'tracking'
        future = self.trajectory.sample(min(age+1./self.output_frequency,self.trajectory.times[-1]))
        if future.v <= .005 and abs(future.w) > .02:
            count = rotation_prefix_count([(p.x,p.y,p.yaw) for p in window],future.v,future.w)
            if count:
                window = window[:count]
                self.tracking_mode = 'tracking_mpc_turn'
        if self.alignment_yaw is not None:
            # A current heading from the optimized MPC, expressed through RPP.
            ego = self.execution_state
            window = [Motion(ego.x,ego.y,ego.yaw,0.,0.),
                      Motion(ego.x,ego.y,self.alignment_yaw,0.,0.)]
            self.tracking_mode = 'tracking_mpc_heading'
        # Resample the optimized a/w stages at the execution epoch. RPP receives
        # only the immediate time window, not a 6-second path with reset times.
        goal.path.header.stamp = self.get_clock().now().to_msg()
        for point in window:
            pose = PoseStamped(); pose.header = copy.deepcopy(goal.path.header)
            pose.pose.position.x,pose.pose.position.y = point.x,point.y
            pose.pose.orientation.z = math.sin(point.yaw/2.)
            pose.pose.orientation.w = math.cos(point.yaw/2.)
            goal.path.poses.append(pose)
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'goal_checker'
        goal.progress_checker_id = 'progress_checker'
        self.last_send = time.monotonic()
        self.last_goal_stamp = stamp_ns(self.plan.header.stamp)
        self.goal_pending = True
        requested_mode = self.tracking_mode
        future = self.client.send_goal_async(goal)

        def response(f):
            self.goal_pending = False
            try:
                handle = f.result()
            except Exception as error:
                self.hold(f'action_error: {error}')
                return
            if not handle.accepted:
                self.hold('follow_path_rejected')
                return
            if epoch != self.epoch or not self.enabled:
                handle.cancel_goal_async()
                return
            if self.goal_handle is None or requested_mode != self.active_tracking_mode:
                self.active_since = time.monotonic()
                self.raw_command = None
            self.active_tracking_mode = requested_mode
            self.goal_handle = handle
            self.active_sequence = sequence
            handle.get_result_async().add_done_callback(lambda result: self.on_result(result, sequence, epoch))

        future.add_done_callback(response)

    def on_result(self, future, sequence, epoch):
        if epoch != self.epoch or sequence != self.active_sequence or sequence < self.sequence:
            return
        self.goal_handle = None
        self.raw_command = None
        try:
            result = future.result()
            reason = 'path_segment_complete' if result.status == GoalStatus.STATUS_SUCCEEDED else 'follow_path_stopped'
        except Exception:
            reason = 'follow_path_result_error'
        self.hold(reason, cancel=False)

    def tick(self):
        self.diagnostics = {}
        now = self.get_clock().now().nanoseconds
        wall = time.monotonic()
        if self.last_clock is not None and now < self.last_clock:
            self.plan = self.odom = self.peer = self.trajectory = None
            self.alignment_yaw = None
            self.clear_goal()
        control_dt = 1./self.output_frequency if self.last_clock is None else min(
            2./self.output_frequency,max(0.,(now-self.last_clock)*1e-9))
        self.last_clock = now
        if not self.enabled:
            self.hold('disabled'); return
        if not self.fresh(self.odom, self.odom_wall, now, self.state_timeout) or not self.fresh(
                self.peer, self.peer_wall, now, self.state_timeout):
            self.hold('waiting_for_fresh_gt'); return
        if not self.fresh(self.plan, self.plan_wall, now, self.plan_timeout):
            self.hold(self.invalid_reason if self.plan is None else 'expired_mpc_plan'); return
        if control_dt <= 0.:
            # A repeated clock sample is not a stop request. Freshness checks
            # above still stop on wall-time expiry if physics actually pauses.
            self.report('waiting_for_clock'); return
        p,q,v = self.odom.pose.pose.position,self.odom.pose.pose.orientation,self.odom.twist.twist.linear
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        acceleration = min(self.max_acceleration,self.plan.max_acceleration)
        ego = Motion(p.x,p.y,yaw,v.x,self.odom.twist.twist.angular.z)
        if abs(v.y) > .05:
            self.hold('unicycle_tracking_error'); return
        # Propagate both measured states to this SAME execution epoch.
        ego_age = max(0.,(now-stamp_ns(self.odom.header.stamp))*1e-9)
        if ego_age:
            ego=step(ego,ego.v,ego.w,ego_age,acceleration,self.max_yaw_acceleration)
        self.execution_state = ego
        peer_age = max(0.,(now-stamp_ns(self.peer.header.stamp))*1e-9)
        peer = (self.peer.x+self.peer.vx*peer_age,self.peer.y+self.peer.vy*peer_age,
                self.peer.vx,self.peer.vy)
        age = max(0.,(now-stamp_ns(self.plan.header.stamp))*1e-9)
        try:
            reference = self.trajectory.sample(age+control_dt)
        except (ValueError,AttributeError):
            self.hold('expired_mpc_motion_profile'); return
        limit = min(self.max_speed,max(0.,reference.v))
        if limit <= .001 and abs(reference.w) <= .02:
            self.hold('mpc_stop'); return
        if self.alignment_yaw is not None:
            target = alignment_heading(self.trajectory,age,ego.yaw,reference.w,self.alignment_horizon)
            if target is None or reference.v > .01 or abs(reference.w) <= .02:
                self.alignment_yaw = None
                self.last_send = -float('inf')
                self.last_goal_stamp = -1
            else:
                self.alignment_yaw = target
        if not self.client.server_is_ready():
            self.hold('waiting_for_follow_path_server'); return
        # Nav2 SpeedLimit=0 means restore default speed, so zero is handled by
        # cancel/command gating above, never by publishing a zero speed limit.
        speed_limit = SpeedLimit()
        speed_limit.header.stamp = self.get_clock().now().to_msg()
        speed_limit.speed_limit = max(.001, limit)
        speed_limit.percentage = False
        self.speed_pub.publish(speed_limit)
        if not self.goal_pending and wall-self.last_send >= 1./self.goal_frequency and (
                self.goal_handle is None or stamp_ns(self.plan.header.stamp) > self.last_goal_stamp):
            self.request_path(now)
        if self.goal_handle is None:
            self.hold('waiting_for_goal_acceptance', cancel=False); return
        if self.raw_command is None or wall-self.raw_wall > self.command_timeout or self.raw_wall < self.active_since:
            self.hold('waiting_for_fresh_rpp_command', cancel=wall-self.active_since > .5); return
        raw = self.raw_command
        target_v = min(limit,max(0.,raw.linear.x),max(0.,ego.v)+acceleration*control_dt)
        yaw_limit = min(self.max_yaw_rate,self.plan.max_yaw_rate)
        # Continue the last issued target rather than starting at measured zero
        # each cycle. Limit command lead and still predict from measured motion.
        yaw_request = max(-yaw_limit,min(yaw_limit,raw.angular.z))
        if raw.linear.x > 1e-4 and reference.v > .005:
            yaw_request *= min(1.,target_v/raw.linear.x)
        target_w = approach(self.last_output.angular.z,yaw_request,
                            self.max_yaw_acceleration*control_dt)
        yaw_lead = self.max_yaw_acceleration*self.reaction_time
        target_w = max(ego.w-yaw_lead,min(ego.w+yaw_lead,target_w))
        target_w = max(-yaw_limit,min(yaw_limit,target_w))
        nominal = max(self.minimum_separation,self.plan.minimum_separation)
        gap = math.hypot(ego.x-peer[0],ego.y-peer[1])-nominal
        # Reserve most of the remaining gap close to the boundary: both robots
        # can react between CV measurements. Letting each spend half can use
        # the entire gap simultaneously, before the next GT/command cycle.
        separation = nominal+min(self.execution_buffer,max(0.,gap*self.execution_gap_fraction))
        self.diagnostics = dict(plan_age=age,ego_age=ego_age,peer_age=peer_age,
            profile_speed=reference.v,profile_yaw_rate=reference.w,
            position_error=math.hypot(ego.x-self.trajectory.sample(age).x,ego.y-self.trajectory.sample(age).y),
            measured_speed=ego.v,measured_yaw_rate=ego.w,
            rpp_speed=raw.linear.x,rpp_yaw_rate=raw.angular.z,
            nominal_separation=nominal,checked_separation=separation,
            separation=math.hypot(ego.x-peer[0],ego.y-peer[1]))
        selected = filter_command(ego,target_v,target_w,peer,separation,
                                  self.reaction_time,acceleration,self.max_yaw_acceleration,
                                  self.braking_deceleration,
                                  mpc_yaw_rate=reference.w,rpp_yaw_rate=raw.angular.z)
        if selected is None:
            self.proximity_release.stop()
        blocked_translation = selected is None or selected[0] <= .001
        wrong_initial_turn = (abs(reference.w) > .02 and
            (raw.angular.z*reference.w <= 0. or abs(raw.angular.z) < .25*abs(reference.w)))
        if self.alignment_yaw is None and blocked_translation and wrong_initial_turn:
            target_yaw = alignment_heading(self.trajectory,age,ego.yaw,reference.w,self.alignment_horizon)
            if target_yaw is not None:
                self.alignment_yaw = target_yaw
                self.last_send = -float('inf');self.last_goal_stamp = -1
                self.hold('waiting_for_mpc_heading',cancel=False);return
        if selected is None:
            self.diagnostics['candidate_rejected'] = True
            # A temporary command veto must not cancel a valid FollowPath goal.
            # Keep accepting fresh plans/RPP commands, but gate the Swerve output.
            self.hold('proximity_stop',cancel=False); return
        vx,w,clearance,scale = selected
        rotation_only = scale == 0. and abs(target_v) > 1e-4 and abs(w) > 1e-4
        self.diagnostics.update(command_speed=vx,command_yaw_rate=w,predicted_clearance=clearance,
            speed_scale=scale,rotation_only=rotation_only,candidate_speed=vx,candidate_yaw_rate=w)
        if not self.proximity_release.allow(now):
            # Do not call hold(): it interrupts the consecutive-safe window.
            self.diagnostics.update(command_speed=0.,command_yaw_rate=0.)
            self.emit(Twist())
            self.report('proximity_recovery_wait'); return
        command = Twist(); command.linear.x,command.angular.z = vx,w
        self.emit(command)
        self.report('tracking_safe_rotation' if rotation_only else
                    'tracking_speed_reduced' if scale < 1. else self.tracking_mode)

    def close(self):
        self.enabled = False
        if rclpy.ok():
            self.clear_goal()
            self.cmd_pub.publish(Twist())
        self.destroy_node()


def main():
    rclpy.init()
    node = MpcRppAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        if rclpy.ok():
            rclpy.shutdown()
