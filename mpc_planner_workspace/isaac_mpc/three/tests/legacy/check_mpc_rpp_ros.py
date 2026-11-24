"""Real ROS action/command tests in domain 230, isolated from Isaac.

A fake FollowPath server deliberately keeps publishing commands after cancel.
The adapter must still block them when the original MPC plan/GT is invalid.
"""
import copy
import json
import math
import os
import threading
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '230':
        raise RuntimeError('Use reserved ROS_DOMAIN_ID=230')
    import rclpy
    from rclpy.action import ActionServer, GoalResponse, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from antbot_interfaces.msg import MpcPlan, PeerState
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import FollowPath
    from nav2_msgs.msg import SpeedLimit
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from antbot_mpc_ros.mpc_rpp_adapter import MpcRppAdapter

    rclpy.init()

    class FakeRpp(Node):
        def __init__(self):
            super().__init__('fake_rpp', namespace='robot_a')
            self.current = None
            self.requests = self.cancellations = 0
            self.delay = False
            self.spin_rate = 0.
            self.force_translation = False
            self.accepted_paths = []
            self.pub = self.create_publisher(Twist, 'rpp_cmd_vel', 1)
            self.timer = self.create_timer(.02, self.emit)
            self.server = ActionServer(self, FollowPath, 'follow_path', self.execute,
                goal_callback=self.goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
                handle_accepted_callback=self.accept, callback_group=ReentrantCallbackGroup())

        def emit(self):
            cmd = Twist(); cmd.linear.x = .3 if self.spin_rate == 0. or self.force_translation else 0.
            cmd.angular.z = self.spin_rate
            self.pub.publish(cmd)

        def goal(self, request):
            self.requests += 1
            if self.delay:
                self.delay = False
                time.sleep(.35)
            return GoalResponse.ACCEPT

        def accept(self, handle):
            self.current = handle
            self.accepted_paths.append(copy.deepcopy(handle.request.path))
            handle.execute()

        def execute(self, handle):
            while rclpy.ok() and handle.is_active:
                if handle.is_cancel_requested:
                    handle.canceled(); self.cancellations += 1; break
                if self.current is not handle:
                    handle.abort(); break
                time.sleep(.01)
            return FollowPath.Result()

    driver = Node('adapter_diagnostic')
    fake = FakeRpp()
    adapter = MpcRppAdapter(namespace='robot_a', parameter_overrides=[Parameter('use_sim_time',value=True)])
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (driver, fake, adapter):executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True);thread.start()
    clock = driver.create_publisher(Clock,'/clock',10)
    plan_pub = driver.create_publisher(MpcPlan,'/robot_a/mpc_plan',1)
    odom_pub = driver.create_publisher(Odometry,'/robot_a/odom',10)
    peer_pub = driver.create_publisher(PeerState,'/robot_a/peers/robot_b/state',10)
    second_peer_pub=driver.create_publisher(PeerState,'/robot_a/peers/robot_c/state',10)
    commands, statuses, limits, all_commands, diagnostics = [], [], [], [], []
    def receive_command(m):
        commands.append((time.monotonic(),m.linear.x))
        all_commands.append((time.monotonic(),m.linear.x,m.angular.z))
    subs = [driver.create_subscription(Twist,'/robot_a/cmd_vel',receive_command,10),
            driver.create_subscription(String,'/robot_a/control_status',lambda m:statuses.append(m.data),10),
            driver.create_subscription(String,'/robot_a/control_diagnostics',lambda m:diagnostics.append(json.loads(m.data)),10),
            driver.create_subscription(SpeedLimit,'/robot_a/speed_limit',lambda m:limits.append(m.speed_limit),10)]
    enable = driver.create_client(SetBool,'/robot_a/mpc_rpp_adapter/set_enabled')
    begin = time.monotonic()

    def timestamp(seconds):
        ns=round(seconds*1e9);return Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)

    def feed(mode='normal'):
        seconds=100.+.5*(time.monotonic()-begin)
        turning=mode in ('turn','turn_positive')
        blocked=mode in ('blocked_translation','blocked_no_plan')
        turn_rate=.3 if mode=='turn_positive' else -.3
        stamp=timestamp(seconds);clock.publish(Clock(clock=stamp))
        if mode != 'no_gt':
            odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='world'
            odom.child_frame_id='robot_a/base_link';odom.pose.pose.orientation.w=1.
            odom.twist.twist.linear.x=.12 if mode not in ('zero','invalid') and not turning and not blocked else 0.
            odom_pub.publish(odom)
            peer=PeerState();peer.header=odom.header;peer.robot_id='robot_b'
            peer.x=1.1501 if blocked else (.9 if mode in ('near','near_no_plan') else 5.)
            peer.valid=True;peer.status='ok';peer_pub.publish(peer)
            if mode!='no_second':
                second=copy.deepcopy(peer);second.robot_id='robot_c'
                second.x=.9 if mode=='second_near' else 0.;second.y=0. if mode=='second_near' else 5.
                second_peer_pub.publish(second)
        if mode not in ('no_plan','near_no_plan','blocked_no_plan'):
            plan=MpcPlan();plan.header.stamp=stamp;plan.header.frame_id='world';plan.path.header=plan.header
            plan.valid=mode!='invalid';plan.status='tracking' if plan.valid else 'solver_failed'
            plan.target_speed=0. if mode=='zero' or turning else .12;plan.minimum_separation=1.15
            plan.target_yaw_rate=turn_rate if turning else 0.
            if blocked:
                plan.target_speed=.1;plan.target_yaw_rate=.2
            plan.max_acceleration=.5;plan.max_yaw_rate=.8
            if plan.valid:
                for k in range(15):
                    pose=PoseStamped();pose.header.frame_id='world';pose.header.stamp=timestamp(seconds+k*.2)
                    pose.pose.position.x=k*.024 if mode!='zero' and not turning else 0.
                    heading=turn_rate*k*.2 if turning else 0.
                    if blocked:
                        heading=.2*k*.2
                        pose.pose.position.x=.5*math.sin(heading)
                        pose.pose.position.y=.5*(1.-math.cos(heading))
                    pose.pose.orientation.w=math.cos(heading/2.);pose.pose.orientation.z=math.sin(heading/2.)
                    plan.path.poses.append(pose)
                    plan.linear_velocities.append(.1 if blocked else (.12 if mode!='zero' and not turning else 0.))
                    plan.linear_accelerations.append(0.);plan.angular_velocities.append(.2 if blocked else (turn_rate if turning else 0.))
            plan_pub.publish(plan)

    def run(seconds, mode='normal', feeding=True):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if feeding:feed(mode)
            time.sleep(.025)

    def stopped():
        recent=[(v,w) for t,v,w in all_commands if time.monotonic()-t<.2]
        assert recent and all(max(abs(v),abs(w))<1e-9 for v,w in recent), recent

    try:
        run(1.5)
        assert any(v>.05 for _,v in commands), statuses[-20:]
        assert all(v<=.120001 for _,v in commands)
        assert limits and all(x>0 for x in limits)
        assert fake.requests>=2 and fake.accepted_paths
        p=fake.accepted_paths[-1]
        assert all(s.header.stamp==p.header.stamp for s in p.poses)
        print('PASS: FollowPath updates, spatial timestamps, MPC speed cap and command forwarding',flush=True)
        fake.spin_rate=-.15;fake.force_translation=True
        run(.7,'turn')
        assert min(w for t,v,w in all_commands if time.monotonic()-t<.3)<-.05, 'Zero MPC translation erased its planned rotation'
        fake.force_translation=False;fake.spin_rate=0.;run(.5)
        print('PASS: a genuine MPC turn retains RPP rotation even when the translation cap is zero',flush=True)
        previous=fake.cancellations
        fake.spin_rate=-.025
        run(2.6,'turn')
        assert adapter.alignment_yaw is not None
        assert min(w for t,v,w in all_commands if time.monotonic()-t<.4)<-.024
        assert fake.cancellations==previous,'Heading refresh canceled the existing FollowPath'
        fake.spin_rate=.025;run(.7,'turn_positive')
        assert adapter.alignment_yaw>.05,'Latest opposite MPC heading was not adopted'
        assert max(w for t,v,w in all_commands if time.monotonic()-t<.3)>.024
        assert fake.cancellations==previous
        fake.spin_rate=0.;run(.7)
        assert adapter.alignment_yaw is None
        assert fake.cancellations==previous
        print('PASS: 0.5x simulation small yaw ramps accumulate; latest MPC headings refresh/reverse/exit without cancellation',flush=True)
        previous=fake.cancellations;diag_start=len(diagnostics)
        fake.spin_rate=.2;fake.force_translation=True
        run(1.,'blocked_translation')
        recent=[(v,w) for t,v,w in all_commands if time.monotonic()-t<.3]
        assert recent and all(abs(v)<1e-9 for v,w in recent) and any(w>0. for v,w in recent), recent
        assert 'tracking_safe_rotation' in statuses[-20:],statuses[-20:]
        rotation=[d for d in diagnostics[diag_start:] if d.get('rotation_only') and d.get('command_yaw_rate',0.)>0.]
        assert rotation and all(d['predicted_clearance']>=d['checked_separation'] for d in rotation),rotation
        assert fake.cancellations==previous
        print('PASS: blocked translation uses aligned, clearance-checked rotation without canceling FollowPath',flush=True)
        run(.8,'blocked_no_plan');stopped()
        assert fake.cancellations>previous
        print('PASS: expired MPC also blocks safe-rotation fallback despite fresh GT and RPP',flush=True)
        fake.spin_rate=0.;fake.force_translation=False;run(.7)
        run(.4,'second_near');stopped()
        assert statuses[-1]=='proximity_stop'
        assert len(adapter.diagnostics['peer_checks'])==2
        run(.7);run(.7,'no_second');stopped()
        assert statuses[-1]=='waiting_for_fresh_gt'
        run(.7)
        print('PASS: second peer vetoes motion and missing second GT expires the common command',flush=True)
        previous=fake.cancellations
        run(.4,'near');stopped()
        assert statuses[-1]=='proximity_stop', statuses[-10:]
        assert fake.cancellations==previous
        assert adapter.goal_handle is not None or adapter.goal_pending
        assert time.monotonic()-adapter.raw_wall<.15
        print('PASS: proximity stop outputs zero while keeping FollowPath and fresh RPP commands',flush=True)
        pulse_start=time.monotonic()
        for _ in range(3):
            run(.075)
            run(.15,'near')
        assert all(abs(v)<1e-9 for t,v in commands if t>=pulse_start)
        assert fake.cancellations==previous
        run(.075);stopped()
        assert 'proximity_recovery_wait' in statuses[-20:], statuses[-20:]
        run(.4)
        assert any(v>.05 for t,v in commands if time.monotonic()-t<.2)
        assert fake.cancellations==previous
        print('PASS: brief safe pulses cannot restart; sustained safety releases without cancel/restart',flush=True)
        run(.35,'near');stopped()
        previous=fake.cancellations
        run(.6,'invalid');stopped();assert fake.cancellations>previous
        print('PASS: failed MPC during proximity hold still cancels FollowPath and blocks RPP commands',flush=True)
        run(.5,'zero');stopped()
        print('PASS: zero MPC speed stops without Nav2 zero-limit reset semantics',flush=True)
        previous=fake.cancellations;requests=fake.requests;fake.delay=True
        for _ in range(30):
            run(.025)
            if fake.requests>requests:break
        assert fake.requests>requests
        run(.8,'invalid');stopped();assert fake.cancellations>previous
        print('PASS: late accepted goal is canceled after plan invalidation',flush=True)
        run(.7)
        run(.35,'near');stopped()
        previous=fake.cancellations
        run(.7,'near_no_plan');stopped()
        assert fake.cancellations>previous
        print('PASS: expired MPC cancels even after proximity hold despite fresh GT/RPP',flush=True)
        run(.7)
        run(.7,'no_gt');stopped()
        print('PASS: expired GT stops despite fresh MPC and RPP commands',flush=True)
        run(.7)
        checkpoint=time.monotonic();run(.1,feeding=False)
        assert all(max(abs(v),abs(w))>0 for t,v,w in all_commands if t>=checkpoint), 'Duplicate clock emitted a false stop'
        run(.7,feeding=False);stopped()
        print('PASS: paused clock/GT stops on wall time',flush=True)
        begin=time.monotonic();run(.8)
        assert any(v>.05 for t,v in commands if time.monotonic()-t<.3)
        assert enable.wait_for_service(timeout_sec=1)
        request=SetBool.Request();request.data=False
        future=enable.call_async(request);run(.5);assert future.done() and future.result().success;stopped()
        print('PASS: backward clock recovery and explicit disable stop',flush=True)
    finally:
        adapter.enabled=False
        adapter.clear_goal();adapter.cmd_pub.publish(Twist())
        time.sleep(.1)
        executor.shutdown(timeout_sec=3)
        for node in (driver,fake,adapter):node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2)


if __name__=='__main__':main()
