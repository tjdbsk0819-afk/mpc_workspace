"""Actual identical C++ MPC banks on stationary GT, isolated ROS domain 229.

No Isaac and no motion commands. Exercise stall -> seed reset -> optimized paths,
input expiry, clearance rejection and recovery-disabled baseline.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '229':
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=229')
    import rclpy
    from antbot_interfaces.msg import MpcPlan, PeerState
    from builtin_interfaces.msg import Time
    from nav_msgs.msg import Odometry, Path as RosPath
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('mpc_replanning_check')
    robots = ('robot_a', 'robot_b')
    clock = node.create_publisher(Clock, '/clock', 10)
    pubs = {r: {s: node.create_publisher(t, f'/{r}/{s}', 10)
                for s, t in [('odom', Odometry), ('peer_state', PeerState)]} for r in robots}
    latest, samples, subscriptions = {}, [], []
    paths_by_stamp = {}
    def record(key, msg):
        latest[key] = msg
        if isinstance(msg, RosPath):
            paths_by_stamp[(*key, msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
    for r in robots:
        for suffix, kind in [('mpc_plan', MpcPlan), ('recovery_status', String),
                             ('mpc_reference_path', RosPath)]+[(f'mpc_candidate_{i}', RosPath) for i in range(5)]:
            subscriptions.append(node.create_subscription(kind, f'/{r}/{suffix}',
                lambda m, key=(r,suffix): record(key,m), 10))
    begin = time.monotonic()
    processes, logs = [], []
    half_distance = .7

    def stamp(seconds):
        n = round(seconds*1e9)
        return Time(sec=n//1_000_000_000, nanosec=n%1_000_000_000)

    def run(duration, feed=True):
        end = time.monotonic()+duration
        while time.monotonic()<end:
            assert all(p.poll() is None for p in processes), 'MPC process exited'
            now = 100.+time.monotonic()-begin
            if feed:
                t = stamp(now)
                clock.publish(Clock(clock=t))
                for i, r in enumerate(robots):
                    ego = Odometry(); ego.header.frame_id='world'; ego.header.stamp=t
                    ego.child_frame_id=r+'/base_link'; ego.pose.pose.position.x=(-1 if i==0 else 1)*half_distance
                    ego.pose.pose.orientation.z=0. if i==0 else 1.; ego.pose.pose.orientation.w=1. if i==0 else 0.
                    pubs[r]['odom'].publish(ego)
                    peer = PeerState(); peer.header=ego.header; peer.robot_id=robots[1-i]
                    peer.x=-ego.pose.pose.position.x; peer.valid=True; peer.status='ok'
                    pubs[r]['peer_state'].publish(peer)
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=.0001)
            for r in robots:
                if (r,'recovery_status') in latest:
                    samples.append((r,json.loads(latest[r,'recovery_status'].data)))
            time.sleep(.01)

    def start(enabled=True):
        for i,r in enumerate(robots):
            log=tempfile.TemporaryFile(mode='w+');logs.append(log)
            processes.append(subprocess.Popen([
                '/workspace/planner_build/install/mpc_planner_antbot/lib/mpc_planner_antbot/antbot_planner',
                '--ros-args','-r',f'__ns:=/{r}','-p','use_sim_time:=true',
                '-p',f'ego_robot:={r}','-p',f'peer_robot:={robots[1-i]}',
                '-p',f'goal:={[3. if i==0 else -3.,0.]}',
                '-p',f'recovery_enabled:={str(enabled).lower()}',
            ],stdout=log,stderr=subprocess.STDOUT,start_new_session=True))

    def stop():
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        processes.clear()

    result={'verified':False}
    try:
        start();run(7.)
        for r in robots:
            phases=[s for name,s in samples if name==r]
            assert any(s['episode']>=1 and s['phase']=='tracking_candidate' for s in phases), phases[-4:]
            diagnostic=json.loads(latest[r,'recovery_status'].data)
            assert len(diagnostic['costs'])==5 and sum(c is not None for c in diagnostic['costs'])>=2,diagnostic
            plan=latest[r,'mpc_plan'];assert plan.valid and len(plan.path.poses)==30,plan.status
            matching = [paths_by_stamp.get((r,f'mpc_candidate_{i}',plan.header.stamp.sec,plan.header.stamp.nanosec)) for i in range(5)]
            assert any(plan.path==p for p in matching), 'Selected path must be an optimized candidate with the same stamp' 
            for p in plan.path.poses:
                assert math.hypot(p.pose.position.x-(-.7 if r=='robot_b' else .7),p.pose.position.y)>=1.149
            assert not node.get_publishers_info_by_topic(f'/{r}/cmd_vel')
            assert not node.get_publishers_info_by_topic(f'/{r}/recovery_state')
        print('PASS: both identical MPCs detect no progress and optimize feasible candidate banks; no bypass commands or handshake',flush=True)
        result['stationary_recovery']={r:json.loads(latest[r,'recovery_status'].data) for r in robots}
        result['max_lateral_candidate_m']={r:max(abs(p.pose.position.y) for i in range(5)
                            for p in latest[r,f'mpc_candidate_{i}'].poses) for r in robots}
        assert all(v>.1 for v in result['max_lateral_candidate_m'].values()), result
        print('PASS: alternative initial guesses produce optimized paths on both sides of the reference',flush=True)
        run(.8,feed=False)
        assert all(not latest[r,'mpc_plan'].valid for r in robots)
        print('PASS: paused clock/GT invalidates all selected paths',flush=True)
        half_distance=.55;run(.8)
        assert all(latest[r,'mpc_plan'].status=='insufficient_current_clearance' for r in robots)
        assert all(not latest[r,f'mpc_candidate_{i}'].poses for r in robots for i in range(5))
        print('PASS: inside the original safety margin every robot stops; recovery never relaxes it',flush=True)
        stop();latest.clear();samples.clear();half_distance=.7
        start(enabled=False);run(5.)
        assert all(json.loads(latest[r,'recovery_status'].data)['episode']==0 for r in robots)
        print('PASS: recovery_enabled=false preserves the single-start baseline',flush=True)
        result['verified']=True
    except Exception:
        for log in logs:
            log.seek(0);print(log.read()[-7000:],flush=True)
        raise
    finally:
        stop()
        output=Path('/workspace/results/mpc_replanning_ros.json')
        output.write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
        for log in logs:log.close()


if __name__=='__main__':main()
