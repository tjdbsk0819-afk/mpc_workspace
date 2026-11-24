"""Real acados goal slowdown, final heading, latch and clock-reset regression.

Synthetic GT only; no physical command publisher. Run in isolated domain 233.
"""
import math
import os
import signal
import subprocess
import tempfile
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '233':
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=233')
    import rclpy
    from antbot_interfaces.msg import MpcPlan, PeerState
    from builtin_interfaces.msg import Time
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('goal_arrival_check')
    clock = node.create_publisher(Clock, '/clock', 10)
    odom = node.create_publisher(Odometry, '/robot_a/odom', 10)
    peer = node.create_publisher(PeerState, '/robot_a/peer_state', 10)
    latest = {}
    subs = [node.create_subscription(MpcPlan, '/robot_a/mpc_plan', lambda p: latest.update(plan=p), 10),
            node.create_subscription(String, '/robot_a/mpc_status', lambda p: latest.update(status=p.data), 10)]
    begin = time.monotonic()
    state = [0., 0., 0., 1.006]
    process = None
    log = tempfile.TemporaryFile(mode='w+')

    def run(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            assert process.poll() is None, 'Planner exited'
            ns = round((100. + time.monotonic() - begin)*1e9)
            t = Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)
            clock.publish(Clock(clock=t))
            o = Odometry()
            o.header.frame_id = 'world'; o.header.stamp = t; o.child_frame_id = 'robot_a/base_link'
            o.pose.pose.position.x, o.pose.pose.position.y = state[:2]
            o.pose.pose.orientation.z = math.sin(state[2]/2.)
            o.pose.pose.orientation.w = math.cos(state[2]/2.)
            o.twist.twist.linear.x = state[3]; odom.publish(o)
            p = PeerState(); p.header = o.header; p.robot_id = 'robot_b'
            p.x = -8.; p.y = 5.; p.valid = True; p.status = 'ok'; peer.publish(p)
            rclpy.spin_once(node, timeout_sec=.005); time.sleep(.01)

    try:
        process = subprocess.Popen(['ros2', 'run', 'mpc_planner_antbot', 'antbot_planner', '--ros-args',
            '-r', '__ns:=/robot_a', '-p', 'use_sim_time:=true', '-p', 'recovery_enabled:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        run(3.)
        plan = latest['plan']
        assert plan.valid, latest
        assert abs(plan.linear_velocities[0]-1.006)<.001
        assert plan.linear_velocities[1]<=1.001 and plan.linear_accelerations[0]<0., plan
        print('PASS: small measured overspeed is preserved as x0 while future speed is bounded and brakes', flush=True)
        state[:] = [2.4, 0., 0., .2]
        run(1.)
        plan = latest['plan']
        assert plan.valid, latest
        last = plan.path.poses[-1].pose.position
        assert math.hypot(last.x-3., last.y)<.2, (last, plan.linear_velocities)
        assert abs(plan.linear_velocities[-1])<.1, plan.linear_velocities
        for pose, speed in zip(plan.path.poses[1:],plan.linear_velocities[1:]):
            distance=math.hypot(pose.pose.position.x-3.,pose.pose.position.y)
            assert speed*speed+.175*speed<=.7*max(0.,distance-.08)+.002
        print('PASS: real OCP approaches the original goal and lowers terminal speed', flush=True)
        state[:] = [3., 0., .4, 0.]
        run(1.)
        plan = latest['plan']
        assert plan.valid and latest['status']=='aligning_goal', latest
        assert plan.angular_velocities[0]<-.005, plan.angular_velocities
        assert max(plan.linear_velocities[1:])<.001, plan.linear_velocities
        print('PASS: final yaw is optimized with stationary future translation', flush=True)
        state[2] = .01
        run(.8)
        assert latest['status']=='goal_reached' and not latest['plan'].valid, latest
        state[:] = [3.5, .2, .4, 0.]
        run(.8)
        assert latest['status']=='goal_reached' and not latest['plan'].valid, latest
        print('PASS: final pose dwell completes and drift cannot restart the mission', flush=True)
        begin += 10.
        state[:] = [2.4, 0., 0., .2]
        run(1.2)
        assert latest['plan'].valid and latest['status']=='tracking', latest
        assert not node.get_publishers_info_by_topic('/robot_a/cmd_vel')
        print('PASS: clock rewind resets completed goal and fresh MPC planning resumes; no cmd_vel published', flush=True)
    except Exception:
        log.seek(0); print(log.read()[-12000:], flush=True); raise
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try: process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        node.destroy_node(); rclpy.shutdown(); log.close()


if __name__ == '__main__':
    main()
