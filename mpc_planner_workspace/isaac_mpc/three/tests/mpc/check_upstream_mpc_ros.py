"""Run real original C++ planners on synthetic GT in reserved ROS domain 232.

No motion commands are published. Requires planner_build and the Zenoh router.
Run directly in the MPC image, not in generic unittest discovery.
"""
import math
import os
import signal
import subprocess
import tempfile
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '232':
        raise RuntimeError('Run this diagnostic with ROS_DOMAIN_ID=232 (isolated from Isaac).')
    import rclpy
    from antbot_interfaces.msg import PeerState
    from nav_msgs.msg import Odometry, Path
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('upstream_mpc_diagnostic')
    clock = node.create_publisher(Clock, '/clock', 10)
    names = ('robot_a', 'robot_b')
    odom = {r: node.create_publisher(Odometry, f'/{r}/odom', 10) for r in names}
    peer = {r: node.create_publisher(PeerState, f'/{r}/peer_state', 10) for r in names}
    received = {r: {} for r in names}
    subscriptions = []
    for robot in names:
        for suffix, msg_type in [('mpc_path', Path), ('mpc_peer_prediction', Path), ('mpc_reference_path', Path), ('mpc_status', String)]:
            subscriptions.append(node.create_subscription(msg_type, f'/{robot}/{suffix}',
                lambda msg, r=robot, s=suffix: received[r].update({s: msg}), 10))
    processes, logs = [], []
    begin = time.monotonic()
    ego_override = None

    def stamp_at(seconds):
        from builtin_interfaces.msg import Time
        ns = round(seconds * 1e9)
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def feed(invalid_a=False):
        elapsed = time.monotonic() - begin
        stamp = stamp_at(100. + elapsed)
        clock.publish(Clock(clock=stamp))
        positions = {'robot_a': (-3., 0.), 'robot_b': (3.-.05*elapsed, 0.)}
        if ego_override is not None:positions['robot_a'] = ego_override
        for r in names:
            msg = Odometry()
            msg.header.frame_id = 'world'; msg.header.stamp = stamp
            msg.child_frame_id = r + '/base_link'
            msg.pose.pose.position.x, msg.pose.pose.position.y = positions[r]
            msg.pose.pose.orientation.z = 1. if r == 'robot_b' else 0.
            msg.pose.pose.orientation.w = 0. if r == 'robot_b' else 1.
            msg.twist.twist.linear.x = .05 if r == 'robot_b' else 0.
            odom[r].publish(msg)
            target = 'robot_b' if r == 'robot_a' else 'robot_a'
            state = PeerState()
            state.header = msg.header; state.robot_id = target
            state.x, state.y = positions[target]
            state.vx = -.05 if target == 'robot_b' else 0.
            state.vy = 0.; state.valid = not (invalid_a and r == 'robot_a'); state.status = 'ok'
            peer[r].publish(state)

    def spin_for(duration, feed_inputs=True, invalid_a=False):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if any(p.poll() is not None for p in processes):
                raise AssertionError('Planner exited unexpectedly')
            if feed_inputs:
                feed(invalid_a)
            rclpy.spin_once(node, timeout_sec=.01)
            time.sleep(.01)

    try:
        for r in names:
            other = 'robot_b' if r == 'robot_a' else 'robot_a'
            goal = '[3.0, 0.0]' if r == 'robot_a' else '[-3.0, 0.0]'
            log = tempfile.TemporaryFile(mode='w+')
            logs.append(log)
            processes.append(subprocess.Popen([
                'ros2', 'run', 'mpc_planner_antbot', 'antbot_planner', '--ros-args',
                '-r', f'__ns:=/{r}', '-p', 'use_sim_time:=true', '-p', f'ego_robot:={r}',
                '-p', f'peer_robot:={other}', '-p', f'goal:={goal}',
            ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
        for _ in range(80):
            spin_for(.1)
            if all(received[r].get('mpc_path') and len(received[r]['mpc_path'].poses) == 30 for r in names):
                break
        for r in names:
            path = received[r].get('mpc_path')
            assert path and len(path.poses) == 30, f'{r}: no successful 30-point path, status={received[r].get("mpc_status")}'
            assert path.header.frame_id == 'world'
            sign = 1 if r == 'robot_a' else -1
            assert sign * (path.poses[-1].pose.position.x - path.poses[0].pose.position.x) > .05
            assert all(math.isfinite(p.pose.position.x) and math.isfinite(p.pose.position.y) for p in path.poses)
            pred = received[r]['mpc_peer_prediction']
            assert len(pred.poses) == 30
            stamp = lambda t: t.sec + t.nanosec * 1e-9
            assert abs(stamp(pred.poses[0].header.stamp)-stamp(pred.header.stamp)-.2) < 1e-6
            if r == 'robot_a':
                for p in pred.poses:
                    expected = 3. - .05*(stamp(p.header.stamp)-100.)
                    assert abs(p.pose.position.x-expected) < 1e-5, 'CV timestamps misaligned with solver stages'
            assert not node.get_publishers_info_by_topic(f'/{r}/cmd_vel'), 'Preview must not drive robots'
        print('PASS: two original C++ planners solve and publish separate finite paths', flush=True)
        print('PASS: original C++ CV is aligned to solver stage 1 at now+dt', flush=True)
        spin_for(.7, invalid_a=True)
        assert not received['robot_a']['mpc_path'].poses
        assert received['robot_b']['mpc_path'].poses
        print('PASS: invalid peer input clears only the affected robot path', flush=True)
        spin_for(.9, feed_inputs=False)
        assert all(not received[r]['mpc_path'].poses for r in names)
        print('PASS: paused clock/GT expires both paths; no cmd_vel publishers', flush=True)
        begin = time.monotonic()  # Start a new simulated episode at t=100.
        for _ in range(50):
            spin_for(.1)
            if all(received[r]['mpc_path'].poses for r in names):
                break
        assert all(len(received[r]['mpc_path'].poses) == 30 for r in names)
        print('PASS: backward clock reset discards old episode and resumes with new GT', flush=True)
        from std_srvs.srv import Trigger
        reset = node.create_client(Trigger, '/robot_a/mpc_planner/replan_from_current')
        assert reset.wait_for_service(timeout_sec=2)
        ego_override = (-2., -1.)
        spin_for(.4)
        future = reset.call_async(Trigger.Request())
        end = time.monotonic()+4.
        while not future.done() and time.monotonic()<end:
            spin_for(.05)
        assert future.done() and future.result().success, (
            future.result().message if future.done() else 'Replan service response timed out')
        spin_for(.3)
        reference = received['robot_a']['mpc_reference_path']
        first, last = reference.poses[0].pose.position, reference.poses[-1].pose.position
        assert abs(first.x+2.) < 1e-5 and abs(first.y+1.) < 1e-5
        assert abs((last.y-first.y)/(last.x-first.x)-1./5.) < 1e-5
        print('PASS: recovery replan rebuilds the original reference from current off-path GT', flush=True)
        ego_override = (3., 0.)
        spin_for(.5)
        assert received['robot_a']['mpc_status'].data == 'goal_reached'
        print('PASS: recovery replan preserves original final goal instead of the extended spline endpoint', flush=True)
    except Exception:
        for robot, log in zip(names, logs):
            log.seek(0)
            print(f'--- {robot} C++ log ---\n{log.read()[-7000:]}', flush=True)
        raise
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        node.destroy_node()
        rclpy.shutdown()
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
