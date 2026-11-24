"""Real regenerated acados: measured yaw envelope, history expiry and recovery.

Only synthetic state is published, in isolated domain 229; no cmd_vel is sent.
"""
import bisect
import json
import math
import os
import signal
import subprocess
import tempfile
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '229':
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=229')
    from pathlib import Path
    solver=json.loads((Path(__file__).resolve().parents[2]/'mpc_planning/mpc_planner_solver/acados/Solver/Solver.json').read_text())
    assert solver['dims']['nx']==5 and solver['dims']['nu']==2 and solver['dims']['nh_0']==2
    import rclpy
    from antbot_interfaces.msg import MpcPlan, PeerState
    from builtin_interfaces.msg import Time
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    rclpy.init();node=rclpy.create_node('yaw_ocp_diagnostic')
    clock=node.create_publisher(Clock,'/clock',10)
    odom=node.create_publisher(Odometry,'/robot_a/odom',10)
    peer=node.create_publisher(PeerState,'/robot_a/peer_state',10)
    plans=[];reports=[];samples=[];process=None
    subs=[node.create_subscription(MpcPlan,'/robot_a/mpc_plan',plans.append,20),
          node.create_subscription(String,'/robot_a/recovery_status',lambda m:reports.append(json.loads(m.data)),20)]
    begin=time.monotonic();rate=.2;valid=True
    def feed():
        now=100.+time.monotonic()-begin;ns=round(now*1e9)
        stamp=Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
        clock.publish(Clock(clock=stamp));samples.append((now,rate))
        m=Odometry();m.header.stamp=stamp;m.header.frame_id='world';m.child_frame_id='robot_a/base_link'
        m.pose.pose.position.x=-3.;m.pose.pose.orientation.w=1.;m.twist.twist.angular.z=rate;odom.publish(m)
        p=PeerState();p.header=m.header;p.robot_id='robot_b';p.x=8.;p.y=5.;p.valid=valid;p.status='ok';peer.publish(p)
    def run(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert process.poll() is None,'Planner exited'
            feed();rclpy.spin_once(node,timeout_sec=.005);time.sleep(.01)
    def checked(rows):
        good=[p for p in rows if p.valid]
        assert good,'No valid OCP output'
        times=[t for t,_ in samples]
        for p in good:
            t=p.header.stamp.sec+p.header.stamp.nanosec*1e-9
            i=max(0,bisect.bisect_right(times,t)-1);measured=samples[i][1]
            assert len(p.angular_velocities)==30
            assert abs(p.angular_velocities[0]-measured)<=.050001,(t,p.angular_velocities[0],measured)
        return good
    log=tempfile.TemporaryFile(mode='w+')
    try:
        process=subprocess.Popen(['ros2','run','mpc_planner_antbot','antbot_planner','--ros-args',
            '-r','__ns:=/robot_a','-p','use_sim_time:=true','-p','recovery_enabled:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        run(3.)
        checked(plans)
        assert any(r.get('yaw_continuity_active') for r in reports)
        print('PASS: real acados first yaw is within measured +0.2 +/-0.05 and prior-plan cost activates',flush=True)
        offset=len(plans);rate=-.2;run(1.2)
        # Exclude only the transition's already-in-flight plan by source stamp.
        later=[p for p in plans[offset:] if p.valid and p.angular_velocities[0]<0.]
        checked(later)
        assert later,'Opposite measured rotation was pinned by previous-plan bias'
        print('PASS: new measured opposite yaw overrides previous-plan preference; optimized input remains bounded',flush=True)
        service=node.create_client(Trigger,'/robot_a/mpc_planner/replan_from_current')
        assert service.wait_for_service(timeout_sec=2)
        offset=len(reports);future=service.call_async(Trigger.Request());run(.5)
        assert future.done() and future.result().success
        tracking=[x for x in reports[offset:] if x['phase'] in ('tracking','tracking_candidate')]
        assert tracking and all(not x['yaw_continuity_active'] for x in tracking)
        for x in tracking:
            assert -.250001<=x['yaw_first_solution']<=-.149999
        run(1.)
        assert any(x.get('yaw_continuity_active') for x in reports[offset:])
        print('PASS: replan clears history and relaxes soft cost for 1sim second while first-input envelope stays active',flush=True)
        valid=False;run(.4);assert plans and not plans[-1].valid
        offset=len(reports);valid=True;run(.5)
        tracking=[x for x in reports[offset:] if x['phase'] in ('tracking','tracking_candidate')]
        assert tracking and not tracking[0]['yaw_continuity_active']
        assert any(x.get('yaw_continuity_active') for x in tracking[1:])
        print('PASS: invalid input clears previous selected history; fresh plans rebuild it',flush=True)
    except Exception:
        log.seek(0);print(log.read()[-10000:],flush=True);raise
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node();rclpy.shutdown();log.close()


if __name__=='__main__':main()
