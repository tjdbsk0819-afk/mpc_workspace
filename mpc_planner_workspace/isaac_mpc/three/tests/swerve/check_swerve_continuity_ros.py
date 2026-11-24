"""Real controller_manager/Swerve/Isaac bridge against synthetic joint feedback.

Domain 226, FastDDS only. No Isaac, MPC or user-domain commands. No result files.
"""
from collections import defaultdict
import math
import os
import signal
import subprocess
import tempfile
import time
import copy


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='226' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_fastrtps_cpp':
        raise RuntimeError('Use isolated domain 226 and rmw_fastrtps_cpp')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from builtin_interfaces.msg import Time
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory
    from antbot_interfaces.msg import JointCommandFeedback
    rclpy.init();node=rclpy.create_node('synthetic_swerve_feedback')
    robots=('robot_a','robot_b')
    names=['steering_front_left_joint','steering_front_right_joint','steering_rear_left_joint','steering_rear_right_joint',
           'wheel_front_left_joint','wheel_front_right_joint','wheel_rear_left_joint','wheel_rear_right_joint']
    initial_angles=[.95,.2,-.2,-.3]
    latest={r:(list(initial_angles),[0.]*4) for r in robots}
    pending={r:[] for r in robots}
    applied={r:None for r in robots}
    applied_when={r:None for r in robots}
    delay=.06
    forced_wheel_speed=None
    feedback_enabled=True
    emergency_stop=False
    forced_steering=None
    custom_setpoint=None
    raw_commands=defaultdict(list)
    state={r:(list(initial_angles)+[0.]*4,[0.]*8) for r in robots}
    commands=defaultdict(list);profiles=defaultdict(list);phase='startup';blocked=False
    subs=[]
    for r in robots:
        def receive(m,r=r):
            pending[r].append((time.monotonic()+delay,copy.deepcopy(m)))
            raw_commands[r].append((phase,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,list(m.position[:4]),list(m.velocity[4:])))
        def commanded(m,r=r):commands[r].append((phase,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,list(m.position[:4]),list(m.velocity[4:])))
        def planned(m,r=r):profiles[r].append((phase,m))
        subs.append(node.create_subscription(JointState,f'/{r}/joint_command',receive,10))
        subs.append(node.create_subscription(JointState,f'/{r}/joint_commanders',commanded,10))
        subs.append(node.create_subscription(JointTrajectory,f'/{r}/antbot_swerve_controller/planned_trajectory',planned,10))
    clock=node.create_publisher(Clock,'/clock',10)
    joints={r:node.create_publisher(JointState,f'/{r}/joint_states',qos_profile_sensor_data) for r in robots}
    feedback={r:node.create_publisher(JointCommandFeedback,f'/{r}/joint_feedback',qos_profile_sensor_data) for r in robots}
    velocity={r:node.create_publisher(Twist,f'/{r}/cmd_vel',10) for r in robots}
    begin=previous=time.monotonic()
    log=tempfile.TemporaryFile(mode='w+')
    process=subprocess.Popen(['ros2','launch','antbot_mpc_ros','two_antbot_controllers.launch.py'],
                             stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

    def run(duration,moving=False,publishing=True):
        nonlocal previous
        end=time.monotonic()+duration
        while time.monotonic()<end:
            now=time.monotonic();dt=now-previous;previous=now;t=100.+now-begin
            ns=round(t*1e9);stamp=Time(sec=ns//10**9,nanosec=ns%10**9)
            clock.publish(Clock(clock=stamp))
            for i,r in enumerate(robots):
                while pending[r] and pending[r][0][0]<=now:
                    _,command=pending[r].pop(0)
                    applied[r]=command
                    applied_when[r]=stamp
                    latest[r]=(list(command.position[:4]),list(command.velocity[4:]))
                pos,vel=state[r]
                if not blocked:
                    angle,wheel=latest[r]
                    for j in range(4):
                        vel[j]=(angle[j]-pos[j])/max(dt,1e-5);pos[j]=angle[j]
                        vel[j+4]=wheel[j];pos[j+4]+=wheel[j]*dt
                if forced_wheel_speed is not None:
                    vel[4:]=[forced_wheel_speed]*4
                if forced_steering is not None:
                    pos[0]=forced_steering;vel[0]=0.
                msg=JointState();msg.header.stamp=stamp;msg.name=names;msg.position=pos;msg.velocity=vel
                joints[r].publish(msg)
                fb=JointCommandFeedback();fb.state=copy.deepcopy(msg)
                if applied[r] is not None:
                    fb.applied_command=applied[r];fb.applied_at=applied_when[r];fb.command_valid=True
                if feedback_enabled:
                    feedback[r].publish(fb)
                if publishing:
                    cmd=Twist()
                    if moving:
                        cmd.linear.x=.2+.004*math.sin((now-begin)*67.)
                        cmd.angular.z=(1. if i==0 else -1.)*(.1+.004*math.sin((now-begin)*43.))
                        if custom_setpoint is not None:
                            cmd.linear.x,cmd.angular.z=custom_setpoint
                    if emergency_stop:
                        cmd.linear.z=-1.
                    velocity[r].publish(cmd)
            for _ in range(6):rclpy.spin_once(node,timeout_sec=0.)
            time.sleep(.008)

    try:
        run(5.)
        assert all(commands[r] for r in robots),'Controllers did not activate'
        for r in robots:
            rows=[x for x in raw_commands[r] if x[0]=='startup']
            assert any(x[2][0]>.94 for x in rows),'Out-of-inset startup was not exercised'
            assert latest[r][0][0]<=.939001,'Startup failed to recover inside inset range'
            for a,b in zip(rows,rows[1:]):
                dt=b[1]-a[1]
                if dt>0.:
                    assert abs(b[2][0]-a[2][0])<=5.01*dt+1e-5,'Startup boundary recovery jumped steering'
            assert all(max(map(abs,x[3]))<1e-9 for x in rows),'Startup recovery moved wheels'
        print('PASS: startup outside inset range returns continuously with zero wheels',flush=True)
        phase='retarget';run(6.,moving=True)
        for r in robots:
            rows=[h for h in commands[r] if h[0]=='retarget']
            assert max(max(map(abs,h[3])) for h in rows)>1.,'Drive remained starved by profile restarts'
            assert max(max(map(abs,h[2])) for h in rows)>.08,'Steering failed to progress'
            ps=[p for label,p in profiles[r] if label=='retarget']
            assert len(ps)>30,'Frequent retarget case was not exercised'
            assert any(any(abs(v)>.01 for v in p.points[0].velocities[:4]) for p in ps),'Steering velocity reset on every retarget'
            max_speed=max_accel=0.
            for a,b in zip(rows,rows[1:]):
                dt=b[1]-a[1]
                if dt<=0.:continue
                max_speed=max(max_speed,max(abs(x-y)/dt for x,y in zip(a[2],b[2])))
                max_accel=max(max_accel,max(abs(x-y)/dt for x,y in zip(a[3],b[3])))
            assert max_speed<5.05,('steering speed limit',max_speed)
            assert max_accel<19.0,('wheel acceleration limit',max_accel)
            print(f'PASS: {r} actual ROS progress with 60ms command delay and matched applied feedback, {len(ps)} retargets, steering <= {max_speed:.3f}rad/s, wheel accel <= {max_accel:.3f}rad/s2',flush=True)
        log.flush();log.seek(0)
        assert 'Persistent joint tracking error' not in log.read(),'Transport delay was misclassified as persistent tracking error'
        phase='stop';run(.7)
        for r in robots:
            tail=[h for h in commands[r] if h[0]=='stop'][-5:]
            assert tail and all(max(map(abs,h[3]))<1e-9 for h in tail),'Zero command did not stop'
        print('PASS: explicit stop clears active profile and commands zero wheels',flush=True)
        phase='resume';run(1.5,moving=True)
        assert all(max(map(abs,latest[r][1]))>1. for r in robots),'Resume failed'
        phase='inertial_stop';forced_wheel_speed=3.;run(.4,moving=False)
        phase='inertial_restart';run(.4,moving=True)
        for r in robots:
            tail=[h for h in commands[r] if h[0]=='inertial_restart'][-5:]
            assert tail and all(max(map(abs,h[3]))<1e-9 for h in tail),'Restarted before measured wheels stopped'
            assert max(max(abs(a-b) for a,b in zip(tail[0][2],h[2])) for h in tail)<1e-9,'Stop hold moved steering between old feedback samples'
        print('PASS: nonzero measured wheel speed holds zero commands and one steering target despite a new drive request',flush=True)
        forced_wheel_speed=None;phase='confirmed_restart';run(1.5,moving=True)
        assert all(max(map(abs,latest[r][1]))>1. for r in robots),'Confirmed stop did not resume'
        forced_wheel_speed=3.;emergency_stop=True;phase='emergency_stop';run(.3,moving=True)
        emergency_stop=False;phase='emergency_restart';run(.4,moving=True)
        for r in robots:
            tail=[h for h in commands[r] if h[0]=='emergency_restart'][-5:]
            assert tail and all(max(map(abs,h[3]))<1e-9 for h in tail),'Emergency release bypassed physical stop confirmation'
        forced_wheel_speed=None;phase='emergency_recovered';run(1.5,moving=True)
        assert all(max(map(abs,latest[r][1]))>1. for r in robots),'Emergency stop did not recover after measured stop'
        print('PASS: emergency stop release also waits for actual wheel stop before restarting',flush=True)
        phase='blocked';blocked=True
        for r in robots:state[r][1][4:]=[0.]*4
        run(1.5,moving=True)
        log.flush();log.seek(0);text=log.read()
        assert 'Persistent joint tracking error' in text,'Tracking divergence was not detected'
        print('PASS: persistent blocked joint feedback triggers a latched wheel stop',flush=True)
        blocked=False;phase='recover';run(1.5,moving=True)
        assert all(max(map(abs,latest[r][1]))>1. for r in robots),'Reanchor recovery failed'
        phase='timeout';run(1.,publishing=False)
        assert all(max(map(abs,latest[r][1]))<1e-9 for r in robots),'Command timeout retained motion'
        print('PASS: feedback recovery resumes and command timeout clears motion',flush=True)
        phase='feedback_resume';run(1.5,moving=True)
        feedback_enabled=False;phase='feedback_missing';run(.8,moving=True)
        assert all(max(map(abs,latest[r][1]))<1e-9 for r in robots),'Missing atomic feedback left a moving command latched'
        feedback_enabled=True;phase='feedback_restored';run(1.5,moving=True)
        assert all(max(map(abs,latest[r][1]))>1. for r in robots),'Restored feedback did not confirm stop and resume'
        print('PASS: joint_states alone cannot authorize motion after atomic feedback timeout; restored feedback recovers',flush=True)
        phase='small_spin';custom_setpoint=(0.,-.008);run(2.,moving=True)
        assert all(max(map(abs,latest[r][1]))>.015 for r in robots),'Small spin remained stopped'
        print('PASS: actual ROS Swerve executes a nonzero 0.008rad/s spin below the old stop threshold',flush=True)
        phase='boundary_approach';custom_setpoint=(.02,.06);run(2.,moving=True)
        for r in robots:
            assert latest[r][0][0]>.90,'Boundary approach was not exercised'
            assert max(map(abs,latest[r][0]))<=.939001,'Normal target exceeded inset steering range'
        phase='boundary_overshoot';forced_steering=.960;run(.35,moving=True)
        for r in robots:
            rows=[x for x in raw_commands[r] if x[0]=='boundary_overshoot'][-8:]
            assert rows and all(x[2][0]>.90 for x in rows),'Limit recovery jumped steering toward zero'
            assert all(max(map(abs,x[3]))<1e-9 for x in rows),'Boundary recovery did not stop wheels'
        forced_steering=None;phase='boundary_recovered';run(1.5,moving=True)
        assert all(max(map(abs,latest[r][1]))>.02 for r in robots),'Boundary recovery failed to resume'
        print('PASS: inset targets, physical limit overshoot, zero-wheel continuous recovery and confirmed restart',flush=True)
    except Exception:
        log.seek(0);print(log.read()[-9000:],flush=True);raise
    finally:
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node();rclpy.shutdown();log.close()


if __name__=='__main__':main()
