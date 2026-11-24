"""Real MPC + real Nav2 RPP + adapter against synthetic unicycle GT (domain 228).

No Isaac/Swerve physics: only validate the closed ROS path-to-command connection.
A four-second initial freeze deliberately exercises homogeneous MPC recovery.
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
    if os.environ.get('ROS_DOMAIN_ID') not in ('227','228'):raise RuntimeError('Use isolated ROS_DOMAIN_ID=227 or 228')
    import rclpy
    from antbot_interfaces.msg import PeerState, MpcPlan
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from antbot_mpc_ros.timed_tracking import Motion,step
    def interrupted(*_):raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    rclpy.init();node=rclpy.create_node('mpc_rpp_kinematic_world')
    robots=('robot_a','robot_b');goals=((3.,0.),(-3.,0.))
    states=json.loads(os.environ.get('ANTBOT_INITIAL_STATES','null')) or [[-3.,0.,0.,0.,0.],[3.,0.,math.pi,0.,0.]]
    assert len(states)==2 and all(len(x)==5 for x in states)
    cmd={r:(0.,0.,0.) for r in robots};latest={};subs=[]
    clock=node.create_publisher(Clock,'/clock',10);tf=TransformBroadcaster(node)
    odom={r:node.create_publisher(Odometry,f'/{r}/odom',10) for r in robots}
    peer={r:node.create_publisher(PeerState,f'/{r}/peer_state',10) for r in robots}
    def command(r,m):
        assert abs(m.linear.y)<1e-9,'Direct lateral recovery command is forbidden'
        cmd[r]=(m.linear.x,m.angular.z,time.monotonic())
    for r in robots:
        subs.append(node.create_subscription(Twist,f'/{r}/cmd_vel',lambda m,r=r:command(r,m),10))
        for suffix,kind in [('recovery_status',String),('control_status',String),('mpc_plan',MpcPlan),('rpp_cmd_vel',Twist),('control_diagnostics',String)]:
            subs.append(node.create_subscription(kind,f'/{r}/{suffix}',lambda m,k=(r,suffix):latest.update({k:m}),10))
    processes=[];logs=[];history=[];result={'verified':False,'environment':'synthetic unicycle GT, real MPC and RPP; no Isaac physics'}
    def spawn(args):
        log=tempfile.TemporaryFile(mode='w+');logs.append(log)
        processes.append(subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    def timestamp(t):
        ns=round(t*1e9);return Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
    begin=time.monotonic();prev=begin;next_sample=0.;minimum_separation=math.inf
    try:
        spawn(['ros2','launch','mpc_planner_antbot','two_antbot_mpc.launch.py','scenario:=A'])
        spawn(['ros2','launch','antbot_navigation','isaac_rpp.launch.py'])
        for i,r in enumerate(robots):
            spawn(['ros2','run','antbot_mpc_ros','mpc_rpp_adapter','--ros-args','-r',f'__ns:=/{r}',
                   '-p','use_sim_time:=true','-p',f'ego_robot:={r}','-p',f'peer_robot:={robots[1-i]}'])
        while time.monotonic()-begin<100.:
            wall=time.monotonic();elapsed=wall-begin;dt=min(.03,wall-prev);prev=wall
            t=timestamp(100.+elapsed);clock.publish(Clock(clock=t))
            for i,r in enumerate(robots):
                v,w,received=cmd[r]
                if elapsed<4.5 or wall-received>.2:v=w=0.
                current=Motion(*states[i])
                moved=step(current,v,w,dt,.5,.5)
                states[i]=[moved.x,moved.y,moved.yaw,moved.v,moved.w]
            minimum_separation=min(minimum_separation,math.hypot(states[0][0]-states[1][0],states[0][1]-states[1][1]))
            transforms=[]
            for i,r in enumerate(robots):
                x,y,yaw,v,w=states[i]
                msg=Odometry();msg.header.frame_id='world';msg.header.stamp=t;msg.child_frame_id=r+'/base_link'
                msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
                msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
                msg.twist.twist.linear.x=v
                msg.twist.twist.angular.z=w
                odom[r].publish(msg)
                transform=TransformStamped();transform.header=msg.header;transform.child_frame_id=msg.child_frame_id
                transform.transform.translation.x=x;transform.transform.translation.y=y
                transform.transform.rotation=msg.pose.pose.orientation;transforms.append(transform)
                other=states[1-i];p=PeerState();p.header=msg.header;p.robot_id=robots[1-i];p.valid=True;p.status='ok'
                p.x,p.y=other[:2];p.vx=other[3]*math.cos(other[2]);p.vy=other[3]*math.sin(other[2]);peer[r].publish(p)
            tf.sendTransform(transforms)
            for _ in range(15):rclpy.spin_once(node,timeout_sec=.0001)
            if elapsed>=next_sample:
                next_sample=elapsed+.1
                row={'time':elapsed,'separation':math.hypot(states[0][0]-states[1][0],states[0][1]-states[1][1])}
                for i,r in enumerate(robots):
                    d=math.hypot(states[i][0]-goals[i][0],states[i][1]-goals[i][1])
                    status=latest.get((r,'control_status'));recovery=latest.get((r,'recovery_status'))
                    plan=latest.get((r,'mpc_plan'));raw=latest.get((r,'rpp_cmd_vel'))
                    diagnostic=latest.get((r,'control_diagnostics'))
                    row[r]={'diagnostic':json.loads(diagnostic.data) if diagnostic else {},'mpc_speed':plan.target_speed if plan else None,
                            'mpc_yaw_rate':plan.target_yaw_rate if plan else None,
                            'raw_rpp':[raw.linear.x,raw.angular.z] if raw else None,'state':states[i][:],'goal_distance':d,'command':cmd[r][:2],
                            'control':status.data if status else '',
                            'recovery':json.loads(recovery.data) if recovery else {}}
                history.append(row)
                if len(history)%100==0:print(json.dumps(row),flush=True)
                if all(row[r]['goal_distance']<=.2 for r in robots):break
            time.sleep(.003)
        result.update(final=history[-1],minimum_center_separation_m=minimum_separation,
                      recovery_episodes={r:max(h[r]['recovery'].get('episode',0) for h in history) for r in robots})
        assert all(result['recovery_episodes'][r]>=1 for r in robots),'Both robots must exercise recovery'
        assert all(history[-1][r]['goal_distance']<=.2 for r in robots),'Both original goals not reached'
        assert result['minimum_center_separation_m']>=1.15-1e-6,'Safety margin violated'
        result['verified']=True
        print('PASS: real MPC/RPP closed loop, homogeneous recovery and both goals on synthetic GT',flush=True)
    except Exception:
        for log in logs:
            log.seek(0);print(log.read()[-5000:],flush=True)
        raise
    finally:
        for p in processes:
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        Path(os.environ.get('ANTBOT_LOOP_OUTPUT','/workspace/results/aligned_tracking_loop.json')).write_text(json.dumps({'result':result,'trace':history},indent=2)+'\n')
        print(json.dumps(result),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for log in logs:log.close()


if __name__=='__main__':main()
