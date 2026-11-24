"""Real C-scenario GT/MPC/native RPP and malformed/out-of-bounds plan checks (domain 231)."""
import json,math,os,signal,subprocess,tempfile,time
from pathlib import Path


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='231':raise RuntimeError('Use isolated ROS_DOMAIN_ID=231')
    import rclpy
    import yaml
    from rclpy.qos import qos_profile_sensor_data
    from antbot_interfaces.msg import MpcPlan
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Twist,TransformStamped,PoseStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    model=yaml.safe_load((Path(__file__).resolve().parents[1]/'mpc_planning/mpc_planner_antbot/config/peer_prediction.yaml').read_text())['/**']['ros__parameters']['peer_prediction_model']
    rclpy.init();node=rclpy.create_node('corridor_check');names=('robot_a','robot_b');latest={};subs=[]
    pubs={topic:node.create_publisher(kind,topic,10) for topic,kind in [('/clock',Clock),('/tf',TFMessage),
        ('/robot_a/odom',Odometry),('/robot_b/odom',Odometry),('/robot_a/mpc_plan',MpcPlan)]}
    for robot in names:
        for suffix,kind in [('mpc_plan',MpcPlan),('mpc_peer_distribution',String),('cmd_vel',Twist),('control_status',String),('control_diagnostics',String)]:
            key=robot+'/'+suffix
            subs.append(node.create_subscription(kind,'/'+key,lambda m,k=key:latest.update({k:m}),qos_profile_sensor_data))
    processes=[];logs=[];begin=time.monotonic();outside=False;injected=None
    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=5.)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    def spawn(args):
        log=tempfile.TemporaryFile(mode='w+');logs.append(log)
        proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(proc);return proc
    def timestamp(t):
        ns=round(t*1e9);return Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
    def run(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            assert all(p.poll() is None for p in processes),'C launch exited'
            now=100.+time.monotonic()-begin;t=timestamp(now);pubs['/clock'].publish(Clock(clock=t));transforms=[]
            for robot,x,y,yaw in [('robot_a',-2.,.3 if outside else 0.,0.),('robot_b',2.,0.,math.pi)]:
                o=Odometry();o.header.frame_id='world';o.header.stamp=t;o.child_frame_id=robot+'/base_link'
                o.pose.pose.position.x=x;o.pose.pose.position.y=y;o.pose.pose.orientation.z=math.sin(yaw/2.);o.pose.pose.orientation.w=math.cos(yaw/2.)
                pubs['/'+robot+'/odom'].publish(o)
                tf=TransformStamped();tf.header=o.header;tf.child_frame_id=o.child_frame_id
                tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.rotation=o.pose.pose.orientation;transforms.append(tf)
            pubs['/tf'].publish(TFMessage(transforms=transforms))
            if injected:
                p=MpcPlan();p.header.frame_id='world';p.header.stamp=t;p.path.header=p.header;p.valid=True;p.status='tracking'
                p.max_acceleration=.5;p.max_yaw_rate=.8;p.minimum_separation=1.15;p.target_speed=.2
                for k in range(2):
                    pose=PoseStamped();pose.header.frame_id='world';pose.header.stamp=timestamp(now+.4*k)
                    y=.3*k if injected=='outside' else .145 if injected=='arc' else 0.
                    yaw=(.16-.32*k) if injected=='arc' else 0.
                    pose.pose.position.x=-2.+.2*k;pose.pose.position.y=y
                    pose.pose.orientation.z=math.sin(yaw/2.);pose.pose.orientation.w=math.cos(yaw/2.);p.path.poses.append(pose)
                    p.linear_velocities.append(.5 if injected=='arc' else .2);p.linear_accelerations.append(0.);p.angular_velocities.append(-.8 if injected=='arc' else 0.)
                pubs['/robot_a/mpc_plan'].publish(p)
            for _ in range(5):rclpy.spin_once(node,timeout_sec=.001)
            time.sleep(.01)
    def zero():
        cmd=latest['robot_a/cmd_vel'];assert abs(cmd.linear.x)<1e-9 and abs(cmd.angular.z)<1e-9,cmd
    try:
        with tempfile.TemporaryDirectory() as temp:
            spawn(['ros2','launch','antbot_mpc_ros','gt_peer_cv.launch.py'])
            planner=spawn(['ros2','launch','mpc_planner_antbot','two_antbot_mpc.launch.py','scenario:=C'])
            spawn(['ros2','launch','antbot_navigation','isaac_rpp.launch.py','scenario:=C','output_directory:='+temp])
            run(6.)
            for robot in names:
                p=latest[robot+'/mpc_plan'];assert p.valid,p.status
                assert len(p.path.poses)==30
                for pose in p.path.poses:
                    assert -3.450001<=pose.pose.position.x<=3.450001 and abs(pose.pose.position.y)<=.150001
                d=json.loads(latest[robot+'/mpc_peer_distribution'].data)
                assert d['model']==model and d['workspace_bounds']==[-4.,4.,-.7,.7]
                diag=json.loads(latest[robot+'/control_diagnostics'].data)
                assert diag['peer_prediction_model']==model
                assert diag['workspace_bounds_enabled'] and diag['predicted_wall_clearance']>=0.
                assert latest[robot+'/cmd_vel'].linear.x>0.,diag
            print(f'PASS: real C planners and native RPP move inside shared wall bounds with {model}',flush=True)
            outside=True;run(.8);assert not latest['robot_a/mpc_plan'].valid
            assert latest['robot_a/mpc_plan'].status=='outside_workspace_bounds';zero()
            print('PASS: ego outside wall-safe strip invalidates real MPC and stops native RPP',flush=True)
            outside=False;run(1.2);assert latest['robot_a/mpc_plan'].valid
            stop(planner);processes.remove(planner)
            for mode in ('outside','arc'):
                injected=mode;run(.8);zero()
                assert latest['robot_a/control_status'].data=='mpc_path_outside_workspace',latest['robot_a/control_status']
            print('PASS: native RPP rejects wall-crossing plans, including arcs with both knots inside the corridor',flush=True)
            injected='safe';run(1.2);assert latest['robot_a/cmd_vel'].linear.x>0.,latest['robot_a/control_status']
            print('PASS: native RPP resumes on a fresh wall-safe plan',flush=True)
    except Exception:
        for log in logs:log.seek(0);print(log.read()[-8000:],flush=True)
        raise
    finally:
        for p in processes:stop(p)
        node.destroy_node();rclpy.shutdown()
        for log in logs:log.close()

if __name__=='__main__':main()
