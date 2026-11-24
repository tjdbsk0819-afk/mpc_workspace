"""Real D-scenario GT/MPC/RPP avoids a known cylinder (isolated ROS domain 230)."""
import json,math,os,signal,subprocess,tempfile,time
from pathlib import Path


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='230':raise RuntimeError('Use isolated ROS_DOMAIN_ID=230')
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
    rclpy.init();node=rclpy.create_node('static_obstacle_check');robots=('robot_a','robot_b');latest={};subs=[]
    pubs={t:node.create_publisher(k,t,10) for t,k in [('/clock',Clock),('/tf',TFMessage),
        ('/robot_a/odom',Odometry),('/robot_b/odom',Odometry),('/robot_a/mpc_plan',MpcPlan)]}
    for robot in robots:
        for suffix,kind in [('mpc_plan',MpcPlan),('mpc_peer_distribution',String),('cmd_vel',Twist),('control_status',String),('control_diagnostics',String)]:
            key=robot+'/'+suffix
            subs.append(node.create_subscription(kind,'/'+key,lambda m,k=key:latest.update({k:m}),qos_profile_sensor_data))
    processes=[];logs=[];begin=time.monotonic();inside=False;injected=None
    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=5.)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    def spawn(args):
        log=tempfile.TemporaryFile(mode='w+');logs.append(log)
        p=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    def stamp(t):
        ns=round(t*1e9);return Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
    def run(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            assert all(p.poll() is None for p in processes),'D node exited'
            now=100.+time.monotonic()-begin;t=stamp(now);pubs['/clock'].publish(Clock(clock=t));transforms=[]
            for robot,x,yaw in [('robot_a',-.9 if inside else -3.,0.),('robot_b',3.,math.pi)]:
                o=Odometry();o.header.frame_id='world';o.header.stamp=t;o.child_frame_id=robot+'/base_link'
                o.pose.pose.position.x=x;o.pose.pose.orientation.z=math.sin(yaw/2.);o.pose.pose.orientation.w=math.cos(yaw/2.)
                pubs['/'+robot+'/odom'].publish(o)
                tf=TransformStamped();tf.header=o.header;tf.child_frame_id=o.child_frame_id
                tf.transform.translation.x=x;tf.transform.rotation=o.pose.pose.orientation;transforms.append(tf)
            pubs['/tf'].publish(TFMessage(transforms=transforms))
            if injected:
                p=MpcPlan();p.header.frame_id='world';p.header.stamp=t;p.path.header=p.header;p.valid=True;p.status='tracking'
                p.max_acceleration=.5;p.max_yaw_rate=.8;p.minimum_separation=1.15;p.target_speed=1. if injected=='crossing' else .2
                for k in range(2):
                    pose=PoseStamped();pose.header.frame_id='world'
                    pose.header.stamp=stamp(now+(2.4 if injected=='crossing' else .4)*k)
                    pose.pose.position.x=(-1.2+2.4*k) if injected=='crossing' else (-3.+.08*k)
                    pose.pose.orientation.w=1.;p.path.poses.append(pose)
                    p.linear_velocities.append(p.target_speed);p.linear_accelerations.append(0.);p.angular_velocities.append(0.)
                pubs['/robot_a/mpc_plan'].publish(p)
            for _ in range(5):rclpy.spin_once(node,timeout_sec=.001)
            time.sleep(.01)
    def zero():
        c=latest['robot_a/cmd_vel'];assert abs(c.linear.x)<1e-9 and abs(c.angular.z)<1e-9,c
    try:
        with tempfile.TemporaryDirectory() as temp:
            spawn(['ros2','launch','antbot_mpc_ros','gt_peer_cv.launch.py'])
            planner=spawn(['ros2','launch','mpc_planner_antbot','two_antbot_mpc.launch.py','scenario:=D'])
            spawn(['ros2','launch','antbot_navigation','isaac_rpp.launch.py','scenario:=D','output_directory:='+temp])
            run(6.)
            for robot in robots:
                p=latest[robot+'/mpc_plan'];assert p.valid,p.status
                assert len(p.path.poses)==30 and any(abs(v.pose.position.y)>.5 for v in p.path.poses)
                assert all(math.hypot(v.pose.position.x,v.pose.position.y)>=1.05-1e-6 for v in p.path.poses)
                d=json.loads(latest[robot+'/mpc_peer_distribution'].data)
                assert d['model']==model and len(d['static_obstacles'])==1
                assert abs(d['static_obstacles'][0]['radius']-.4)<1e-12
                diag=json.loads(latest[robot+'/control_diagnostics'].data)
                assert diag['peer_prediction_model']==model
                assert diag['static_obstacle_enabled'] and diag['predicted_static_clearance']>=0.
                assert latest[robot+'/cmd_vel'].linear.x>0.,diag
            print('PASS: both real D planners avoid the fixed cylinder and Gaussian peer; native RPP commands include fixed-object braking checks',flush=True)
            inside=True;run(.8);assert not latest['robot_a/mpc_plan'].valid
            assert latest['robot_a/mpc_plan'].status=='inside_static_obstacle';zero()
            print('PASS: ego inside the cylinder safety area invalidates MPC and stops commands',flush=True)
            inside=False;run(1.2)
            stop(planner);processes.remove(planner)
            injected='crossing';run(.8);zero()
            assert latest['robot_a/control_status'].data=='mpc_path_hits_static_obstacle',latest['robot_a/control_status']
            print('PASS: native RPP rejects a cylinder-crossing plan even when both endpoints are outside',flush=True)
            injected='safe';run(1.2);assert latest['robot_a/cmd_vel'].linear.x>0.,latest['robot_a/control_status']
            print('PASS: native RPP resumes on a fresh plan clear of the fixed cylinder',flush=True)
    except Exception:
        for log in logs:log.seek(0);print(log.read()[-9000:],flush=True)
        raise
    finally:
        for p in processes:stop(p)
        node.destroy_node();rclpy.shutdown()
        for log in logs:log.close()

if __name__=='__main__':main()
