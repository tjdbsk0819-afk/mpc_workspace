"""Real Nav2 RPP plugin in the integrated node; domain 229, no physical robot."""
import json,math,os,signal,subprocess,tempfile,time,sys
from pathlib import Path

def main():
    if os.environ.get('ROS_DOMAIN_ID')!='229':raise RuntimeError('Use isolated ROS domain 229')
    import rclpy,yaml
    from antbot_interfaces.msg import MpcPlan,PeerState
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Twist,PoseStamped,TransformStamped
    from nav_msgs.msg import Odometry,Path as RosPath
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import qos_profile_sensor_data
    from simulation.helpers.rpp_config import mpc_rpp_parameters
    root=Path(__file__).resolve().parents[2]
    template=yaml.safe_load((root/'path_following_controller/config/isaac/rpp.yaml').read_text())
    execution=yaml.safe_load((root/'path_following_controller/config/isaac/mpc_execution.yaml').read_text())['/**']['ros__parameters']
    legacy='--legacy-two' in sys.argv
    peers=['robot_b'] if legacy else ['robot_b','robot_c']
    params=mpc_rpp_parameters(template,execution,'robot_a',peers)
    rclpy.init();node=rclpy.create_node('native_rpp_check');latest={};received={};pubs={};subs=[]
    peer_topics={p:('/robot_a/peer_state' if legacy else f'/robot_a/peers/{p}/state') for p in peers}
    for topic,kind in [('/clock',Clock),('/tf',TFMessage),('/robot_a/odom',Odometry),('/robot_a/mpc_plan',MpcPlan),
                       *[(t,PeerState) for t in peer_topics.values()]]:
        pubs[topic]=node.create_publisher(kind,topic,10)
    for suffix,kind in [('cmd_vel',Twist),('control_status',String),('control_diagnostics',String),('rpp_tracking_path',RosPath)]:
        def callback(msg,key=suffix):latest[key]=msg;received[key]=received.get(key,0)+1
        subs.append(node.create_subscription(kind,'/robot_a/'+suffix,callback,qos_profile_sensor_data))
    log=tempfile.TemporaryFile(mode='w+');process=None;begin=time.monotonic();ns=0
    def run(duration,mode='normal',freeze=False):
        nonlocal ns
        end=time.monotonic()+duration
        while time.monotonic()<end:
            assert process.poll() is None,'Integrated RPP process exited'
            if not freeze:ns=round((100+time.monotonic()-begin)*1e9)
            t=Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000);pubs['/clock'].publish(Clock(clock=t))
            o=Odometry();o.header.frame_id='world';o.header.stamp=t;o.child_frame_id='robot_a/base_link';o.pose.pose.orientation.w=1.
            turning=mode in ('turn_positive','turn_negative')
            o.twist.twist.linear.x=0. if turning else .2
            if mode!='no_ego':pubs['/robot_a/odom'].publish(o)
            tf=TransformStamped();tf.header=o.header;tf.child_frame_id=o.child_frame_id;tf.transform.rotation.w=1.
            pubs['/tf'].publish(TFMessage(transforms=[tf]))
            for peer in peers:
                y=3. if peer=='robot_b' else -3.
                if mode=='missing_second' and peer==peers[-1]:continue
                p=PeerState();p.header=o.header;p.robot_id=peer;p.valid=True;p.status='ok';p.x=5.;p.y=y
                if mode=='close' and peer==peers[-1]:p.x=.9;p.y=0.
                pubs[peer_topics[peer]].publish(p)
            if mode!='no_plan':
                p=MpcPlan();p.header=o.header;p.path.header=o.header;p.valid=mode!='invalid_plan';p.status='tracking' if p.valid else 'solver_failed'
                p.target_speed=.2;p.max_acceleration=.5;p.max_yaw_rate=.8;p.minimum_separation=1.15
                rate=(.08 if mode=='turn_positive' else -.08) if turning else 0.
                p.target_speed=0. if turning else .2;p.target_yaw_rate=rate
                for k in range(30):
                    pose=PoseStamped();pose.header.frame_id='world';kp=ns+k*200_000_000
                    pose.header.stamp=Time(sec=kp//1_000_000_000,nanosec=kp%1_000_000_000)
                    pose.pose.orientation.z=math.sin(rate*k*.2/2.);pose.pose.orientation.w=math.cos(rate*k*.2/2.)
                    pose.pose.position.x=0. if turning else .04*k;p.path.poses.append(pose)
                    p.linear_velocities.append(0. if turning else .2);p.linear_accelerations.append(0.);p.angular_velocities.append(rate)
                pubs['/robot_a/mpc_plan'].publish(p)
            rclpy.spin_once(node,timeout_sec=.002);time.sleep(.01)
    def zero():
        c=latest['cmd_vel'];assert abs(c.linear.x)<1e-9 and abs(c.angular.z)<1e-9,(c,latest.get('control_status'))
    try:
        with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as f:
            yaml.safe_dump(params,f);f.flush()
            remaps=['-r','peers/robot_b/state:=peer_state'] if legacy else []
            process=subprocess.Popen(['ros2','run','antbot_navigation','mpc_rpp_controller','--ros-args','-r','__ns:=/robot_a','--params-file',f.name,*remaps],
                                     stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            run(6.)
            assert latest.get('cmd_vel') and latest['cmd_vel'].linear.x>0.,(latest.get('control_status'),latest.keys())
            assert latest['cmd_vel'].linear.x<=.201
            assert len(latest['rpp_tracking_path'].poses)>=2
            assert len(json.loads(latest['control_diagnostics'].data)['peer_checks'])==len(peers)
            info=node.get_publishers_info_by_topic('/robot_a/cmd_vel')
            assert len(info)==1 and info[0].node_name=='mpc_rpp_controller',info
            assert not node.get_publishers_info_by_topic('/robot_a/rpp_cmd_vel')
            names=node.get_node_names();assert 'mpc_rpp_adapter' not in names and 'controller_server' not in names,names
            print('PASS: native Nav2 RPP consumes MpcPlan and directly publishes bounded cmd_vel; no adapter/server/raw topic',flush=True)
            run(.6,'close');zero();assert latest['control_status'].data in ('proximity_stop','waiting_for_mpc_heading')
            print('PASS: second neighbor vetoes final native RPP command',flush=True)
            run(1.);assert latest['cmd_vel'].linear.x>0.
            run(.7,'missing_second');zero();assert latest['control_status'].data=='waiting_for_fresh_gt'
            run(.8);assert latest['cmd_vel'].linear.x>0.
            run(.3,'invalid_plan');zero();assert latest['control_status'].data=='mpc_solver_failed'
            run(.8);assert latest['cmd_vel'].linear.x>0.
            run(.7,'no_plan');zero();assert latest['control_status'].data=='expired_mpc_plan'
            run(.8);assert latest['cmd_vel'].linear.x>0.
            run(.8,'turn_positive');assert latest['cmd_vel'].angular.z>.005,latest['control_status']
            run(.8,'turn_negative');assert latest['cmd_vel'].angular.z<-.005,latest['control_status']
            print('PASS: optimized in-place prefixes retain small positive and negative RPP turns',flush=True)
            run(.8);assert latest['cmd_vel'].linear.x>0.
            run(.7,freeze=True);zero()
            run(.8);assert latest['cmd_vel'].linear.x>0.
            begin+=5.;run(1.5);assert latest['cmd_vel'].linear.x>0.
            client=node.create_client(SetBool,'/robot_a/mpc_rpp_controller/set_enabled')
            assert client.wait_for_service(timeout_sec=2.)
            future=client.call_async(SetBool.Request(data=False))
            run(.4);assert future.done() and future.result().success;zero()
            future=client.call_async(SetBool.Request(data=True))
            run(.8);assert future.done() and future.result().success and latest['cmd_vel'].linear.x>0.
            print('PASS: clock rewind and enable/disable service recover with fresh MPC inputs',flush=True)
            print('PASS: missing peer, invalid/expired plan and frozen clock stop commands; fresh complete data resumes',flush=True)
    except Exception:
        log.seek(0);print(log.read()[-12000:],flush=True);raise
    finally:
        if process and process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        log.close()

if __name__=='__main__':main()
