"""Actual GT observer -> Gaussian-only MPC -> native RPP, isolated domain 234."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID')!='234':
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=234')
    import rclpy
    import yaml
    from antbot_interfaces.msg import MpcPlan,PeerState
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import Twist,TransformStamped
    from nav_msgs.msg import Odometry,Path as RosPath
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import qos_profile_sensor_data
    from simulation.helpers.rpp_config import mpc_rpp_parameters
    root=Path(__file__).resolve().parents[1]
    model_file=root/'mpc_planning/mpc_planner_antbot/config/peer_prediction.yaml'
    shared=yaml.safe_load(model_file.read_text())['/**']['ros__parameters']
    # This is specifically a no-CV random-walk regression. Keep its explicit
    # model separate from the currently selected production comparison model.
    shared['peer_prediction_model']='gaussian_random_walk'
    template=yaml.safe_load((root/'path_following_controller/config/isaac/rpp.yaml').read_text())
    execution=yaml.safe_load((root/'path_following_controller/config/isaac/mpc_execution.yaml').read_text())['/**']['ros__parameters']
    execution.update(shared)
    native=mpc_rpp_parameters(template,execution,'robot_a',['robot_b'])
    rclpy.init();node=rclpy.create_node('gaussian_only_check');latest={};subs=[]
    pubs={t:node.create_publisher(k,t,10) for t,k in [('/clock',Clock),('/tf',TFMessage),
          ('/robot_a/odom',Odometry),('/robot_b/odom',Odometry)]}
    for key,kind in [('peer_state',PeerState),('peer_prediction',RosPath),('mpc_peer_prediction',RosPath),
                     ('mpc_plan',MpcPlan),('mpc_peer_distribution',String),('peer_prediction_distribution',String),
                     ('cmd_vel',Twist),('control_diagnostics',String)]:
        subs.append(node.create_subscription(kind,'/robot_a/'+key,lambda m,k=key:latest.update({k:m}),qos_profile_sensor_data))
    processes=[];logs=[];begin=time.monotonic();peer_speed=-12.;feed_peer=True
    def run(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            assert all(p.poll() is None for p in processes),'Prediction/control process exited'
            ns=round((100.+time.monotonic()-begin)*1e9)
            t=Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000);pubs['/clock'].publish(Clock(clock=t))
            for robot,x,y,v in [('robot_a',-3.,0.,.1),('robot_b',0.,1.3,peer_speed)]:
                if robot=='robot_b' and not feed_peer:continue
                o=Odometry();o.header.frame_id='world';o.header.stamp=t;o.child_frame_id=robot+'/base_link'
                o.pose.pose.position.x=x;o.pose.pose.position.y=y;o.pose.pose.orientation.w=1.;o.twist.twist.linear.x=v
                pubs['/'+robot+'/odom'].publish(o)
                if robot=='robot_a':
                    tf=TransformStamped();tf.header=o.header;tf.child_frame_id=o.child_frame_id
                    tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.rotation.w=1.
                    pubs['/tf'].publish(TFMessage(transforms=[tf]))
            for _ in range(4):rclpy.spin_once(node,timeout_sec=.001)
            time.sleep(.01)
    def checked():
        plan=latest['mpc_plan'];assert plan.valid,plan.status
        for key in ['peer_prediction','mpc_peer_prediction']:
            p=latest[key];assert len(p.poses)==30,key
            assert all(abs(v.pose.position.x)<1e-8 and abs(v.pose.position.y-1.3)<1e-8 for v in p.poses),key
        distribution=json.loads(latest['mpc_peer_distribution'].data)
        assert distribution['model']=='gaussian_random_walk' and len(distribution['steps'])==30
        steps=distribution['steps']
        assert all(a['sigma']<b['sigma'] for a,b in zip(steps,steps[1:]))
        assert all(abs(v['radius']-math.sqrt(-2.*math.log(.05))*v['sigma'])<1e-9 for v in steps)
        diag=json.loads(latest['control_diagnostics'].data)
        assert diag['peer_prediction_model']=='gaussian_random_walk' and diag['clearance_includes_gaussian']
        assert latest['cmd_vel'].linear.x>0.,diag
        assert any(p.pose.position.y<-.1 for p in plan.path.poses), 'No Gaussian clearance deviation'
        for k,p in enumerate(plan.path.poses[1:]):
            distance=math.hypot(p.pose.position.x,p.pose.position.y-1.3)
            assert distance>=1.2+steps[k]['radius']-.01,(k,distance,steps[k]['radius'])
    try:
        with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as f, \
             tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as model_params:
            yaml.safe_dump({'/**':{'ros__parameters':shared}},model_params);model_params.flush()
            model_file=Path(model_params.name)
            yaml.safe_dump(native,f);f.flush()
            commands=[
                ['ros2','run','antbot_mpc_ros','gt_peer_predictor','--ros-args','-r','__ns:=/robot_a',
                 '--params-file',str(model_file),'-p','use_sim_time:=true','-p','ego_robot:=robot_a','-p','peer_robot:=robot_b'],
                ['ros2','run','mpc_planner_antbot','antbot_planner','--ros-args','-r','__ns:=/robot_a',
                 '--params-file',str(model_file),'-p','use_sim_time:=true','-p','recovery_enabled:=false'],
                ['ros2','run','antbot_navigation','mpc_rpp_controller','--ros-args','-r','__ns:=/robot_a',
                 '--params-file',f.name,'-r','peers/robot_b/state:=peer_state']]
            for command in commands:
                log=tempfile.TemporaryFile(mode='w+');logs.append(log)
                processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
            run(6.);checked();assert abs(latest['peer_state'].vx+12.)<1e-8
            print('PASS: real GT/MPC/RPP use Gaussian random walk; both prediction means ignore nonzero velocity',flush=True)
            print('PASS: uncertainty grows; actual OCP avoids Gaussian confidence radii; final guard includes uncertainty',flush=True)
            peer_speed=15.;run(1.);checked();assert abs(latest['peer_state'].vx-15.)<1e-8
            print('PASS: changing only peer velocity preserves stationary forecast means and real state telemetry',flush=True)
            feed_peer=False;run(1.)
            assert not latest['peer_state'].valid and not latest['mpc_plan'].valid
            assert abs(latest['cmd_vel'].linear.x)<1e-9 and abs(latest['cmd_vel'].angular.z)<1e-9
            print('PASS: missing GT still invalidates plans and stops native RPP',flush=True)
    except Exception:
        for log in logs:
            log.seek(0);print(log.read()[-7000:],flush=True)
        raise
    finally:
        for p in processes:
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=4)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        node.destroy_node();rclpy.shutdown()
        for log in logs:log.close()


if __name__=='__main__':main()
