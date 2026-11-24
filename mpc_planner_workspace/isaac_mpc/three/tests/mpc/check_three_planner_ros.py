"""Real two-obstacle acados OCP, isolated ROS domain 229; no motion commands."""
import json,math,os,signal,subprocess,tempfile,time
from pathlib import Path

def main():
 if os.environ.get('ROS_DOMAIN_ID')!='229':raise RuntimeError('Use domain 229')
 import rclpy
 from antbot_interfaces.msg import MpcPlan,PeerState
 from builtin_interfaces.msg import Time
 from nav_msgs.msg import Odometry,Path as RosPath
 from rosgraph_msgs.msg import Clock
 from std_msgs.msg import String
 solver=json.loads((Path(__file__).resolve().parents[2]/'mpc_planning/mpc_planner_solver/acados/Solver/Solver.json').read_text())
 assert solver['dims']['nh']==4 and solver['dims']['nh_0']==2
 rclpy.init();node=rclpy.create_node('three_ocp_check');clock=node.create_publisher(Clock,'/clock',10)
 odom=node.create_publisher(Odometry,'/robot_a/odom',10)
 pubs={p:node.create_publisher(PeerState,f'/robot_a/peers/{p}/state',10) for p in ('robot_b','robot_c')}
 latest={};subs=[]
 for suffix,kind in [('mpc_plan',MpcPlan),('mpc_status',String),('mpc_peer_prediction/robot_b',RosPath),('mpc_peer_prediction/robot_c',RosPath)]:
  subs.append(node.create_subscription(kind,'/robot_a/'+suffix,lambda m,k=suffix:latest.update({k:m}),10))
 log=tempfile.TemporaryFile(mode='w+');process=None;begin=time.monotonic()
 def run(duration,mode='normal'):
  end=time.monotonic()+duration
  while time.monotonic()<end:
   assert process.poll() is None,'Planner exited'
   ns=round((100.+time.monotonic()-begin)*1e9);t=Time(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
   clock.publish(Clock(clock=t));o=Odometry();o.header.stamp=t;o.header.frame_id='world';o.child_frame_id='robot_a/base_link'
   o.pose.pose.position.x=-3.;o.pose.pose.orientation.w=1.;odom.publish(o)
   for name,pub in pubs.items():
    if name=='robot_c' and mode=='missing':continue
    p=PeerState();p.header=o.header;p.robot_id=name;p.valid=not(name=='robot_c' and mode=='invalid');p.status='ok'
    p.x,p.y=(8.,5.) if name=='robot_b' else ((-2.1,0.) if mode=='close' else (-1.7,0.))
    pub.publish(p)
   rclpy.spin_once(node,timeout_sec=.005);time.sleep(.01)
 try:
  process=subprocess.Popen(['ros2','run','mpc_planner_antbot','antbot_planner','--ros-args','-r','__ns:=/robot_a','-p','use_sim_time:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  run(1.5,'missing')
  assert 'mpc_plan'in latest and not latest['mpc_plan'].valid,'Missing second peer must block planning'
  run(3.)
  p=latest['mpc_plan'];assert p.valid and len(p.path.poses)==30,(p.valid,p.status)
  for peer in pubs:assert len(latest['mpc_peer_prediction/'+peer].poses)==30
  minimum=min(math.hypot(x.pose.position.x+1.7,x.pose.position.y) for x in p.path.poses)
  assert minimum>=1.15,minimum
  print('PASS: real two-obstacle OCP publishes both CV predictions and avoids the second neighbor',flush=True)
  run(.4,'close');assert not latest['mpc_plan'].valid and latest['mpc_plan'].status=='insufficient_current_clearance'
  print('PASS: unsafe current clearance to second neighbor blocks the optimized path',flush=True)
  run(.6,'invalid');assert not latest['mpc_plan'].valid
  run(.8);assert latest['mpc_plan'].valid
  print('PASS: missing/invalid second peer gates planning and fresh complete inputs restore it',flush=True)
 except Exception:
  log.seek(0);print(log.read()[-8000:],flush=True);raise
 finally:
  if process and process.poll() is None:
   os.killpg(process.pid,signal.SIGINT)
   try:process.wait(timeout=3)
   except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
  node.destroy_node();rclpy.shutdown();log.close()
if __name__=='__main__':main()
