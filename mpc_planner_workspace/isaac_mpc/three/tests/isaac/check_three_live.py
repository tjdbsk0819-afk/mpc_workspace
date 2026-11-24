"""Read-only physical test observer. Writes only a terminal summary, never commands."""
import itertools,json,math,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from antbot_interfaces.msg import PeerState
from rosgraph_msgs.msg import Clock

rclpy.init();node=rclpy.create_node('three_physical_observer');robots=('robot_a','robot_b','robot_c');latest={};received={};subs=[];sim=None
minimum=math.inf;motion_start=None;goals={};begin=time.monotonic();last_print=0.
def receive(key,value):latest[key]=value;received[key]=received.get(key,0)+1
def clock_cb(m):
 global sim
 sim=m.clock.sec+m.clock.nanosec*1e-9
for robot in robots:
 for suffix,kind,convert in [('odom',Odometry,lambda m:dict(t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,x=m.pose.pose.position.x,y=m.pose.pose.position.y,v=m.twist.twist.linear.x)),('cmd_vel',Twist,lambda m:[m.linear.x,m.angular.z]),('control_status',String,lambda m:m.data)]:
  key=robot+'/'+suffix;subs.append(node.create_subscription(kind,'/'+key,lambda m,k=key,c=convert:receive(k,c(m)),qos_profile_sensor_data))
 for peer in robots:
  if peer==robot:continue
  key=f'{robot}/peers/{peer}/state';subs.append(node.create_subscription(PeerState,'/'+key,lambda m,k=key:receive(k,m.valid),qos_profile_sensor_data))
subs.append(node.create_subscription(Clock,'/clock',clock_cb,qos_profile_sensor_data))
try:
 while time.monotonic()-begin<300.:
  rclpy.spin_once(node,timeout_sec=.01)
  if sim is None:continue
  if motion_start is None and any(max(abs(x) for x in latest.get(r+'/cmd_vel',[0.,0.]))>1e-4 for r in robots):motion_start=sim
  for robot in robots:
   if latest.get(robot+'/control_status')=='mpc_goal_reached' and robot not in goals:goals[robot]=sim
  for a,b in itertools.combinations(robots,2):
   p,q=latest.get(a+'/odom'),latest.get(b+'/odom')
   if p and q and abs(p['t']-q['t'])<=.011:minimum=min(minimum,math.hypot(p['x']-q['x'],p['y']-q['y']))
  if time.monotonic()-last_print>10.:
   last_print=time.monotonic();print('LIVE',sim,{r:latest.get(r+'/control_status') for r in robots},{r:latest.get(r+'/odom') for r in robots},flush=True)
  if len(goals)==3:break
  if motion_start is not None and sim-motion_start>110.:break
 pairs=[f'{a}/peers/{b}/state' for a in robots for b in robots if a!=b]
 summary=dict(all_three_goals=len(goals)==3,goals=goals,first_motion=motion_start,minimum_sampled_center_distance=minimum if math.isfinite(minimum) else None,peer_channels={k:received.get(k,0) for k in pairs},final={r:latest.get(r+'/odom') for r in robots},status={r:latest.get(r+'/control_status') for r in robots})
 print('SUMMARY',json.dumps(summary),flush=True)
 assert all(received.get(k,0)>0 for k in pairs),'All six peer channels required'
 assert minimum>=1.15-1e-6,'Sampled clearance below configured nominal'
 assert len(goals)==3,'Not all three robots reached goals during observation'
finally:node.destroy_node();rclpy.shutdown()
