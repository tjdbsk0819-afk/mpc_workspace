"""Print three-robot state and goal progress; no files or control commands."""
import itertools,math,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
rclpy.init();node=rclpy.create_node('three_robot_monitor');latest={};subs=[];robots=('robot_a','robot_b','robot_c')
for robot in robots:
 for suffix,kind,convert in [('odom',Odometry,lambda m:(m.pose.pose.position.x,m.pose.pose.position.y,m.twist.twist.linear.x,m.twist.twist.angular.z)),('cmd_vel',Twist,lambda m:(m.linear.x,m.angular.z)),('control_status',String,lambda m:m.data),('mpc_status',String,lambda m:m.data)]:
  key=robot+'/'+suffix;subs.append(node.create_subscription(kind,'/'+key,lambda m,k=key,c=convert:latest.update({k:c(m)}),qos_profile_sensor_data))
last=0.
try:
 while rclpy.ok():
  rclpy.spin_once(node,timeout_sec=.1)
  if time.monotonic()-last<1.:continue
  last=time.monotonic()
  for robot in robots:print(robot,'pose/v/w=',latest.get(robot+'/odom'),'cmd(v,w)=',latest.get(robot+'/cmd_vel'),'status=',latest.get(robot+'/control_status'),flush=True)
  positions={r:latest[r+'/odom'] for r in robots if r+'/odom'in latest}
  print('center distances:',{a+'/'+b:round(math.dist(positions[a][:2],positions[b][:2]),3) for a,b in itertools.combinations(positions,2)},flush=True)
except KeyboardInterrupt:pass
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
