"""Read-only startup barrier: all three robots must publish fresh GT and joints."""
import argparse
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from antbot_interfaces.msg import MpcPlan
from geometry_msgs.msg import Twist
parser=argparse.ArgumentParser();parser.add_argument('--driving',action='store_true');args=parser.parse_args()
rclpy.init();node=rclpy.create_node('three_startup_barrier');seen={};clock=[None];subs=[]
for name in ('robot_a','robot_b','robot_c'):
 specs=[('odom',Odometry),('joint_states',JointState)]
 if args.driving:specs += [('joint_command',JointState),('cmd_vel',Twist),('mpc_plan',MpcPlan)]
 for suffix,kind in specs:
  key=name+'/'+suffix
  subs.append(node.create_subscription(kind,'/'+key,lambda m,k=key:seen.update({k:time.monotonic()}),qos_profile_sensor_data))
subs.append(node.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,time.monotonic()),qos_profile_sensor_data))
try:
 end=time.monotonic()+120.
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.1);now=time.monotonic()
  if len(seen)==(15 if args.driving else 6) and all(now-t<1. for t in seen.values()) and clock[0] and now-clock[0]<1.:
   print('All three robots: '+('MPC, commands and joints ready' if args.driving else 'odom, joint_states and /clock ready'),flush=True);break
 else:raise SystemExit('Timed out waiting for '+('three-robot driving topics' if args.driving else 'three Isaac robots; controllers were not started'))
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
