"""Observe real Isaac motion/joints and the integrated command publishers only."""
import argparse,json,math,time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--robot-count',type=int,choices=(2,3),default=3);args=parser.parse_args()
    robots=('robot_a','robot_b','robot_c')[:args.robot_count]
    rclpy.init();node=rclpy.create_node('native_isaac_connection_observer');latest={};first={};maximum={r:0. for r in robots};nonzero=set();joints=set();subs=[]
    for robot in robots:
        for suffix,kind in [('odom',Odometry),('cmd_vel',Twist),('joint_command',JointState),('control_status',String)]:
            key=robot+'/'+suffix
            subs.append(node.create_subscription(kind,'/'+key,lambda m,k=key:latest.update({k:m}),qos_profile_sensor_data))
    begin=time.monotonic();last=0.
    try:
        while time.monotonic()-begin<130.:
            rclpy.spin_once(node,timeout_sec=.02)
            for r in robots:
                o=latest.get(r+'/odom');c=latest.get(r+'/cmd_vel');j=latest.get(r+'/joint_command')
                if o:
                    xy=(o.pose.pose.position.x,o.pose.pose.position.y);first.setdefault(r,xy)
                    maximum[r]=max(maximum[r],math.dist(first[r],xy))
                if c and max(abs(c.linear.x),abs(c.angular.z))>1e-3:nonzero.add(r)
                if j and any('wheel_' in name and k<len(j.velocity) and abs(j.velocity[k])>.01 for k,name in enumerate(j.name)):joints.add(r)
            if time.monotonic()-last>10.:
                last=time.monotonic();print('WAIT',maximum,'nonzero_cmd',sorted(nonzero),'wheel_commands',sorted(joints),flush=True)
            if all(maximum[r]>.05 for r in robots) and len(nonzero)==len(joints)==len(robots):break
        else:raise AssertionError(('No complete physical command chain',maximum,nonzero,joints))
        for r in robots:
            info=node.get_publishers_info_by_topic('/'+r+'/cmd_vel')
            assert len(info)==1 and info[0].node_name=='mpc_rpp_controller',info
            assert not node.get_publishers_info_by_topic('/'+r+'/rpp_cmd_vel')
        names=node.get_node_names();assert 'mpc_rpp_adapter' not in names and 'controller_server' not in names,names
        print('PASS:',json.dumps(dict(maximum_displacement_m=maximum,nonzero_cmd_robots=sorted(nonzero),wheel_command_robots=sorted(joints),direct_native_publishers=True)),flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
