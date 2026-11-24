"""Wait for all Gazebo robots and verify scenario metadata before driving."""
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from scenarios import layout


def wait_ready(scene, timeout=60.):
    node=Node('gazebo_scene_check',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    samples={};received={};metadata={};futures={};requested={}
    def odom(msg,robot):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        p=msg.pose.pose.position
        if (msg.header.frame_id=='world' and msg.child_frame_id==robot+'/base_link' and
            stamp>0 and all(math.isfinite(v) for v in (p.x,p.y,p.z))):
            previous=samples.get(robot)
            if previous is None or stamp>previous:
                samples[robot]=stamp;received[robot]=time.monotonic()
    clients={}
    for robot in scene.robots:
        node.create_subscription(Odometry,f'/{robot}/odom',lambda msg,r=robot:odom(msg,r),qos_profile_sensor_data)
        clients[robot]=node.create_client(GetParameters,f'/{robot}/gazebo_io/get_parameters')
    deadline=time.monotonic()+timeout
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            for robot,client in clients.items():
                if robot in futures and robot not in metadata and time.monotonic()-requested[robot]>2.:
                    client.remove_pending_request(futures.pop(robot))
                if robot not in futures and client.service_is_ready():
                    request=GetParameters.Request(names=['backend','scenario','robot_count'])
                    futures[robot]=client.call_async(request);requested[robot]=time.monotonic()
                if robot in futures and futures[robot].done() and robot not in metadata:
                    response=futures[robot].result()
                    if response is None or len(response.values)!=3:raise RuntimeError('Missing scene metadata')
                    values=response.values
                    if (values[0].string_value,values[1].string_value,values[2].integer_value)!=('gazebo',scene.scenario,len(scene.robots)):
                        raise RuntimeError('Running Gazebo scene differs from requested scenario/count')
                    metadata[robot]=True
            now=node.get_clock().now().nanoseconds*1e-9
            if all(robot in metadata and robot in samples and time.monotonic()-received[robot]<.35 and
                   0<=now-samples[robot]<.35 for robot in scene.robots):return
        raise RuntimeError('Timed out waiting for fresh Gazebo odometry and matching scene metadata')
    finally:node.destroy_node()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--scenario',choices=list('ABCD'),default='B');p.add_argument('--robots',type=int,choices=[3,4],default=3)
    a=p.parse_args();rclpy.init()
    try:wait_ready(layout(a.scenario,a.robots));print('Gazebo scene ready')
    finally:rclpy.try_shutdown()
