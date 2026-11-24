"""Foreground multi-robot driving test, with timestamp-aligned all-pairs metrics."""
import argparse
from collections import deque
from datetime import datetime, timezone
import itertools
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from scenarios import layout
from ready import wait_ready


def main():
    p=argparse.ArgumentParser();p.add_argument('--scenario',choices=list('ABCD'),default='B')
    p.add_argument('--robots',type=int,choices=[3,4],default=3);p.add_argument('--timeout',type=float,default=120.)
    a=p.parse_args();scene=layout(a.scenario,a.robots);rclpy.init();wait_ready(scene)
    node=Node('gazebo_test',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    timings={}
    states={};history={robot:deque(maxlen=32) for robot in scene.robots};statuses={};received={}
    minima={f'{first}:{second}':None for first,second in itertools.combinations(scene.robots,2)}
    samples={key:0 for key in minima};invalid=0
    def odom(msg,robot):
        nonlocal invalid
        t=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        pos=msg.pose.pose.position;v=msg.twist.twist.linear
        q=msg.pose.pose.orientation
        values=(t,pos.x,pos.y,v.x,v.y,q.x,q.y,q.z,q.w)
        if msg.header.frame_id!='world' or msg.child_frame_id!=robot+'/base_link' or not all(math.isfinite(x) for x in values):
            invalid+=1;return
        if robot in states and t<=states[robot]['stamp']:return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        state=dict(stamp=t,x=pos.x,y=pos.y,yaw=yaw,speed=math.hypot(v.x,v.y))
        states[robot]=state;received[robot]=time.monotonic();history[robot].append(state)
        for peer in scene.robots:
            if peer==robot:continue
            matches=[other for other in history[peer] if abs(t-other['stamp'])<=.011]
            if not matches:continue
            other=min(matches,key=lambda item:abs(t-item['stamp']))
            key=':'.join(sorted((robot,peer)));distance=math.hypot(pos.x-other['x'],pos.y-other['y'])
            minima[key]=distance if minima[key] is None else min(minima[key],distance);samples[key]+=1
    def timing(msg,robot):
        try:
            record=json.loads(msg.data)
            if record.get('robot')==robot:timings[robot]=record
        except (ValueError,TypeError):pass
    for robot in scene.robots:
        node.create_subscription(String,f'/{robot}/planning_timing',lambda msg,r=robot:timing(msg,r),10)
        node.create_subscription(Odometry,f'/{robot}/odom',lambda msg,r=robot:odom(msg,r),qos_profile_sensor_data)
        node.create_subscription(String,f'/{robot}/mpc_status',lambda msg,r=robot:statuses.__setitem__(r,msg.data),1)
    # Test owns drive; stop any separately started drive before using this command.
    deadline=time.monotonic()+5.
    while len(states)<a.robots and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
    for robot,config in scene.robots.items():
        state=states.get(robot);start=config['start']
        if (state is None or math.hypot(state['x']-start[0],state['y']-start[1])>.15 or
            abs(math.remainder(state['yaw']-start[2],2*math.pi))>.15):
            node.destroy_node();rclpy.try_shutdown();raise RuntimeError('Restart sim: initial pose mismatch')
    command=['ros2','launch','/workspace/gazebo/launch/drive.launch.py',f'scenario:={a.scenario}',f'robots:={a.robots}']
    output=Path('/workspace/gazebo/results');output.mkdir(parents=True,exist_ok=True)
    tag=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    log=(output/f'{tag}_{a.scenario}_{a.robots}.log').open('w')
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    started=time.monotonic();dwell=None;complete=False
    try:
        while time.monotonic()-started<a.timeout and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.05)
            reached=[]
            for robot,config in scene.robots.items():
                state=states.get(robot);goal=config['goal']
                reached.append(state is not None and time.monotonic()-received[robot]<.35 and
                               math.hypot(state['x']-goal[0],state['y']-goal[1])<=.2 and state['speed']<=.05)
            if all(reached):
                dwell=dwell or time.monotonic()
                if time.monotonic()-dwell>=.5:complete=True;break
            else:dwell=None
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
        log.close();node.destroy_node();rclpy.try_shutdown()
    separation_ok=all(distance is not None and distance>=1.15-.001 for distance in minima.values())
    report=dict(backend='gazebo',planner=os.environ['ANTBOT_PLANNER'],scenario=a.scenario,robot_count=a.robots,
                elapsed_wall_seconds=time.monotonic()-started,all_goals_reached=complete,
                all_pairs_separation_ok=separation_ok,pair_minimum_center_distances=minima,
                pair_synchronized_samples=samples,final_states=states,final_statuses=statuses,invalid_odometry=invalid,
                planning_timing=timings,
                result='passed' if complete and separation_ok and not invalid else 'not_passed',
                note='C preserves the original narrow corridor; deadlock is an expected possible outcome. Metrics use center distances, not Gazebo contact sensors.')
    target=output/f'{tag}_{a.scenario}_{a.robots}.json';target.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    return 0 if report['result']=='passed' else 1

if __name__=='__main__':raise SystemExit(main())
