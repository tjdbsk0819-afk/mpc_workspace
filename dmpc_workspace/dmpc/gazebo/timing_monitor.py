"""Display each robot's cumulative planning timings; does not start driving."""
import argparse
from collections import deque
from pathlib import Path
import sys
import json
import math
import time

sys.path.insert(0,str(Path(__file__).resolve().parent/'single_mpc'))
from timing_rates import planning_rates,format_rates

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robots',type=int,choices=[3,4],default=3)
    parser.add_argument('--period',type=float,default=2.)
    args=parser.parse_args()
    if not math.isfinite(args.period) or args.period<=0:
        parser.error('--period must be positive and finite')
    rclpy.init()
    node=rclpy.create_node('planning_timing_monitor')
    records={}
    rate_samples={}
    statuses={}
    timing_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def receive(msg,robot):
        try:
            value=json.loads(msg.data)
            if value.get('robot')==robot:
                records[robot]=(value,time.monotonic())
                rate_samples.setdefault(robot,deque(maxlen=1000)).append(value)
        except (ValueError,TypeError):pass
    robots=[f'robot_{chr(97+i)}' for i in range(args.robots)]
    for robot in robots:
        node.create_subscription(String,f'/{robot}/planning_timing',lambda msg,r=robot:receive(msg,r),timing_qos)
        node.create_subscription(String,f'/{robot}/mpc_status',lambda msg,r=robot:statuses.__setitem__(r,msg.data),1)
    print('Wall-clock milliseconds. solver=sum of all solver calls in one replan.',flush=True)
    print('planning=request to result handling, including validation; >100ms misses 10Hz budget.',flush=True)
    deadline=time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.1)
            if time.monotonic()<deadline:continue
            for robot in robots:
                if robot not in records:
                    publishers=node.count_publishers(f'/{robot}/planning_timing')
                    if publishers==0:
                        reason='no timing publisher: rebuild/restart planner; check workspace/ROS_DOMAIN_ID'
                    else:
                        reason=f"no optimization sample yet; planner={statuses.get(robot,'status not received')}"
                    print(f'{robot}: {reason}',flush=True)
                    continue
                v,received=records[robot]
                stale=' [stale]' if time.monotonic()-received>max(2.,2*args.period) else ''
                print(f"{robot} {v['planner']}{stale}: "
                      f"solver={v['solver_ms']:.1f}ms (mean {v['mean_solver_ms']:.1f}, max {v['max_solver_ms']:.1f}); "
                      f"planning={v['planning_ms']:.1f}ms (mean {v['mean_planning_ms']:.1f}, max {v['max_planning_ms']:.1f}); "
                      f"over {v['deadline_ms']:.0f}ms={v['deadline_misses']}/{v['samples']}; "
                      f"accepted={v['accepted_samples']}/{v['samples']}; "
                      f"attempts={v['attempts']}; {v['outcome']}",flush=True)
                print(format_rates(planning_rates(list(rate_samples.get(robot,[])),1000./v['deadline_ms'])),flush=True)
            deadline=time.monotonic()+args.period
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
