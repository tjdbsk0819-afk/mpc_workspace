"""Run until user stop and stream the selected robot's solver/planning timings."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import threading
import yaml
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from reference import load_reference
from ready import wait_ready
from result_store import TimingArchive, ArrivalGate
from timing_rates import planning_rates, format_rates
from scenarios import layout


def distribution(values, final=False):
    if not values:
        return None
    result = dict(mean=sum(values)/len(values), max=max(values), last=values[-1])
    if final:
        ordered = sorted(values)
        def quantile(fraction):
            index=(len(ordered)-1)*fraction; lo=int(index); hi=min(lo+1,len(ordered)-1)
            return ordered[lo]+(index-lo)*(ordered[hi]-ordered[lo])
        result.update(median=quantile(.5), p95=quantile(.95))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',choices=list('ABCD'),required=True)
    parser.add_argument('--ego',choices=[f'robot_{r}' for r in 'abcd'],default='robot_a')
    parser.add_argument('--reference',required=True)
    parser.add_argument('--timeout',type=float,help=argparse.SUPPRESS)  # Old commands remain accepted; ignored.
    args=parser.parse_args(); load_reference(args.reference,args.scenario)
    backend=os.environ['ANTBOT_PLANNER']; root=Path('/workspace'); gazebo=root/'gazebo'
    if backend=='dmpc':
        launch=gazebo/'drive.launch.py'; config_path=root/'isaac/controller_bringup/config/dmpc.yaml'
        config=yaml.safe_load(config_path.read_text())['/**']['ros__parameters']
        horizon,dt=config['horizon'],config['dt']
    else:
        launch=gazebo/'launch/drive.launch.py'; config_path=root/'mpc_planning/mpc_planner_antbot/config/settings.yaml'
        config=yaml.safe_load(config_path.read_text()); horizon,dt=config['N'],config['integrator_step']
    archive=TimingArchive(gazebo/'results','mpc_planner' if backend=='mpc' else 'dmpc',
                          horizon*dt,args.scenario,args.ego,config_path.read_text())
    reference_hash=hashlib.sha256(Path(args.reference).read_bytes()).hexdigest()
    stopping=False
    def stop_requested(signum, frame):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop_requested); signal.signal(signal.SIGTERM,stop_requested)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    # Wait for the simulator before hardware activation; retry without an overall deadline.
    while not stopping:
        try:
            wait_ready(layout(args.scenario,4),timeout=5.)
            break
        except RuntimeError:
            continue
    if stopping:
        rclpy.try_shutdown()
        return 0
    node=Node('single_mpc_measurement',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    values={'solver_ms':[],'planning_ms':[]}; status={}; started=time.monotonic()
    samples=[]; gate=ArrivalGate(); saved=False; arrival_wall=None; report=None
    goal=layout(args.scenario,4).robots['robot_a']['goal']
    def timing(msg):
        try:
            record=json.loads(msg.data)
            epoch=status.get('ego_enabled_stamp')
            if saved or record.get('robot')!=args.ego or epoch is None or record['stamp']<epoch:
                return
            if gate.arrival_stamp is not None and record['stamp']>gate.arrival_stamp:
                return
            sample={key:record[key] for key in ('stamp','solver_ms','planning_ms')}
            for key in ('start_wall_sec','completion_wall_sec','accepted'):
                if key in record:sample[key]=record[key]
            for key in values: values[key].append(sample[key])
            samples.append(sample)
        except (ValueError,TypeError,KeyError):
            pass
    def reference(msg):
        try:
            status.clear();status.update(json.loads(msg.data))
        except ValueError:
            pass
    def on_a_odometry(msg):
        nonlocal arrival_wall
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        epoch=status.get('ego_enabled_stamp')
        if epoch is None or stamp<epoch or msg.header.frame_id!='world':return
        p=msg.pose.pose.position;v=msg.twist.twist.linear
        if gate.observe(math.hypot(p.x-goal[0],p.y-goal[1]),math.hypot(v.x,v.y),stamp) and arrival_wall is None:
            arrival_wall=time.monotonic()
            samples[:]=[row for row in samples if row['stamp']<=gate.arrival_stamp]
            for key in values:values[key][:]=[row[key] for row in samples]
    node.create_subscription(Odometry,'/robot_a/odom',on_a_odometry,qos_profile_sensor_data)
    node.create_subscription(String,f'/{args.ego}/planning_timing',timing,100)
    node.create_subscription(String,'/fixed_reference/status',reference,10)
    process=subprocess.Popen(['ros2','launch',str(launch),f'scenario:={args.scenario}','robots:=4',
                              f'single_ego:={args.ego}',f'reference:={args.reference}'],
                             stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True,text=True)
    def show_launch_errors():
        seen=set()
        for line in process.stdout:
            if any(marker in line for marker in ('[ERROR]','Traceback','Error:')):
                text=line.strip()
                if text not in seen:
                    print('Launch diagnostic: '+text,flush=True);seen.add(text)
    threading.Thread(target=show_launch_errors,daemon=True).start()
    def save(outcome,final=False):
        report=dict(backend=backend,scenario=args.scenario,ego=args.ego,ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),
                    prediction_seconds=horizon*dt,horizon_steps=horizon,dt=dt,
                    reference_sha256=reference_hash,
                    outcome=outcome,automatic_stop=False,elapsed_wall_seconds=time.monotonic()-started,
                    samples=len(values['solver_ms']),solver_ms=distribution(values['solver_ms'],final),
                    planning_ms=distribution(values['planning_ms'],final),
                    planning_hz=planning_rates(samples,config['control_frequency']),
                    measurement_start_sim=status.get('ego_enabled_stamp'),
                    measurement_end_sim=gate.arrival_stamp,
                    note='Wall-clock time per replan; solver sums its calls. Failed replans included. Startup excluded. No synthetic samples when the planner is idle.')
        archive.save(report,samples)
        return report
    print(f'Saved only when robot_a arrives and stops; simulation runs until Ctrl+C. Timing output: {archive.path}',flush=True)
    last_print=-float('inf'); outcome='running'
    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node,timeout_sec=.05)
            if process.poll() is not None:
                outcome='launch_exited';break
            if not saved and arrival_wall is not None and time.monotonic()-arrival_wall>=.5:
                report=save('robot_a_arrived',final=True);saved=True
                print(f'robot_a arrived: saved {len(samples)} samples to {archive.path}; simulation continues.',flush=True)
            if time.monotonic()-last_print>=2:
                if values['solver_ms']:
                    parts=[f"{key}={v[-1]:.1f}ms (mean {sum(v)/len(v):.1f}, max {max(v):.1f})" for key,v in values.items()]
                    print(f"{args.ego}: "+'; '.join(parts)+f"; samples={len(values['solver_ms'])}",flush=True)
                else:
                    print(f"{args.ego}: waiting for optimization samples; {status.get('phase','launching')}",flush=True)
                print(format_rates(planning_rates(samples,config['control_frequency'])),flush=True)
                if saved: print('Arrival result already saved; no further timing samples are added.',flush=True)
                last_print=time.monotonic()
        if stopping: outcome='user_stopped'
    finally:
        # Never persist an incomplete journey. Keep the arrival snapshot frozen.
        if gate.arrival_stamp is not None and not saved:
            report=save('robot_a_arrived',final=True);saved=True
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
        node.destroy_node();rclpy.try_shutdown()
        if saved:
            print(json.dumps(report,indent=2),flush=True);print(f'Saved: {archive.path}',flush=True)
        else:
            print('robot_a did not arrive; this run was not saved.',flush=True)
    return 0 if outcome=='user_stopped' else 1


if __name__=='__main__':
    raise SystemExit(main())
