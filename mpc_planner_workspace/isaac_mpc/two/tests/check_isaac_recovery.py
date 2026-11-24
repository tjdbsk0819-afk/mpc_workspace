"""Drive the live two-AntBot scene through homogeneous MPC recovery and goal arrival.

Requires an initial center separation above 1.15m. For a recovery test, first hold
the scene with native RPP controllers disabled for more than 3 simulation seconds, then run.
All motion commands must come through RPP; this check never starts Isaac itself.

This test moves both robots. It always disables both native RPP controllers at exit and writes
GT/command/state evidence; it does not measure physical contact forces.
"""
import json
import math
from pathlib import Path
import time


def main():
    import rclpy
    from antbot_interfaces.msg import MpcPlan
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('isaac_recovery_check')
    robots = ('robot_a', 'robot_b')
    goals = {'robot_a': (3., 0.), 'robot_b': (-3., 0.)}
    latest, subscriptions, history = {}, [], []
    for r in robots:
        for suffix, kind in [('odom', Odometry), ('cmd_vel', Twist), ('mpc_plan', MpcPlan),
                             ('control_status', String), ('recovery_status', String)]:
            topic = f'/{r}/{suffix}'
            subscriptions.append(node.create_subscription(kind, topic,
                lambda m,t=topic:latest.update({t:(m,time.monotonic())}), rclpy.qos.qos_profile_sensor_data))
    services = {r:node.create_client(SetBool, f'/{r}/mpc_rpp_controller/set_enabled') for r in robots}

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)

    def enabled(value):
        futures = []
        for client in services.values():
            if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Missing control service')
            req = SetBool.Request();req.data = value;futures.append(client.call_async(req))
        end = time.monotonic()+4
        while time.monotonic()<end and not all(f.done() for f in futures):spin(.05)
        assert all(f.done() and f.result().success for f in futures), 'Control service failed'

    result = {'verified':False}
    try:
        end = time.monotonic()+120
        while time.monotonic()<end and not all(f'/{r}/odom' in latest for r in robots):spin(.05)
        assert all(f'/{r}/odom' in latest for r in robots), 'No live GT'
        enabled(False);spin(.3)
        enabled(True)
        end, previous, last_print = time.monotonic()+600, None, 0.
        while time.monotonic()<end:
            spin(.12)
            row = {}
            for r in robots:
                odom, received = latest[f'/{r}/odom']
                assert time.monotonic()-received < 2., 'Live GT stalled'
                p = odom.pose.pose.position
                cmd = latest.get(f'/{r}/cmd_vel', (None,0))[0]
                status = latest.get(f'/{r}/control_status', (None,0))[0]
                recovery = latest.get(f'/{r}/recovery_status', (None,0))[0]
                recovery = json.loads(recovery.data) if recovery else {}
                plan = latest.get(f'/{r}/mpc_plan', (None,0))[0]
                row[r] = dict(x=p.x,y=p.y,sim_time=odom.header.stamp.sec+odom.header.stamp.nanosec*1e-9,
                    goal_distance=math.hypot(p.x-goals[r][0],p.y-goals[r][1]),
                    cmd_x=cmd.linear.x if cmd else 0., cmd_y=cmd.linear.y if cmd else 0.,
                    status=status.data if status else '', phase=recovery.get('phase',''), episode=recovery.get('episode',0),
                    selected=recovery.get('selected',0),
                    mpc_status=plan.status if plan else '')
            row['separation'] = math.hypot(row['robot_a']['x']-row['robot_b']['x'],
                                         row['robot_a']['y']-row['robot_b']['y'])
            history.append(row)
            assert row['separation'] >= 1.14, 'Center clearance below MPC safety margin (1 cm diagnostic tolerance)'
            assert all(abs(row[r]['cmd_y'])<1e-9 for r in robots), 'Recovery must never bypass MPC with lateral Twist'
            phases = tuple(row[r]['phase'] for r in robots)
            if phases != previous or time.monotonic()-last_print>10:
                print(json.dumps(row),flush=True);previous=phases;last_print=time.monotonic()
            if all(row[r]['mpc_status']=='goal_reached' and row[r]['goal_distance']<=.2 for r in robots):
                break
        else:raise AssertionError('Both original goals not reached before timeout')
        phases = {r:{row[r]['phase'] for row in history} for r in robots}
        assert all(any(row[r]['episode']>=1 for row in history) for r in robots), 'Recovery was not exercised for both robots'
        result.update(verified=True,minimum_center_separation_m=min(row['separation'] for row in history),
                      final={r:history[-1][r] for r in robots}, phases={r:sorted(p) for r,p in phases.items()},
                      simulation_seconds=history[-1]['robot_a']['sim_time']-history[0]['robot_a']['sim_time'])
        print('PASS: live recovery, MPC-only recovery and both original goals reached',flush=True)
    finally:
        try:
            enabled(False);spin(.3)
            stopped = {r:dict(x=latest[f'/{r}/cmd_vel'][0].linear.x,
                             y=latest[f'/{r}/cmd_vel'][0].linear.y,
                             yaw=latest[f'/{r}/cmd_vel'][0].angular.z) for r in robots}
            assert all(abs(v)<1e-9 for fields in stopped.values() for v in fields.values())
            result['final_control']='disabled';result['final_commands']=stopped
        except Exception as error:
            result.update(verified=False,final_control='stop_unconfirmed',stop_error=str(error))
        output=Path('/workspace/results/mpc_replanning_isaac.json');output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps({'result':result,'trace':history},ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
