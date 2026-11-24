"""Exercise the real control chain for 5 simulation seconds, then disable it.

Run only with the two-AntBot Isaac scene and two_antbot_driving.launch.py.
Writes measured motion/command evidence to results/control_connection.json.
"""
import json
from pathlib import Path
import time


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    rclpy.init()
    node=rclpy.create_node('isaac_control_connection_check')
    names=('robot_a','robot_b')
    latest, history, subscriptions = {}, [], []
    clock_value=[None]
    subscriptions.append(node.create_subscription(Clock,'/clock',lambda m:clock_value.__setitem__(0,m.clock.sec+m.clock.nanosec*1e-9),10))
    for robot in names:
        for suffix,kind in [('odom',Odometry),('cmd_vel',Twist),('control_diagnostics',String),
                            ('joint_command',JointState),('control_status',String),('mpc_status',String)]:
            topic=f'/{robot}/{suffix}'
            subscriptions.append(node.create_subscription(kind,topic,
                lambda m,t=topic:latest.update({t:m}),rclpy.qos.qos_profile_sensor_data))
    clients={r:node.create_client(SetBool,f'/{r}/mpc_rpp_controller/set_enabled') for r in names}

    def spin(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)

    def set_enabled(value):
        futures=[]
        for client in clients.values():
            if not client.wait_for_service(timeout_sec=2):raise RuntimeError('Native RPP control service unavailable')
            req=SetBool.Request();req.data=value;futures.append(client.call_async(req))
        end=time.monotonic()+3
        while time.monotonic()<end and not all(f.done() for f in futures):spin(.03)
        assert all(f.done() and f.result().success for f in futures),'Enable/disable request failed'

    def sample():
        row={'sim_time':clock_value[0]}
        for r in names:
            odom=latest.get(f'/{r}/odom');cmd=latest.get(f'/{r}/cmd_vel')
            diagnostics=latest.get(f'/{r}/control_diagnostics');raw=json.loads(diagnostics.data) if diagnostics else {};joints=latest.get(f'/{r}/joint_command')
            row[r]=dict(x=odom.pose.pose.position.x,y=odom.pose.pose.position.y,
                        actual_forward=odom.twist.twist.linear.x,
                        cmd_forward=cmd.linear.x if cmd else 0.,rpp_forward=raw.get('rpp_speed',0.),
                        wheel_commands=[joints.velocity[i] for i,n in enumerate(joints.name)
                                        if 'wheel_' in n and i<len(joints.velocity)] if joints else [],
                        control_status=latest[f'/{r}/control_status'].data if f'/{r}/control_status' in latest else '')
        history.append(row)

    result={'verified':False}
    try:
        end=time.monotonic()+10
        while time.monotonic()<end and not all(f'/{r}/odom' in latest for r in names):spin(.05)
        assert clock_value[0] is not None and all(f'/{r}/odom' in latest for r in names),'No Isaac GT'
        set_enabled(False);spin(.3);sample()
        initial=history[-1]
        set_enabled(True)
        start=clock_value[0];end=time.monotonic()+40
        while time.monotonic()<end and clock_value[0]-start<5.:
            spin(.1);sample()
        assert clock_value[0]-start>=5.,'Isaac clock did not advance'
        driven=history[-1]
        set_enabled(False)
        stop_start=clock_value[0];end=time.monotonic()+15
        while time.monotonic()<end and clock_value[0]-stop_start<1.:
            spin(.1);sample()
        metrics={}
        for r in names:
            moved=abs(driven[r]['x']-initial[r]['x'])
            max_command=max(row[r]['cmd_forward'] for row in history)
            max_wheel=max(abs(v) for row in history for v in row[r]['wheel_commands'])
            final=history[-1][r]
            metrics[r]=dict(displacement_m=moved,max_command_m_s=max_command,max_wheel_command_rad_s=max_wheel,
                            final_command_m_s=final['cmd_forward'],final_actual_m_s=final['actual_forward'])
            assert moved>.1 and max_command>.05 and max_wheel>.2,metrics[r]
            assert abs(final['cmd_forward'])<1e-9 and abs(final['actual_forward'])<.02,metrics[r]
            publishers=node.get_publishers_info_by_topic(f'/{r}/cmd_vel')
            assert len(publishers)==1 and publishers[0].node_name=='mpc_rpp_controller'
        result.update(verified=True,metrics=metrics,drive_simulation_seconds=driven['sim_time']-start,
                      final_state='Native RPP controllers disabled after test')
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    finally:
        try:set_enabled(False);spin(.1)
        except Exception:pass
        output=Path('/workspace/results/control_connection.json');output.parent.mkdir(parents=True,exist_ok=True)
        output.write_text(json.dumps(dict(result=result,trace=history),ensure_ascii=False,indent=2))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
