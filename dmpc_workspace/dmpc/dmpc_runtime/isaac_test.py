"""Run a foreground Isaac driving test and append results to one Markdown file."""
import argparse
from collections import Counter, deque
import fcntl
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
from zoneinfo import ZoneInfo

from simulation.helpers.scenario_config import (
    scenario_layout, WALL_ROBOT_RADIUS, WALL_SAFETY_MARGIN, STATIC_SAFETY_MARGIN,
)

ROOT = Path(__file__).resolve().parents[1]
ROBOTS = ('robot_a', 'robot_b')
BEGIN = '<!-- ISAAC_TEST_RUNS_BEGIN -->'
END = '<!-- ISAAC_TEST_RUNS_END -->'


class Measurements:
    """Compute geometry only from nearly simultaneous, fresh world odometry."""
    def __init__(self, scenario, config):
        self.layout = scenario_layout(scenario)
        self.config = config
        self.odom = {}
        self.max_speed = [0., 0.]
        self.max_command_speed = [0., 0.]
        self.valid_plans = [0, 0]
        self.statuses = {r: {'planner': Counter(), 'controller': Counter()} for r in ROBOTS}
        self.minimum_separation = None
        self.minimum_wall_margin = None
        self.minimum_obstacle_margin = None
        self.first_stamp = self.last_stamp = None
        self.clock_reset = False
        self.pairs = 0
        self.invalid_odometry = 0

    def observe(self, index, stamp, x, y, yaw, speed, received):
        if not all(math.isfinite(v) for v in (stamp, x, y, yaw, speed)) or stamp < 0:
            self.invalid_odometry += 1
            return
        previous = self.odom.get(index)
        if previous and stamp < previous['stamp']-.01:
            self.clock_reset = True
            return
        if previous and stamp <= previous['stamp']:
            return
        self.odom[index] = dict(stamp=stamp, x=x, y=y, yaw=yaw, speed=speed, received=received)
        self.max_speed[index] = max(self.max_speed[index], speed)
        if len(self.odom) != 2:
            return
        a, b = self.odom[0], self.odom[1]
        # Pair each timestamp once; callback order must not affect the metric.
        if abs(a['stamp']-b['stamp']) > 1e-6 or received-min(a['received'], b['received']) > .35:
            return
        self.pairs += 1
        self.last_stamp = a['stamp']
        if self.first_stamp is None:
            self.first_stamp = a['stamp']
        distance = math.hypot(a['x']-b['x'], a['y']-b['y'])
        self.minimum_separation = distance if self.minimum_separation is None else min(self.minimum_separation, distance)
        for state in (a, b):
            if self.layout.bounds:
                xmin, xmax, ymin, ymax = self.layout.bounds
                margin = min(state['x']-xmin, xmax-state['x'], state['y']-ymin, ymax-state['y'])
                margin -= WALL_ROBOT_RADIUS+WALL_SAFETY_MARGIN
                self.minimum_wall_margin = margin if self.minimum_wall_margin is None else min(self.minimum_wall_margin, margin)
            for x, y, radius in self.layout.obstacles:
                margin = math.hypot(state['x']-x, state['y']-y)-(radius+WALL_ROBOT_RADIUS+STATIC_SAFETY_MARGIN)
                self.minimum_obstacle_margin = margin if self.minimum_obstacle_margin is None else min(self.minimum_obstacle_margin, margin)

    def at_start(self, now):
        if len(self.odom) != 2:
            return False
        for i, pose in enumerate(self.layout.starts):
            state = self.odom[i]
            yaw_error = math.atan2(math.sin(state['yaw']-pose[2]), math.cos(state['yaw']-pose[2]))
            if (now-state['received'] > .35 or state['speed'] > .05 or
                    math.hypot(state['x']-pose[0], state['y']-pose[1]) > .25 or abs(yaw_error) > .2):
                return False
        return True

    def reached(self, now):
        if len(self.odom) != 2:
            return [False, False]
        return [now-self.odom[i]['received'] <= .35 and self.odom[i]['speed'] <= .05 and
                math.hypot(self.odom[i]['x']-goal[0], self.odom[i]['y']-goal[1]) <= self.config['goal_tolerance']
                for i, goal in enumerate(self.layout.goals)]

    def progress(self, now):
        """Expose the simultaneous distance, speed and freshness arrival checks."""
        arrived = self.reached(now)
        rows = []
        for i, goal in enumerate(self.layout.goals):
            state = self.odom.get(i)
            distance = math.hypot(state['x']-goal[0], state['y']-goal[1]) if state else None
            speed = state['speed'] if state else None
            age = max(0., now-state['received']) if state else None
            rows.append(dict(robot=ROBOTS[i], goal_error_m=distance, speed_mps=speed,
                             odom_age_s=age, reached=arrived[i]))
        return rows

    def summary(self, reason, now):
        distances, yaw_errors = [], []
        for i, goal in enumerate(self.layout.goals):
            state = self.odom.get(i)
            distances.append(math.hypot(state['x']-goal[0], state['y']-goal[1]) if state else None)
            yaw_errors.append(abs(math.atan2(math.sin(state['yaw']-goal[2]), math.cos(state['yaw']-goal[2]))) if state else None)
        checks = []
        if self.minimum_separation is not None and self.minimum_separation < self.config['separation']-.001:
            checks.append('로봇 간 최소 간격 미달')
        if self.minimum_wall_margin is not None and self.minimum_wall_margin < -.001:
            checks.append('벽 안전 범위 이탈')
        if self.minimum_obstacle_margin is not None and self.minimum_obstacle_margin < -.001:
            checks.append('고정 장애물 안전 범위 침범')
        if max(self.max_speed) > self.config['max_speed']+.05:
            checks.append('실측 속도 제한 초과(허용 오차 0.05 m/s)')
        if self.clock_reset:
            checks.append('실험 중 시뮬레이션 시간 초기화')
        assessment = ('관측 기준 위반' if checks else
                      '측정 자료 부족' if not self.pairs else
                      '위치 도착(접촉·방향 별도)' if reason == '두 로봇 목표 도착' and all(self.reached(now)) else '미완료')
        return dict(reason=reason, assessment=assessment, reached=self.reached(now), final_goal_distances=distances,
                    final_positions=[[self.odom[i]['x'], self.odom[i]['y'], self.odom[i]['yaw']]
                                     if i in self.odom else None for i in range(2)],
                    final_yaw_errors=yaw_errors,
                    maximum_measured_speed=[self.max_speed[i] if i in self.odom else None for i in range(2)],
                    maximum_commanded_speed=list(self.max_command_speed), valid_plan_counts=list(self.valid_plans),
                    minimum_separation=self.minimum_separation, minimum_wall_margin=self.minimum_wall_margin,
                    minimum_obstacle_margin=self.minimum_obstacle_margin, synchronized_sample_pairs=self.pairs,
                    invalid_odometry=self.invalid_odometry,
                    elapsed_sim_time=None if self.first_stamp is None else self.last_stamp-self.first_stamp,
                    violations=checks, statuses={robot: {kind: dict(counts) for kind, counts in groups.items()}
                                                 for robot, groups in self.statuses.items()})


def number(value):
    return '—' if value is None else f'{value:.3f}'


class ProcessOutput:
    """Drain launch output into bounded memory; never create a log file."""
    def __init__(self, process):
        self.tail = deque(maxlen=40)
        self.errors = deque(maxlen=20)
        self.error_count = 0
        self.thread = threading.Thread(target=self._read, args=(process.stdout,), daemon=True)
        self.thread.start()

    def _read(self, stream):
        for raw in stream:
            line = re.sub(r'\x1b\[[0-9;]*m', '', raw.rstrip())
            self.tail.append(line)
            if 'MPC solve failed:' in line:
                print(line, flush=True)
            if re.search(r'process has died .*exit code -?\d+', line):
                self.errors.append(line)
                self.error_count += 1
        stream.close()

    def finish(self):
        self.thread.join(timeout=5.)
        errors = list(self.errors)
        if self.thread.is_alive():
            errors.append('주행 출력 스트림 종료 시간 초과')
        return dict(process_errors=errors, process_error_count=self.error_count, output_tail=list(self.tail))


def save_result(result, root=ROOT):
    """Append a summary row and details, preserving history in the same MD file."""
    path = root/'docs/SCENARIO_TEST_RESULTS.md'
    path.parent.mkdir(parents=True, exist_ok=True)
    run_id = result.get('run_id') or re.sub(r'[^0-9]', '', result['started_kst'])+'__'+result['scenario']
    anchor = 'run-'+run_id
    goals = '/'.join('도착' if v else '미도착' for v in result['reached'])
    speed = max((v for v in result['maximum_measured_speed'] if v is not None), default=None)
    cell = lambda value: str(value).replace('|', '\\|').replace('\n', ' ')
    row = (f"| {result['started_kst']} | {result['scenario']} | {cell(result['assessment'])} | "
           f"{cell(result['reason'])} | {goals} | {number(result['minimum_separation'])} | "
           f"{number(speed)} | [상세](#{anchor}) |")
    lines = [f'<a id="{anchor}"></a>', '', f"### 시나리오 {result['scenario']} — {result['started_kst']} KST", '',
             f"- 관측 판정: {result['assessment']}", f"- 종료 사유: {result['reason']}", f"- 설정 SHA-256: `{result['config_sha256']}`",
             f"- 속도 제한: {result['config']['max_speed']} m/s, 가속도: {result['config']['max_acceleration']} m/s²",
             f"- 시뮬레이션 관측 시간: {number(result['elapsed_sim_time'])} s",
             f"- 시각이 일치하는 odom 쌍: {result['synchronized_sample_pairs']}",
             f"- 관측된 최소 중심 간격: {number(result['minimum_separation'])} m (기준 {result['config']['separation']} m)",
             f"- 벽 안전 범위 최소 여유: {number(result['minimum_wall_margin'])} m",
             f"- 고정 장애물 안전 범위 최소 여유: {number(result['minimum_obstacle_margin'])} m", '',
             '| 로봇 | 위치·정지 기준 도착 | 최종 목표 거리(m) | 방향 오차(rad) | 최대 실측 속도(m/s) | 최대 명령 속도(m/s) | 유효 계획 수 |',
             '|---|---|---|---|---|---|---|']
    for i, robot in enumerate(ROBOTS):
        lines.append(f"| {robot} | {'도착' if result['reached'][i] else '미도착'} | "
                     f"{number(result['final_goal_distances'][i])} | {number(result['final_yaw_errors'][i])} | "
                     f"{number(result['maximum_measured_speed'][i])} | {result['maximum_commanded_speed'][i]:.3f} | {result['valid_plan_counts'][i]} |")
    lines += ['', '관측된 기준 위반: '+(' / '.join(result['violations']) or '없음'), '',
              '종료 정리 오류: '+(' / '.join(result.get('shutdown_errors', [])) or '없음'), '',
              '자식 프로세스 오류: '+(' / '.join(result.get('process_errors', [])) or '없음'), '',
              '<details><summary>실행 설정·상태 집계</summary>', '', '```json',
              json.dumps({key: result[key] for key in ('config', 'source_sha256', 'statuses', 'final_positions',
                                                       'final_control_diagnostics', 'final_joint_feedback', 'final_plans')
                          if key in result}, ensure_ascii=False, indent=2, allow_nan=False), '```', '', '</details>', '']
    if result.get('output_tail') and (result['reason'] != '두 로봇 목표 도착' or result.get('process_errors')):
        lines += ['<details><summary>마지막 실행 메시지</summary>', '', '```text',
                  *result['output_tail'], '```', '', '</details>', '']
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    with os.fdopen(descriptor, 'r+', encoding='utf-8') as stream:
        fcntl.flock(stream, fcntl.LOCK_EX)
        stream.seek(0)
        document = stream.read()
        if not document:
            document = '# 시나리오 테스트 결과\n\n'+BEGIN+'\n'+END+'\n\n## 실행별 상세 기록\n'
        if f'<a id="{anchor}"></a>' in document:
            return path
        before, rest = document.split(BEGIN, 1)
        table, after = rest.split(END, 1)
        if '| 시작 시각(KST)' not in table:
            table = ('\n\n| 시작 시각(KST) | 시나리오 | 관측 판정 | 종료 사유 | A/B 도착 | 최소 간격(m) | 최대 실측 속도(m/s) | 기록 |\n'
                     '|---|---|---|---|---|---|---|---|')
        table = table.rstrip()+'\n'+row+'\n\n'
        document = before+BEGIN+table+END+after.rstrip()+'\n\n'+'\n'.join(lines)
        stream.seek(0)
        stream.write(document)
        stream.truncate()
        stream.flush()
        os.fsync(stream.fileno())
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=list('ABCD'))
    parser.add_argument('--timeout', type=float, default=120., help='Maximum wall seconds after starting the driving launch')
    parser.add_argument('--ready-timeout', type=float, default=30., help='Wait for matching Isaac scene at its initial poses')
    args = parser.parse_args()
    if not all(math.isfinite(v) and v > 0 for v in (args.timeout, args.ready_timeout)):
        parser.error('Timeouts must be positive and finite')
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rcl_interfaces.srv import GetParameters
    from lifecycle_msgs.srv import ChangeState
    from lifecycle_msgs.msg import Transition
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    from antbot_interfaces.msg import MpcPlan, JointCommandFeedback

    config_path = ROOT/'isaac/controller_bringup/config/dmpc.yaml'
    raw_config = config_path.read_bytes()
    config = yaml.safe_load(raw_config)['/**']['ros__parameters']
    started = datetime.now(ZoneInfo('Asia/Seoul'))
    metrics = Measurements(args.scenario, config)
    process = None
    process_output = None
    final_diagnostics = {}
    final_feedback = {}
    final_plans = {}
    reason = '시작 준비 실패'
    # Keep the context available during Ctrl+C cleanup so lifecycle services
    # can release native Nav2 plugins before launch invalidates their context.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    node = Node('dmpc_test_recorder')

    def odom_callback(msg, i):
        if msg.header.frame_id != 'world' or msg.child_frame_id != f'{ROBOTS[i]}/base_link':
            metrics.invalid_odometry += 1
            return
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        speed = math.hypot(v.x, v.y)
        if not all(math.isfinite(x) for x in (stamp, p.x, p.y, yaw, speed)):
            metrics.invalid_odometry += 1
            return
        metrics.observe(i, stamp, p.x, p.y, yaw, speed, time.monotonic())

    def status_callback(msg, robot, kind):
        metrics.statuses[robot][kind][msg.data] += 1

    def command_callback(msg, i):
        value = math.hypot(msg.linear.x, msg.linear.y)
        if math.isfinite(value):
            metrics.max_command_speed[i] = max(metrics.max_command_speed[i], value)

    def plan_callback(msg, i):
        if msg.valid:
            metrics.valid_plans[i] += 1
        final_plans[ROBOTS[i]] = dict(valid=msg.valid, status=msg.status,
            positions=[[p.pose.position.x, p.pose.position.y] for p in msg.path.poses[:6]],
            velocities=list(msg.linear_velocities[:6]), yaw_rates=list(msg.angular_velocities[:6]))

    def diagnostic_callback(msg, robot):
        try:
            value = json.loads(msg.data)
        except (ValueError, TypeError):
            value = msg.data
        final_diagnostics[robot] = value

    def feedback_callback(msg, robot):
        # Preserve one compact final snapshot inside the same Markdown report.
        final_feedback[robot] = dict(command_valid=msg.command_valid,
            names=list(msg.state.name), measured_positions=list(msg.state.position),
            measured_velocities=list(msg.state.velocity),
            commanded_positions=[v if math.isfinite(v) else None for v in msg.applied_command.position],
            commanded_velocities=[v if math.isfinite(v) else None for v in msg.applied_command.velocity])

    try:
        for i, robot in enumerate(ROBOTS):
            node.create_subscription(Odometry, f'/{robot}/odom', lambda msg, i=i: odom_callback(msg, i), qos_profile_sensor_data)
            node.create_subscription(Twist, f'/{robot}/cmd_vel', lambda msg, i=i: command_callback(msg, i), 1)
            node.create_subscription(MpcPlan, f'/{robot}/mpc_plan', lambda msg, i=i: plan_callback(msg, i), 1)
            node.create_subscription(JointCommandFeedback, f'/{robot}/joint_feedback',
                                     lambda msg, r=robot: feedback_callback(msg, r), qos_profile_sensor_data)
            node.create_subscription(String, f'/{robot}/control_diagnostics',
                                     lambda msg, r=robot: diagnostic_callback(msg, r), 1)
            for suffix, kind in (('mpc_status', 'planner'), ('control_status', 'controller')):
                node.create_subscription(String, f'/{robot}/{suffix}', lambda msg, r=robot, k=kind: status_callback(msg, r, k), 1)
        print(f'[Test] Waiting for scenario {args.scenario} at its initial poses. Start sim first.', flush=True)
        ready_deadline = time.monotonic()+args.ready_timeout
        while not metrics.at_start(time.monotonic()):
            rclpy.spin_once(node, timeout_sec=.1)
            if metrics.clock_reset:
                # During readiness an old publisher sample can precede the
                # newly restarted scene. Begin a fresh readiness epoch; only
                # clock resets AFTER driving starts must abort a trial.
                metrics = Measurements(args.scenario, config)
            if time.monotonic() >= ready_deadline:
                raise RuntimeError('Isaac 입력 없음 또는 초기 위치·방향 불일치. 시뮬레이터를 같은 시나리오로 재시작하세요.')
        # A/C/D have the same initial poses: query the simulator to distinguish them.
        for robot in ROBOTS:
            client = node.create_client(GetParameters, f'/{robot}/isaac_io/get_parameters')
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError(f'{robot}: Isaac 시나리오 확인 서비스 없음')
            request = GetParameters.Request()
            request.names = ['scenario']
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
            if not future.done() or not future.result() or future.result().values[0].string_value != args.scenario:
                raise RuntimeError('Isaac 장면과 테스트 시나리오 불일치 또는 이전 실행 코드. sim을 재시작하세요.')
        active = [(name, ns) for name, ns in node.get_node_names_and_namespaces()
                  if ns in ('/robot_a', '/robot_b') and name in ('dmpc_planner', 'mpc_rpp_controller', 'controller_manager')]
        if active:
            raise RuntimeError(f'기존 주행 제어기를 먼저 종료하세요: {active}')
        metrics = Measurements(args.scenario, config)
        process = subprocess.Popen(['ros2', 'launch', 'antbot_dmpc_ros', 'two_antbot_dmpc.launch.py', f'scenario:={args.scenario}'],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True,
                                   text=True, encoding='utf-8', errors='replace', bufsize=1)
        process_output = ProcessOutput(process)
        deadline = time.monotonic()+args.timeout
        reason = '제한시간 종료(미완료)'
        last_print = 0.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            if metrics.clock_reset:
                reason = '시뮬레이션 재시작으로 중단'
                break
            if process.poll() is not None:
                reason = f'주행 프로세스 종료(code={process.returncode})'
                break
            if now-last_print >= 5.:
                progress = metrics.progress(now)
                distances = ', '.join(number(row['goal_error_m']) for row in progress)
                speeds = ', '.join(number(row['speed_mps']) for row in progress)
                ages = ', '.join(number(row['odom_age_s']) for row in progress)
                print(f'[Test {args.scenario}] odom_pairs={metrics.pairs} reached={metrics.reached(now)} '
                      f'goal_error_m=[{distances}] speed_mps=[{speeds}] odom_age_s=[{ages}] '
                      f'min_separation={number(metrics.minimum_separation)}', flush=True)
                last_print = now
            if all(metrics.reached(now)) and all(metrics.valid_plans):
                reason = '두 로봇 목표 도착'
                break
    except KeyboardInterrupt:
        reason = '사용자 중단'
    except Exception as error:
        reason = f'실행 오류: {error}'
        print(reason, flush=True)
    finally:
        measured_end = time.monotonic()
        result = metrics.summary(reason, measured_end)
        result['final_control_diagnostics'] = dict(final_diagnostics)
        result['final_joint_feedback'] = dict(final_feedback)
        result['final_plans'] = dict(final_plans)
        result['shutdown_errors'] = []
        if process is not None:
            if rclpy.ok():
                for robot in ROBOTS:
                    target = f'/{robot}/mpc_rpp_controller/change_state'
                    client = node.create_client(ChangeState, target)
                    if not client.wait_for_service(timeout_sec=1.):
                        result['shutdown_errors'].append(f'{robot}: lifecycle service unavailable')
                        continue
                    for transition in (Transition.TRANSITION_DEACTIVATE, Transition.TRANSITION_CLEANUP):
                        request = ChangeState.Request()
                        request.transition.id = transition
                        future = client.call_async(request)
                        rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
                        if not future.done() or future.exception() or not future.result().success:
                            result['shutdown_errors'].append(f'{robot}: lifecycle transition {transition} failed')
                            break
            for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
                try:
                    # ros2 launch forwards SIGINT to its own children. Sending
                    # it to the entire group makes them receive it twice and
                    # can interrupt ROS/plugin cleanup midway through shutdown.
                    if sig == signal.SIGINT:
                        process.send_signal(sig)
                    else:
                        os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=wait)
                    break
                except subprocess.TimeoutExpired:
                    continue
            result['drive_returncode'] = process.returncode
        result.update(scenario=args.scenario, started_kst=started.strftime('%Y-%m-%d %H:%M:%S'),
                      run_id=f'{started:%Y%m%d_%H%M%S_%f}__{args.scenario}',
                      config=config, config_sha256=hashlib.sha256(raw_config).hexdigest(),
                      source_sha256={str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                     for path in (ROOT/'dmpc_runtime/planner.py', ROOT/'dmpc_runtime/ros_node.py', ROOT/'dmpc_runtime/isaac_test.py',
                                                  ROOT/'isaac/path_following_controller/src/mpc_rpp_controller.cpp')})
        result.update(process_output.finish() if process_output is not None else dict(process_errors=[]))
        report = save_result(result, ROOT)
        print(f'[Test] {reason}; report: {report}', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if (reason == '두 로봇 목표 도착' and not result['violations'] and metrics.pairs
                 and not result['shutdown_errors'] and not result['process_errors']) else 1


if __name__ == '__main__':
    raise SystemExit(main())
