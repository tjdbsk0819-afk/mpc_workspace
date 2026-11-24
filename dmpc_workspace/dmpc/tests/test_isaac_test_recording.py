import copy
from pathlib import Path
import tempfile
import unittest
import subprocess
import sys

import yaml

from dmpc_runtime.isaac_test import BEGIN, END, Measurements, ProcessOutput, save_result

ROOT = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((ROOT/'isaac/controller_bringup/config/dmpc.yaml').read_text())['/**']['ros__parameters']


class IsaacRecordingTests(unittest.TestCase):
    def test_pairing_does_not_compare_positions_from_different_times(self):
        m = Measurements('A', CONFIG)
        m.observe(0, 1., -2., 0., 0., .2, 10.)
        m.observe(1, 1.02, -2., 0., 0., .2, 10.)
        self.assertIsNone(m.minimum_separation)
        m.observe(0, 1.02, 2., 0., 0., .2, 10.)
        self.assertEqual(m.minimum_separation, 4.)
        self.assertEqual(m.pairs, 1)

    def test_goal_requires_fresh_odometry_and_stopped_robots(self):
        m = Measurements('B', CONFIG)
        m.observe(0, 1., 3., 0., 0., .5, 10.)
        m.observe(1, 1., 0., 3., 1.57, 0., 10.)
        self.assertEqual(m.reached(10.), [False, True])
        m.observe(0, 1.02, 3., 0., 0., 0., 10.)
        self.assertEqual(m.reached(10.), [True, True])
        self.assertEqual(m.reached(11.), [False, False])

    def test_wall_obstacle_and_separation_violations_are_preserved(self):
        corridor = Measurements('C', CONFIG)
        corridor.observe(0, 1., 0., .3, 0., 0., 10.)
        corridor.observe(1, 1., 0., 0., 0., 0., 10.)
        result = corridor.summary('제한시간 종료(미완료)', 10.)
        self.assertIn('로봇 간 최소 간격 미달', result['violations'])
        self.assertIn('벽 안전 범위 이탈', result['violations'])
        obstacle = Measurements('D', CONFIG)
        obstacle.observe(0, 1., 0., 0., 0., 0., 10.)
        obstacle.observe(1, 1., 3., 0., 0., 0., 10.)
        self.assertIn('고정 장애물 안전 범위 침범', obstacle.summary('중단', 10.)['violations'])

    def test_clock_reset_is_not_merged_with_the_previous_trial(self):
        m = Measurements('A', CONFIG)
        m.observe(0, 50., -3., 0., 0., 0., 10.)
        m.observe(0, .1, -3., 0., 0., 0., 11.)
        self.assertTrue(m.clock_reset)
        self.assertEqual(m.odom[0]['stamp'], 50.)

    def test_no_data_is_not_reported_as_arrival_or_zero_measured_speed(self):
        result = Measurements('A', CONFIG).summary('실행 오류: 입력 없음', 10.)
        self.assertEqual(result['reached'], [False, False])
        self.assertEqual(result['maximum_measured_speed'], [None, None])
        self.assertIsNone(result['minimum_separation'])
        self.assertEqual(result['assessment'], '측정 자료 부족')

    def test_arrival_does_not_erase_an_earlier_clearance_violation(self):
        m = Measurements('A', CONFIG)
        m.observe(0, 1., 0., 0., 0., 0., 10.)
        m.observe(1, 1., .5, 0., 3.14, 0., 10.)
        m.observe(0, 2., 3., 0., 0., 0., 10.1)
        m.observe(1, 2., -3., 0., 3.14, 0., 10.1)
        result = m.summary('두 로봇 목표 도착', 10.1)
        self.assertEqual(result['reached'], [True, True])
        self.assertEqual(result['assessment'], '관측 기준 위반')

    def test_reports_preserve_previous_runs_and_manual_notes(self):
        result = Measurements('C', CONFIG).summary('제한시간 종료(미완료)', 10.)
        result.update(scenario='C', started_kst='2026-10-28 16:00:00', config=CONFIG, config_sha256='test')
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root/'docs').mkdir()
            index = root/'docs/SCENARIO_TEST_RESULTS.md'
            index.write_text('기존 준비 결과\n'+BEGIN+'\n'+END+'\n수동 메모 유지\n')
            for name, scenario in (('first', 'C'), ('second', 'D')):
                value = copy.deepcopy(result)
                value['scenario'] = scenario
                value['run_id'] = name
                self.assertEqual(save_result(value, root), index)
            text = index.read_text()
            self.assertIn('기존 준비 결과', text)
            self.assertIn('수동 메모 유지', text)
            self.assertIn('[상세](#run-first)', text)
            self.assertIn('[상세](#run-second)', text)
            self.assertEqual(text.count('기존 준비 결과'), 1)
            self.assertIn('미도착', text)
            self.assertEqual([p.relative_to(root).as_posix() for p in root.rglob('*') if p.is_file()],
                             ['docs/SCENARIO_TEST_RESULTS.md'])
            save_result(value, root)
            self.assertEqual(index.read_text(), text)

    def test_shutdown_callbacks_do_not_change_frozen_drive_results(self):
        m = Measurements('B', CONFIG)
        m.valid_plans[0] = 2
        m.max_command_speed[0] = .3
        m.statuses['robot_a']['planner']['tracking'] = 2
        result = m.summary('제한시간 종료(미완료)', 10.)
        m.valid_plans[0] += 10
        m.max_command_speed[0] = .5
        m.statuses['robot_a']['planner']['tracking'] += 10
        self.assertEqual(result['valid_plan_counts'][0], 2)
        self.assertEqual(result['maximum_commanded_speed'][0], .3)
        self.assertEqual(result['statuses']['robot_a']['planner']['tracking'], 2)

    def test_process_output_is_bounded_and_detects_errors_without_log_files(self):
        with tempfile.TemporaryDirectory() as temp:
            process = subprocess.Popen([sys.executable, '-c',
                "for i in range(200): print('output',i)\n"
                "print('[ERROR] node: process has died [pid 1, exit code -11]')"],
                cwd=temp, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            output = ProcessOutput(process)
            process.wait(timeout=5.)
            result = output.finish()
            self.assertEqual(result['process_error_count'], 1)
            self.assertEqual(len(result['process_errors']), 1)
            self.assertEqual(len(result['output_tail']), 40)
            self.assertEqual(list(Path(temp).iterdir()), [])


if __name__ == '__main__':
    unittest.main()
