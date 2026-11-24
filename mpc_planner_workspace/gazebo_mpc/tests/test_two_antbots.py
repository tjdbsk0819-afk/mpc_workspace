from copy import deepcopy
import math
import os
from pathlib import Path
import tempfile
import subprocess
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import numpy as np
from simulation.helpers.state_utils import world_to_body, yaw_quaternion, time_parts, root_state, CommandNodeAdapter
from simulation.helpers.controller_config import namespaced_parameters, namespaced_control_description
from simulation.run_two_antbots import parse_arguments, resolve_isaac_workspace
from simulation import run_two_antbots as runner
from simulation.helpers.scenario_config import scenario_layout, planner_goals


class StateTests(unittest.TestCase):
    def test_default_crossing_spawn_and_planner_goals_are_consistent(self):
        args = parse_arguments([])
        self.assertEqual(args.scenario, 'B')
        spec = scenario_layout(args.scenario)
        self.assertEqual(spec.starts, ((-3., 0., 0.), (0., -3., math.pi/2)))
        for name in ('A', 'B', 'C', 'D'):
            goals = planner_goals(name)
            for robot, pose in zip(('robot_a', 'robot_b'), scenario_layout(name).goals):
                self.assertEqual(tuple(goals[f'{robot}_goal_{axis}'] for axis in ('x', 'y', 'yaw')), pose)
        self.assertEqual(planner_goals('B', {'robot_b_goal_x': '0', 'robot_b_goal_y': '4'})['robot_b_goal_y'], 4.)
        self.assertEqual(planner_goals('A', {'robot_b_goal_x': 0})['robot_b_goal_x'], 0.)
        with self.assertRaises(ValueError):
            planner_goals('Z')

    def test_isaac_scenarios_and_numeric_helpers_work_without_casadi(self):
        # Reproduce the Isaac image, which deliberately has no MPC solver.
        script = '''
import sys
from importlib.abc import MetaPathFinder
class NoCasadi(MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'casadi' or fullname.startswith('casadi.'):
            raise ModuleNotFoundError('CasADi is absent in the Isaac runtime')
sys.meta_path.insert(0, NoCasadi())
from legacy_mpc_reference.scenarios import scenario
from legacy_mpc_reference.model import step, angle_error
from simulation.helpers.state_utils import yaw_quaternion
for name in 'ABCD':
    assert scenario(name).starts.shape == (2, 3)
assert abs(step([0, 0, 0], [1, 0, 0], .2)[0] - .2) < 1e-12
assert abs(angle_error(0, 0)) < 1e-12
assert yaw_quaternion(0) == (0., 0., 0., 1.)
assert 'casadi' not in sys.modules
'''
        result = subprocess.run([sys.executable, '-c', script], cwd=Path(__file__).resolve().parents[1],
                                capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_world_velocity_to_rotated_body(self):
        np.testing.assert_allclose(world_to_body([0, 1, 0], yaw_quaternion(math.pi/2)), [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(world_to_body([-1, 0, 0], yaw_quaternion(math.pi)), [1, 0, 0], atol=1e-12)

    def test_root_uses_link_pose_and_velocity(self):
        q = yaw_quaternion(math.pi/2)
        robot = SimpleNamespace(data=SimpleNamespace(root_link_pose_w=np.array([[2, 3, .4, *q]]),
                                root_link_vel_w=np.array([[0, 1, 0, 0, 0, .2]])))
        pose, linear, angular = root_state(robot)
        np.testing.assert_allclose(pose[:3], [2, 3, .4])
        np.testing.assert_allclose(linear, [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(angular, [0, 0, .2])

    def test_timestamp_carry(self):
        self.assertEqual(time_parts(.9999999999), (1, 0))

    def test_invalid_command_does_not_reach_subscriber_or_refresh_watchdog(self):
        class FakeNode:
            def create_subscription(self, kind, topic, callback, qos, **kwargs):
                self.callback = callback
                return SimpleNamespace(topic_name=topic)
        node = FakeNode()
        applied, refreshed = [], []
        adapter = CommandNodeAdapter(node, ['steer'], ['wheel'], lambda: refreshed.append(True))
        adapter.create_subscription(object, '/joint_command', applied.append, 1)
        node.callback(SimpleNamespace(name=['steer'], position=[0.], velocity=[]))
        node.callback(SimpleNamespace(name=['steer', 'wheel'], position=[0., float('nan')], velocity=[float('nan'), float('nan')]))
        self.assertFalse(applied)
        self.assertFalse(refreshed)
        node.callback(SimpleNamespace(name=['steer', 'wheel'], position=[.2, float('nan')], velocity=[float('nan'), 1.]))
        self.assertEqual(len(applied), 1)
        self.assertEqual(len(refreshed), 1)


class ControllerTests(unittest.TestCase):
    def test_reference_discovery_from_shallow_container_mount(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root/'simulation_runtime').mkdir()
            (root/'robot_model').mkdir()
            (root/'simulation_runtime/run_antbot.py').touch()
            (root/'robot_model/asset.py').touch()
            for mount in ('/dmpc', '/'):
                with self.subTest(mount=mount), patch.object(runner, 'DMPC_ROOT', Path(mount)), \
                     patch.dict(os.environ, {'ANTBOT_ISAAC_WORKSPACE': str(root)}):
                    self.assertEqual(resolve_isaac_workspace(), root.resolve())
            with patch.object(runner, 'DMPC_ROOT', Path('/dmpc')), patch.object(Path, 'is_file', return_value=False):
                with self.assertRaises(FileNotFoundError):
                    resolve_isaac_workspace()

    def test_namespace_parameters_preserve_reference_geometry(self):
        template = {'controller_manager': {'ros__parameters': {'update_rate': 50}},
                    'antbot_swerve_controller': {'ros__parameters': {'cmd_vel_topic': '/cmd_vel',
                     'wheel_radius': .103, 'enable_odom_tf': False}}}
        original = deepcopy(template)
        result = namespaced_parameters(template, 'robot_b')
        self.assertEqual(result['/robot_b/antbot_swerve_controller']['ros__parameters']['cmd_vel_topic'], '/robot_b/cmd_vel')
        self.assertEqual(result['/robot_b/antbot_swerve_controller']['ros__parameters']['wheel_radius'], .103)
        generated = result['/robot_b/antbot_swerve_controller']['ros__parameters']['antbot_swerve_controller']
        self.assertEqual(generated['cmd_vel_topic'], '/robot_b/cmd_vel')
        self.assertEqual(generated['wheel_radius'], .103)
        self.assertFalse(generated['enable_odom_tf'])
        self.assertNotIn('antbot_swerve_controller', generated)
        self.assertEqual(template, original)

    def test_two_hardware_descriptions_have_separate_topic_routes(self):
        xml = '<robot name="antbot"><ros2_control name="system"><hardware><plugin>antbot_isaac_control/IsaacTopicSystem</plugin><param name="state_topic">/joint_states</param><param name="command_topic">/joint_command</param></hardware></ros2_control></robot>'
        for robot in ('robot_a', 'robot_b'):
            result = ET.fromstring(namespaced_control_description(xml, robot))
            params = {p.get('name'): p.text for p in result.findall('ros2_control/hardware/param')}
            self.assertEqual(params['state_topic'], f'/{robot}/joint_states')
            self.assertEqual(params['command_topic'], f'/{robot}/joint_command')

    def test_runner_argument_parsing_needs_no_isaac_runtime(self):
        args = parse_arguments(['--headless', '--robot-b-start', '3', '0', str(math.pi)])
        self.assertTrue(args.headless)
        self.assertEqual(args.robot_b_start[:2], [3., 0.])
        # Build-time tests must work without mounting the external workspace.
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root/'simulation_runtime').mkdir()
            (root/'robot_model').mkdir()
            (root/'simulation_runtime/run_antbot.py').write_text('')
            (root/'robot_model/asset.py').write_text('')
            self.assertEqual(resolve_isaac_workspace(str(root)), root.resolve())


if __name__ == '__main__':
    unittest.main()
