"""Check RPP's multi-robot wiring against the Isaac/Swerve interfaces."""
from copy import deepcopy
from pathlib import Path
import unittest

import yaml

from simulation.helpers.rpp_config import namespaced_rpp_parameters, mpc_rpp_parameters


class RppIntegrationConfigTests(unittest.TestCase):
    def test_native_rpp_owns_mpc_inputs_and_has_no_external_server_wiring(self):
        root = Path(__file__).resolve().parents[2]
        template = yaml.safe_load((root / 'path_following_controller/config/isaac/rpp.yaml').read_text())
        execution = yaml.safe_load((root / 'path_following_controller/config/isaac/mpc_execution.yaml').read_text())['/**']['ros__parameters']
        for robot in ('robot_a', 'robot_b', 'robot_c'):
            peers = [p for p in ('robot_a', 'robot_b', 'robot_c') if p != robot]
            params = mpc_rpp_parameters(template, execution, robot, peers, False)
            self.assertNotIn(f'/{robot}/controller_server', params)
            control = params[f'/{robot}/mpc_rpp_controller']['ros__parameters']
            self.assertEqual(control['ego_robot'], robot)
            self.assertEqual(control['peer_robots'], peers)
            self.assertFalse(control['enabled'])
            self.assertEqual(control['controller_frequency'], control['output_frequency'])
            self.assertNotIn('speed_limit_topic', control)
            self.assertEqual(control['FollowPath']['plugin'], 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')

    def test_two_controllers_use_separate_odometry_and_costmap_frames(self):
        root = Path(__file__).resolve().parents[2]
        template = yaml.safe_load(
            (root / 'path_following_controller/config/isaac/rpp.yaml').read_text()
        )
        original = deepcopy(template)
        generated = {}
        for robot in ('robot_a', 'robot_b'):
            params = namespaced_rpp_parameters(template, robot)
            self.assertTrue(set(generated).isdisjoint(params))
            generated.update(params)
            controller = params[f'/{robot}/controller_server']['ros__parameters']
            self.assertEqual(controller['odom_topic'], f'/{robot}/odom')
            self.assertEqual(controller['speed_limit_topic'], f'/{robot}/speed_limit')
            # Swerve's active subscription requires unstamped Twist.
            self.assertFalse(controller['enable_stamped_cmd_vel'])
            self.assertEqual(
                controller['FollowPath']['plugin'],
                'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
            )
            costmap = params[f'/{robot}/local_costmap/local_costmap']['ros__parameters']
            self.assertEqual(costmap['global_frame'], 'world')
            self.assertEqual(costmap['robot_base_frame'], f'{robot}/base_link')
            self.assertEqual(costmap['plugins'], ['inflation_layer'])
            self.assertNotIn('observation_sources', costmap)
            self.assertTrue(controller['use_sim_time'])
            self.assertTrue(costmap['use_sim_time'])
        self.assertEqual(template, original)


if __name__ == '__main__':
    unittest.main()
