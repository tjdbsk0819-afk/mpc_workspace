"""GT velocity frames, CV timing, freshness, and reset behavior without ROS."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'controller_bringup'))
from antbot_mpc_ros.cv_model import (
    PeerStateCache, WorldPlanarState, body_to_world_velocity,
    constant_velocity_prediction, state_from_body_odometry,
)


def sample(stamp_ns=1_000_000_000):
    return WorldPlanarState(stamp_ns, 'robot_b', 'world', 3.0, 2.0, -1.0, 0.5)


class GroundTruthFrameTests(unittest.TestCase):
    def test_yaw_rotates_body_forward_velocity_into_world_y(self):
        root = math.sqrt(0.5)
        state = state_from_body_odometry(
            0, 'robot_b', 'world', (3, 2, 0), (0, 0, root, root), (1, 0, 0))
        self.assertAlmostEqual(state.vx, 0)
        self.assertAlmostEqual(state.vy, 1)
        self.assertEqual((state.x, state.y), (3, 2))

    def test_lateral_swerve_velocity_is_rotated_too(self):
        root = math.sqrt(0.5)
        vx, vy, _ = body_to_world_velocity((0, 1, 0), (0, 0, root, root))
        self.assertAlmostEqual(vx, -1)
        self.assertAlmostEqual(vy, 0)

    def test_full_rotation_handles_pitch_and_vertical_velocity(self):
        root = math.sqrt(0.5)
        vx, vy, vz = body_to_world_velocity((0, 0, 1), (0, root, 0, root))
        self.assertAlmostEqual(vx, 1)
        self.assertAlmostEqual(vy, 0)
        self.assertAlmostEqual(vz, 0)

    def test_invalid_quaternion_or_nonfinite_gt_is_rejected(self):
        for q in ((0, 0, 0, 0), (0, 0, 0, 2), (0, 0, math.nan, 1)):
            with self.subTest(q=q), self.assertRaises(ValueError):
                body_to_world_velocity((1, 0, 0), q)
        with self.assertRaises(ValueError):
            body_to_world_velocity((math.inf, 0, 0), (0, 0, 0, 1))
        with self.assertRaises(ValueError):
            WorldPlanarState(0, 'robot_b', 'world', math.nan, 0, 0, 0)


class ConstantVelocityTests(unittest.TestCase):
    def test_matches_upstream_cv_equation_and_step_zero(self):
        points = constant_velocity_prediction(sample(), 1_000_000_000, 200_000_000, 4)
        self.assertEqual(points[0], (1_000_000_000, 3, 2))
        for k, (stamp, x, y) in enumerate(points):
            self.assertEqual(stamp, 1_000_000_000 + k*200_000_000)
            self.assertAlmostEqual(x, 3 - 0.2*k)
            self.assertAlmostEqual(y, 2 + 0.1*k)

    def test_old_gt_is_propagated_to_prediction_epoch(self):
        points = constant_velocity_prediction(sample(), 1_100_000_000, 200_000_000, 3)
        self.assertEqual(points[0][0], 1_100_000_000)
        self.assertAlmostEqual(points[0][1], 2.9)
        self.assertAlmostEqual(points[0][2], 2.05)
        self.assertAlmostEqual(points[2][1], 2.5)
        self.assertAlmostEqual(points[2][2], 2.25)

    def test_stationary_peer_and_nanosecond_timing(self):
        state = WorldPlanarState(1, 'robot_b', 'world', 3.0, 2.0, 0.0, 0.0)
        points = constant_velocity_prediction(state, 3, 7, 3)
        self.assertEqual(points, [(3, 3, 2), (10, 3, 2), (17, 3, 2)])

    def test_future_epoch_and_invalid_grid_are_rejected(self):
        for start, dt, steps in ((0, 200_000_000, 4), (1_000_000_000, 0, 4), (1_000_000_000, 1, 1)):
            with self.subTest(grid=(start, dt, steps)), self.assertRaises(ValueError):
                constant_velocity_prediction(sample(), start, dt, steps)


class FreshnessTests(unittest.TestCase):
    def setUp(self):
        self.cache = PeerStateCache()
        self.cache.observe_clock(1_000_000_000)
        self.cache.update(sample(), 10_000_000_000)

    def validity(self, now=1_100_000_000, wall=10_100_000_000):
        return self.cache.validity(now, wall, 500_000_000, 500_000_000)

    def test_fresh_sample(self):
        self.assertEqual(self.validity(), 'ok')

    def test_old_measurement_is_invalid_even_if_just_received(self):
        self.assertEqual(self.validity(now=1_600_000_001), 'stale_sim_time')

    def test_paused_simulation_does_not_keep_old_data_valid_forever(self):
        self.assertEqual(self.validity(wall=10_600_000_001), 'stale_wall_time')

    def test_future_sample_waits_for_simulation_clock(self):
        self.assertEqual(self.validity(now=999_999_999), 'future_state')

    def test_duplicate_and_out_of_order_messages_do_not_refresh_age(self):
        for stamp in (999_999_999, 1_000_000_000):
            self.assertFalse(self.cache.update(sample(stamp), 10_900_000_000))
        self.assertEqual(self.validity(wall=10_900_000_000), 'stale_wall_time')

    def test_world_reset_clears_previous_run_and_accepts_new_time_zero(self):
        self.assertTrue(self.cache.observe_clock(0))
        self.assertIsNone(self.cache.state)
        self.assertEqual(self.validity(now=0), 'waiting_for_peer')
        self.assertTrue(self.cache.update(sample(0), 11_000_000_000))
        self.assertEqual(self.validity(now=0, wall=11_000_000_000), 'ok')


if __name__ == '__main__':
    unittest.main()
