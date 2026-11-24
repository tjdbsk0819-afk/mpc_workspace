import unittest
import numpy as np
import casadi as ca
from legacy_mpc_reference.config import MPCConfig
from legacy_mpc_reference.model import step, step_symbolic, angle_error
from legacy_mpc_reference.planner import Planner, command_is_safe
from legacy_mpc_reference.trajectory import Prediction


class ModelTests(unittest.TestCase):
    def test_body_frame_and_in_place_rotation(self):
        np.testing.assert_allclose(step([0, 0, np.pi/2], [1, 0, 0], .2)[:2], [0, .2], atol=1e-12)
        np.testing.assert_allclose(step([0, 0, 0], [0, 1, 0], .2), [0, .2, 0])
        np.testing.assert_allclose(step([0, 0, 0], [0, 0, 1], .2), [0, 0, .2])

    def test_symbolic_and_simulation_match_with_rotation(self):
        x, u = ca.MX.sym('x', 3), ca.MX.sym('u', 3)
        f = ca.Function('step_test', [x, u], [step_symbolic(x, u, .2)])
        state, command = [1, 2, 1.1], [.3, -.2, .6]
        np.testing.assert_allclose(np.asarray(f(state, command)).reshape(3), step(state, command, .2), atol=1e-12)
        self.assertAlmostEqual(angle_error(-np.pi+.01, np.pi-.01), .02)

    def test_between_sample_collision_veto(self):
        cfg = MPCConfig(dt=1., radius=.1, margin=.1)
        self.assertFalse(command_is_safe(np.array([[-1., 0, 0], [1., 0, 0]]), np.array([[2., 0, 0], [-2., 0, 0]]), cfg))


class PredictionTests(unittest.TestCase):
    def setUp(self):
        self.cfg = MPCConfig()
        self.prediction = Prediction(1., .2, np.array([[0., 0, 0], [.2, 0, 0], [.4, 0, 0]]))

    def test_absolute_time_alignment_and_tail_uncertainty(self):
        states, inflation = self.prediction.align(1.1, self.cfg)
        self.assertAlmostEqual(states[0, 0], .1)
        self.assertGreater(inflation[0], 0)
        self.assertGreater(inflation[-1], inflation[0])

    def test_stale_future_wrong_frame_and_nan(self):
        for now, frame in [(1.7, 'world'), (.9, 'world'), (1.1, 'robot_a/odom')]:
            with self.assertRaises(ValueError):
                self.prediction.align(now, self.cfg, frame)
        with self.assertRaises(ValueError):
            Prediction(0., .2, np.full((3, 3), np.nan))


class PlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = MPCConfig(horizon=10, solver_max_seconds=2.)
        cls.planner = Planner(cls.cfg, 2)

    def test_successful_solution_obeys_dynamics_and_input_rates(self):
        states = np.array([[-2., -.8, 0], [2., .8, np.pi]])
        targets = np.array([[2., -.8, 0], [-2., .8, np.pi]])
        plan = self.planner.solve(states, targets, np.zeros((2, 3)))
        self.assertTrue(plan.success, plan.status)
        self.assertTrue(np.all(np.abs(plan.inputs) <= np.array(self.cfg.velocity)+1e-5))
        differences = np.diff(np.concatenate([np.zeros((2, 1, 3)), plan.inputs], axis=1), axis=1)
        self.assertTrue(np.all(np.abs(differences) <= np.array(self.cfg.acceleration)*self.cfg.dt+1e-5))
        for m in range(2):
            for k in range(self.cfg.horizon):
                np.testing.assert_allclose(step(plan.states[m, k], plan.inputs[m, k], self.cfg.dt), plan.states[m, k+1], atol=1e-5)

    def test_infeasible_overlap_returns_zero(self):
        plan = self.planner.solve(np.zeros((2, 3)), np.array([[2, 0, 0], [-2, 0, 0]]), np.zeros((2, 3)))
        self.assertFalse(plan.success)
        np.testing.assert_array_equal(plan.command, np.zeros((2, 3)))

    def test_missing_local_prediction_stops(self):
        plan = Planner(self.cfg, 1).solve([[0, 0, 0]], [[2, 0, 0]], [[0, 0, 0]])
        self.assertFalse(plan.success)
        np.testing.assert_array_equal(plan.command, np.zeros((1, 3)))


if __name__ == '__main__':
    unittest.main()
