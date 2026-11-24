import unittest
from dataclasses import replace

import numpy as np

from dmpc_runtime.planner import LocalObjectMPC, PlannerConfig, integrate, goal_speed_limit, yaw_command_reference
from dmpc_runtime.prediction import ObjectState
from config.dmpc_cfg import DecentralizedParams
from utils.mpc_utils import predict_from_current_states, set_object_predictions
import casadi as ca


class PlannerTests(unittest.TestCase):
    def test_yaw_slew_uses_fresh_issued_commands_and_expires_stale_commands(self):
        self.assertEqual(yaw_command_reference(.05, .2, .02, .35, .8), .2)
        self.assertEqual(yaw_command_reference(.05, .2, .5, .35, .8), .05)
        self.assertEqual(yaw_command_reference(.05, None, 0., .35, .8), .05)
        self.assertEqual(yaw_command_reference(.05, .2, -.1, .35, .8), .05)

    def test_slow_yaw_response_does_not_pin_the_input_slew_ramp(self):
        measured, command = 0., 0.
        for _ in range(10):
            reference = yaw_command_reference(measured, command, .02, .35, .8)
            command = min(.8, reference+.05)
            measured = .5*command
        self.assertGreater(command, .45)
        # Anchoring every new input to measured response plateaus below 0.1.
        measured, old_command = 0., 0.
        for _ in range(10):
            old_command = measured+.05
            measured = .5*old_command
        self.assertLess(old_command, .1)

    def test_original_decentralized_input_comes_from_states_only(self):
        config = DecentralizedParams(N=5, Q=np.eye(5), R=np.eye(2), H=np.eye(5), term=False, mode="jacobi")
        observed = np.array([[0, 0, 0, 0, .5], [2, 0, 0, np.pi/2, 1]])
        forecasts = predict_from_current_states(observed, "bicycle", 0, .2, 5, config)
        opti = ca.Opti()
        agents = [{"opti": opti, "XYZ_others": [opti.parameter(3, 6)], "SEP_others": [opti.parameter(1, 6)]}]
        set_object_predictions(agents, forecasts, 0, 1.15)
        actual = np.asarray(opti.value(agents[0]["XYZ_others"][0]))
        np.testing.assert_allclose(actual[:, -1], [2, 1, 0], atol=1e-10)
        np.testing.assert_allclose(opti.value(agents[0]["SEP_others"][0]),
                                   1.15+np.asarray(forecasts[1].radii))
        self.assertGreater(forecasts[1].radii[-1],0.)

    def test_own_solution_depends_on_observed_object_motion(self):
        config = PlannerConfig(horizon=15, solve_seconds=1, prediction_model="cv")
        a = LocalObjectMPC(config)
        peer = ObjectState("b", 0, (0, -2, 0), (0, .5, 0))
        plan = a.solve([-2, 0, 0, 0], [2, 0, 0], peer, 0)
        self.assertEqual(plan.states.shape, (4, 16))
        self.assertEqual(plan.controls.shape, (2, 15))
        self.assertAlmostEqual(plan.prediction.positions[-1][1], -.5)
        a.validate(plan)
        # This API accepts no peer goal, control, or optimized trajectory.
        changed = replace(peer, velocity=(0, -.5, 0))
        a.reset()
        other = a.solve([-2, 0, 0, 0], [2, 0, 0], changed, 0)
        self.assertGreater(np.max(np.abs(plan.states-other.states)), .01)

    def test_reverse_enabled_plans_backwards_and_accepts_reverse_odometry(self):
        config = PlannerConfig(horizon=10, allow_reverse=True, solve_seconds=0., prediction_model='cv')
        peer = ObjectState('peer', 0., (0., 5., 0.), (0., 0., 0.))
        for speed in (0., -.15):
            solver = LocalObjectMPC(config)
            plan = solver.solve([0., 0., 0., speed], [-2., 0., 0.], peer, 0.)
            self.assertLess(plan.states[3, 2], -.01)
            self.assertLess(plan.states[0, -1], -.1)
            self.assertAlmostEqual(plan.states[3, 0], speed, places=5)
            solver.validate(plan)

    def test_collision_at_time_zero_is_rejected(self):
        solver = LocalObjectMPC(PlannerConfig(horizon=5))
        with self.assertRaisesRegex(ValueError, "current object clearance"):
            solver.solve([0, 0, 0, 0], [2, 0, 0], ObjectState("b", 0, (.5, 0, 0), (0, 0, 0)), 0)

    def test_small_signed_drift_during_steering_does_not_disable_mpc(self):
        solver = LocalObjectMPC(PlannerConfig(horizon=15, solve_seconds=1., prediction_model='cv'))
        peer = ObjectState('a', 0., (3., 0., 0.), (0., 0., 0.))
        plan = solver.solve([-.22, 2.907, 1.301, -.005], [0., 3., np.pi/2], peer, 0.)
        self.assertAlmostEqual(plan.states[3, 0], 0., places=6)
        solver.validate(plan)
        with self.assertRaisesRegex(ValueError, 'Ego speed'):
            solver.solve([-.22, 2.907, 1.301, -.05], [0., 3., np.pi/2], peer, .1)

    def test_corridor_cold_start_recovers_from_recorded_stopped_poses(self):
        poses = [(-2.1849291325, -.0830522925, -.1530780918),
                 (.8748116493, .1488274932, -3.1363439862)]
        config = PlannerConfig(max_speed=1., bounds=(-4., 4., -.7, .7),
                               prediction_model='cv_gaussian', solve_seconds=1.)
        for i in (0, 1):
            with self.subTest(robot=i):
                planner = LocalObjectMPC(config)
                peer = ObjectState('peer', 0., (*poses[1-i][:2], 0.), (0., 0., 0.))
                goal = [3., 0., 0.] if i == 0 else [-3., 0., np.pi]
                plan = planner.solve([*poses[i], 0.], goal, peer, 0.)
                self.assertGreater(plan.states[3, 1], .01)
                planner.validate(plan)

    def test_unicycle_profiles_match_executor_quadrature(self):
        result = np.asarray(integrate([1, 2, 0, .2], [.5, 0], .2)).ravel()
        np.testing.assert_allclose(result, [1.05, 2, 0, .3], atol=1e-10)

    def test_model_frames_and_parameters_are_validated(self):
        with self.assertRaises(ValueError):
            PlannerConfig(bounds=(-1, 1, -.1, .1))
        for margin in (-.001, float('nan')):
            with self.assertRaises(ValueError):
                PlannerConfig(wall_planning_margin=margin)
        with self.assertRaisesRegex(ValueError, 'Bounds leave no space'):
            PlannerConfig(bounds=(-1, 1, -.551, .551), wall_planning_margin=.002)
        solver = LocalObjectMPC(PlannerConfig(horizon=5))
        with self.assertRaisesRegex(ValueError, "coordinate frame"):
            solver.solve([0, 0, 0, 0], [2, 0, 0], ObjectState("b", 0, (5, 0, 0), (0, 0, 0), "base_link"), 0)

    def test_goal_braking_distance_includes_response_time_and_reserve(self):
        c = PlannerConfig(max_speed=1.)
        for distance in (.08, .2, .5, 1., 3.):
            speed = goal_speed_limit(distance, c)
            stopping = speed*c.goal_reaction_time+speed**2/(2*c.goal_braking_deceleration)
            self.assertLessEqual(stopping, max(0., distance-c.goal_stop_reserve)+1e-9)
        self.assertEqual(goal_speed_limit(5., c), 1.)
        self.assertEqual(goal_speed_limit(.05, c), 0.)

    def test_high_speed_goal_approach_is_braked_before_the_goal(self):
        c = PlannerConfig(max_speed=1., solve_seconds=1., prediction_model='cv')
        planner = LocalObjectMPC(c)
        plan = planner.solve([0., 0., 0., 1.], [3., 0., 0.],
                             ObjectState('b', 0., (0., 10., 0.), (0., 0., 0.)), 0.)
        crossing = np.flatnonzero(np.linalg.norm(plan.states[:2].T-np.array([3., 0.]), axis=1) <= .2)
        self.assertTrue(crossing.size)
        self.assertLessEqual(plan.states[3, crossing[0]], goal_speed_limit(.2, c)+1e-3)
        self.assertLessEqual(np.max(plan.states[0]), 3.05)
        planner.validate(plan)

    def test_measured_speed_outside_braking_envelope_can_recover(self):
        c = PlannerConfig(max_speed=1., solve_seconds=1., prediction_model='cv')
        planner = LocalObjectMPC(c)
        plan = planner.solve([2.5, 0., 0., .8], [3., 0., 0.],
                             ObjectState('b', 0., (0., 10., 0.), (0., 0., 0.)), 0.)
        self.assertLess(plan.controls[0, 0], -.4)
        planner.validate(plan)


if __name__ == "__main__":
    unittest.main()
