import math
import unittest

from dmpc_runtime.observations import PeerStateCache, WorldPlanarState, state_from_body_odometry
from dmpc_runtime.prediction import ObjectState, PredictionConfig, observation_from_model_state, predict_object


class PredictionTests(unittest.TestCase):
    def test_measurement_age_and_horizon_alignment(self):
        state = ObjectState("b", 10, (1, 2, 0), (2, -1, 0))
        prediction = predict_object(state, 10.5, 0.2, 3, PredictionConfig("cv"))
        self.assertEqual(prediction.times, (10.5, 10.7, 10.9, 11.1))
        self.assertEqual(prediction.positions[0], (2, 1.5, 0))
        self.assertAlmostEqual(prediction.positions[-1][0], 3.2)
        self.assertTrue(all(r == 0 for r in prediction.radii))

    def test_default_prediction_is_cv_plus_gaussian(self):
        from config.dmpc_cfg import DecentralizedParams
        self.assertEqual(DecentralizedParams.__dataclass_fields__['prediction_model'].default, 'cv_gaussian')
        state=ObjectState('b',0.,(1.,2.,0.),(2.,-1.,0.))
        result=predict_object(state,0.,.2,3)
        self.assertEqual(result.positions[0],state.position)
        self.assertAlmostEqual(result.positions[1][0],1.4)
        self.assertEqual(result.radii[0],0.)
        self.assertGreater(result.radii[1],0.)

    def test_gaussian_padding_matches_reference_and_includes_age(self):
        state = ObjectState("b", 0, (0, 0, 0), (1, 0, 0))
        result = predict_object(state, 0.4, 0.2, 3, PredictionConfig("cv_gaussian", 0.3, 0.05))
        self.assertAlmostEqual(result.radii[0], math.sqrt(-2*math.log(.05))*.3*math.sqrt(.2*.4))
        self.assertGreater(result.radii[-1], result.radii[0])
        self.assertEqual(result.positions[0][0], .4)

    def test_body_velocity_is_rotated_before_prediction(self):
        q = (0, 0, math.sin(math.pi/4), math.cos(math.pi/4))
        state = state_from_body_odometry(0, "b", "world", (1, 2, 0), q, (1, 0, 0))
        self.assertAlmostEqual(state.vx, 0)
        self.assertAlmostEqual(state.vy, 1)

    def test_repository_state_layouts(self):
        state = observation_from_model_state("b", (1, 2, 0, math.pi/2, 2), "bicycle", 0)
        self.assertAlmostEqual(state.velocity[1], 2)
        state = observation_from_model_state("b", tuple(range(12)), "double_integrator", 0)
        self.assertEqual(state.velocity, (6, 7, 8))
        state = observation_from_model_state("b", tuple(range(6)), "drone", 0)
        self.assertEqual(state.velocity, (3, 4, 5))

    def test_replayed_state_does_not_extend_wall_validity(self):
        cache = PeerStateCache()
        state = WorldPlanarState(100, "b", "world", 1, 2, 1, 0)
        self.assertTrue(cache.update(state, 1000))
        self.assertFalse(cache.update(state, 2000))
        self.assertEqual(cache.validity(110, 2100, 100, 1000), "stale_wall_time")
        cache.observe_clock(110)
        self.assertTrue(cache.observe_clock(0))
        self.assertIsNone(cache.state)

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            ObjectState("b", 0, (float("nan"), 0, 0), (0, 0, 0))
        state = ObjectState("b", 1, (0, 0, 0), (0, 0, 0))
        for start, dt, horizon in ((0, .2, 30), (1, 0, 30), (1, .2, 0)):
            with self.assertRaises(ValueError):
                predict_object(state, start, dt, horizon)


if __name__ == "__main__":
    unittest.main()
