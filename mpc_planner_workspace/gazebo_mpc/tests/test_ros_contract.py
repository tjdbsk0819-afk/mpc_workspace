import unittest
import numpy as np
from legacy_mpc_reference.config import MPCConfig
from legacy_mpc_reference.ros_contract import fresh_stamp, uniform_prediction_times, local_command_is_safe


class ContractTests(unittest.TestCase):
    def test_freshness_uses_sim_and_wall_time(self):
        self.assertTrue(fresh_stamp(1., 1.1, 20., 20.1, .5))
        self.assertFalse(fresh_stamp(1., 1.1, 20., 21., .5))
        self.assertFalse(fresh_stamp(1., 1.7, 20., 20.1, .5))
        self.assertFalse(fresh_stamp(2., 1., 20., 20.1, .5))

    def test_prediction_epoch_and_spacing(self):
        self.assertAlmostEqual(uniform_prediction_times([1., 1.2, 1.4], 1.), .2)
        for times, epoch in [([1., 1.2, 1.5], 1.), ([1., 1.], 1.), ([1., 1.2], .9)]:
            with self.assertRaises(ValueError):
                uniform_prediction_times(times, epoch)

    def test_local_veto_predicts_between_sample_collision(self):
        cfg = MPCConfig(dt=1., radius=.1, margin=.1)
        peer = np.tile([1., 0., 0.], (cfg.horizon+1, 1))
        peer[1, 0] = -1.
        self.assertFalse(local_command_is_safe([-1., 0., 0.], [2., 0., 0.], peer,
                         np.zeros(cfg.horizon+1), cfg))
        peer[:] = [1., 2., 0.]
        self.assertTrue(local_command_is_safe([-1., 0., 0.], [2., 0., 0.], peer,
                        np.zeros(cfg.horizon+1), cfg))


if __name__ == '__main__':
    unittest.main()
