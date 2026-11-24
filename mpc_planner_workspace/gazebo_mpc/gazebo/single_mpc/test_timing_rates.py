import unittest
from timing_rates import planning_rates


class TimingRateTest(unittest.TestCase):
    def test_actual_rate_differs_from_inverse_planning_time(self):
        samples=[dict(start_wall_sec=10+i*.3,completion_wall_sec=10.2+i*.3,
                      planning_ms=200.,accepted=i!=1) for i in range(4)]
        result=planning_rates(samples,10.)
        self.assertAlmostEqual(result['start']['effective_hz'],1/.3)
        self.assertAlmostEqual(result['completion']['effective_hz'],1/.3)
        self.assertAlmostEqual(result['valid_plan_hz'],2/.9)
        self.assertNotAlmostEqual(result['completion']['effective_hz'],1000./200.)

    def test_first_sample_and_legacy_samples_do_not_invent_rates(self):
        for samples in [[],[{'stamp':3.,'planning_ms':20.}],
                        [dict(start_wall_sec=10.,completion_wall_sec=11.,accepted=True)]]:
            result=planning_rates(samples,10.)
            self.assertIsNone(result['start']['effective_hz'])
            self.assertIsNone(result['completion']['effective_hz'])
            self.assertIsNone(result['valid_plan_hz'])

    def test_no_valid_plans_is_zero_hz(self):
        samples=[dict(start_wall_sec=float(i),completion_wall_sec=i+.1,accepted=False) for i in range(3)]
        self.assertEqual(planning_rates(samples,10.)['valid_plan_hz'],0.)


if __name__=='__main__':unittest.main()
