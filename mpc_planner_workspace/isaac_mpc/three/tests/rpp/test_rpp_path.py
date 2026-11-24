"""MPC turn prefixes must retain genuine optimized poses and stop before travel."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'controller_bringup'))
from antbot_mpc_ros.rpp_path import rotation_prefix_count
from antbot_mpc_ros.rpp_path import alignment_heading
from antbot_mpc_ros.timed_tracking import TimedTrajectory


class TurnPrefixTests(unittest.TestCase):
    def test_alignment_uses_latest_mpc_lookahead_and_stops_before_opposite_turn(self):
        t=TimedTrajectory((0.,.2,.4,.6),((0,0,0),(0,0,-.06),(0,0,-.12),(0,0,-.06)),
                          (0.,)*4,(0.,)*4,(-.3,-.3,.3,.3))
        self.assertAlmostEqual(alignment_heading(t,0.,0.,-.3,.4),-.12)
        self.assertAlmostEqual(alignment_heading(t,0.,0.,-.3,.6),-.12)
        self.assertIsNone(alignment_heading(t,0.,-.2,-.3,.4))

    def test_selects_turn_until_translation_exceeds_near_goal_region(self):
        points=[(0.,0.,0.),(0.,0.,.05),(.001,0.,.15),(.003,0.,.3),(.006,0.,.4)]
        self.assertEqual(rotation_prefix_count(points,0.,.25),4)
        self.assertEqual(rotation_prefix_count(points,.1,.25),0)
        self.assertEqual(rotation_prefix_count(points,0.,0.),0)

    def test_wraparound_and_opposite_rotation(self):
        points=[(0.,0.,3.13),(0.,0.,-3.10),(0.,0.,-3.0)]
        self.assertEqual(rotation_prefix_count(points,0.,.3),3)
        self.assertEqual(rotation_prefix_count(points,0.,-.3),0)

    def test_no_invented_turn_when_solver_only_supplies_translation_or_small_heading(self):
        self.assertEqual(rotation_prefix_count([(0.,0.,0.),(.1,0.,.3)],0.,.3),0)
        self.assertEqual(rotation_prefix_count([(0.,0.,0.),(0.,0.,.001)],0.,.3),0)


if __name__=='__main__':unittest.main()
