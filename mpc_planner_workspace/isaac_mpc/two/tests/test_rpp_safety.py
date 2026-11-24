import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'controller_bringup'))
from antbot_mpc_ros.rpp_safety import minimum_cv_clearance


class CommandClearanceTests(unittest.TestCase):
    def test_stationary_robots(self):
        self.assertAlmostEqual(minimum_cv_clearance(0,0,0,0,0,3,0,0,0,1),3)

    def test_head_on_closing(self):
        self.assertAlmostEqual(minimum_cv_clearance(0,0,0,1,0,3,0,-1,0,1),1)

    def test_crossing_between_endpoints(self):
        self.assertAlmostEqual(minimum_cv_clearance(0,0,0,1,0,1,0,-1,0,1,segments=1),0)

    def test_body_heading_changes_world_direction(self):
        self.assertAlmostEqual(minimum_cv_clearance(0,0,math.pi,1,0,-3,0,1,0,1),1)

    def test_pure_rotation_preserves_circle_center(self):
        self.assertAlmostEqual(minimum_cv_clearance(0,0,0,0,1,3,0,0,0,1),3)

    def test_turning_tracks_arc(self):
        # One second with v=w=1 ends at (sin(1), 1-cos(1)).
        self.assertLess(minimum_cv_clearance(0,0,0,1,1,math.sin(1),1-math.cos(1),0,0,1),1e-10)

    def test_invalid_motion_rejected(self):
        with self.assertRaises(ValueError):
            minimum_cv_clearance(0,0,0,float('nan'),0,3,0,0,0,1)


if __name__ == '__main__':
    unittest.main()
