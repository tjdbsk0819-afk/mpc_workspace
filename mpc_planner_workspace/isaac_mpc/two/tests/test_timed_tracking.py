"""MPC timing and the bounded execution model used by the final command guard."""
import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'controller_bringup'))
from antbot_mpc_ros.timed_tracking import Motion,TimedTrajectory,ProximityReleaseGate,step,command_clearance,filter_command


class ProximityReleaseTests(unittest.TestCase):
    def setUp(self):
        self.gate = ProximityReleaseGate(150_000_000, 100_000_000)

    def allow_ms(self, milliseconds):
        return self.gate.allow(round(milliseconds*1_000_000))

    def test_normal_driving_is_not_delayed_but_release_needs_continuous_safety(self):
        self.assertTrue(self.allow_ms(0))
        self.gate.stop()
        for t in (50, 100, 150):
            self.assertFalse(self.allow_ms(t))
        self.assertTrue(self.allow_ms(200))
        self.gate.stop()
        self.assertFalse(self.allow_ms(250))

    def test_new_rejection_or_other_hold_restarts_the_window(self):
        self.gate.stop()
        self.assertFalse(self.allow_ms(0))
        self.assertFalse(self.allow_ms(100))
        self.gate.stop()
        self.assertFalse(self.allow_ms(150))
        self.assertFalse(self.allow_ms(250))
        self.gate.interrupt()
        self.assertFalse(self.allow_ms(300))
        self.assertFalse(self.allow_ms(400))
        self.assertTrue(self.allow_ms(450))

    def test_duplicate_clock_rollback_and_missing_checks_cannot_release(self):
        self.gate.stop()
        self.assertFalse(self.allow_ms(100))
        for _ in range(20):
            self.assertFalse(self.allow_ms(100))
        self.assertFalse(self.allow_ms(50))
        self.assertFalse(self.allow_ms(100))
        self.assertFalse(self.allow_ms(300))
        self.assertFalse(self.allow_ms(400))
        self.assertTrue(self.allow_ms(450))


class TimedTrackingTests(unittest.TestCase):
    def test_sample_uses_execution_age_and_acceleration_instead_of_v1(self):
        t=TimedTrajectory((0.,.2,.4),((0.,0.,0.),(.01,0.,0.),(.04,0.,0.)),(0.,.1,.2),(.5,.5,0.),(0.,0.,0.))
        p=t.sample(.05)
        self.assertAlmostEqual(p.v,.025)
        self.assertAlmostEqual(p.x,.000625)
        self.assertAlmostEqual(t.sample(.25).v,.125)
        self.assertAlmostEqual(t.window(.1,.1)[0].v,.05)
        with self.assertRaises(ValueError):t.sample(.5)

    def test_initial_turn_does_not_look_at_later_opposite_turn(self):
        t=TimedTrajectory((0.,.2,.4),((0.,0.,0.),(0.,0.,-.1),(0.,0.,-.2)),(0.,0.,0.),(0.,0.,0.),(-.5,-.5,.5))
        self.assertAlmostEqual(t.sample(.1).yaw,-.05)
        self.assertLess(t.window(0.,.2)[-1].yaw,0.)

    def test_invalid_missing_or_nonfinite_profile_rejected(self):
        for args in [((0.,.2),((0,0,0),(0,0,0)),(0.,),(0.,0.),(0.,0.)),
                     ((0.,0.),((0,0,0),(0,0,0)),(0.,0.),(0.,0.),(0.,0.)),
                     ((0.,.2),((0,0,0),(0,0,0)),(float('nan'),0.),(0.,0.),(0.,0.))]:
            with self.assertRaises(ValueError):TimedTrajectory(*args)

    def test_stop_has_finite_braking_distance_and_limits_are_respected(self):
        s=Motion(0.,0.,0.,.2,0.)
        self.assertLess(command_clearance(s,0.,0.,(1.18,0.,0.,0.),.25,.5,.5,.5),1.15)
        p=step(Motion(0.,0.,0.,0.,0.),.3,.8,.05,.5,.5)
        self.assertAlmostEqual(p.v,.025);self.assertAlmostEqual(p.w,.025)

    def test_speed_filter_preserves_curvature_and_does_not_relax_clearance(self):
        s=Motion(-.2867235468,-.3263028767,-.9657556121,0.,0.)
        selected=filter_command(s,.032,-.025,(.712971034,.2429250583,0.,0.),1.15,.25,.5,.5,.5)
        self.assertIsNotNone(selected)
        v,w,d,scale=selected
        self.assertLess(v,.032);self.assertAlmostEqual(w,-.025*scale);self.assertGreaterEqual(d,1.15)
        self.assertAlmostEqual(v,.032*scale)
        self.assertIsNone(filter_command(Motion(0,0,0,0,0),.1,0.,(.9,0.,0.,0.),1.15,.25,.5,.5,.5))

    def test_genuine_in_place_rotation_is_not_scaled_away(self):
        selected=filter_command(Motion(0,0,0,0,0),0.,-.03,(2.,0.,0.,0.),1.15,.25,.5,.5,.5)
        self.assertEqual(selected[:2],(0.,-.03))

    def test_blocked_translation_keeps_safe_mpc_rpp_turn_for_both_robots(self):
        separation=1.15+(1.155-1.15)*.9
        for state,peer in [(Motion(0,0,0,0,0),(1.155,0.,0.,0.)),
                           (Motion(1.155,0,math.pi,0,0),(0.,0.,0.,0.))]:
            selected=filter_command(state,.1,.2,peer,separation,.25,.5,.5,.5,
                                    mpc_yaw_rate=.2,rpp_yaw_rate=.2)
            self.assertEqual(selected[:2],(0.,.2))
            self.assertGreaterEqual(selected[2],separation)
            self.assertEqual(selected[3],0.)

    def test_fallback_requires_mpc_and_rpp_direction_agreement(self):
        for mpc,rpp in [(None,None),(-.2,.2),(.2,-.2),(0.,.2),(.2,0.),(float('nan'),.2)]:
            with self.subTest(mpc=mpc,rpp=rpp):
                selected=filter_command(Motion(0,0,0,0,0),.1,.2,(1.155,0.,0.,0.),1.1545,
                    .25,.5,.5,.5,mpc_yaw_rate=mpc,rpp_yaw_rate=rpp)
                self.assertEqual(selected[:2],(0.,0.))

    def test_rotation_uses_measured_braking_and_peer_motion(self):
        # A pure-turn target does not instantly stop an already moving robot.
        self.assertIsNone(filter_command(Motion(0,0,0,.2,0),.1,.2,(1.155,0.,0.,0.),1.1545,
            .25,.5,.5,.5,mpc_yaw_rate=.2,rpp_yaw_rate=.2))
        # Approaching peer makes rotation's longer horizon unsafe; checked stop
        # is still safe and must be selected instead of authorizing rotation.
        selected=filter_command(Motion(0,0,0,0,0),.1,.2,(1.155,0.,-.001,0.),1.1545,
            .25,.5,.5,.5,mpc_yaw_rate=.2,rpp_yaw_rate=.2)
        self.assertEqual(selected[:2],(0.,0.))
        self.assertGreaterEqual(selected[2],1.1545)

    def test_safe_translation_keeps_priority_over_rotation_fallback(self):
        selected=filter_command(Motion(0,0,0,0,0),.1,.2,(3.,0.,0.,0.),1.15,
            .25,.5,.5,.5,mpc_yaw_rate=.2,rpp_yaw_rate=.2)
        self.assertEqual(selected[:2],(.1,.2))
        self.assertEqual(selected[3],1.)


if __name__=='__main__':unittest.main()
