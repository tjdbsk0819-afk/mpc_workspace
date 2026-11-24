import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'controller_bringup'))
from antbot_mpc_ros.timed_tracking import Motion,filter_command_multi
from simulation.robot_layout import load_layout

class ThreeRobotTests(unittest.TestCase):
    def test_t_layout_and_staggered_start(self):
        a=load_layout(scenario='simultaneous');b=load_layout(scenario='staggered')
        self.assertEqual(len(a),3)
        self.assertEqual(a['robot_c']['start'],[0.,-3.,math.pi/2])
        self.assertEqual(b['robot_c']['start'],[0.,-5.,math.pi/2])
        self.assertEqual(a['robot_c']['goal'],b['robot_c']['goal'])
        self.assertEqual(a['robot_a'],b['robot_a'])

    def test_second_neighbor_can_veto_first_neighbor_safe_command(self):
        far=((5.,0.,0.,0.),1.15);close=((.9,0.,0.,0.),1.15)
        state=Motion(0,0,0,0,0)
        self.assertEqual(filter_command_multi(state,.1,0.,[far],.25,.5,.5,.5)[:2],(.1,0.))
        for neighbors in ([far,close],[close,far]):
            self.assertIsNone(filter_command_multi(state,.1,0.,neighbors,.25,.5,.5,.5))

    def test_rotation_must_pass_both_neighbors(self):
        neighbors=[((5.,0.,0.,0.),1.155),((1.155,0.,0.,0.),1.1545)]
        result=filter_command_multi(Motion(0,0,0,0,0),.1,.2,neighbors,.25,.5,.5,.5,mpc_yaw_rate=.2,rpp_yaw_rate=.2)
        self.assertEqual(result[:2],(0.,.2))
        neighbors.append(((0.,.9,0.,0.),1.15))
        self.assertIsNone(filter_command_multi(Motion(0,0,0,0,0),.1,.2,neighbors,.25,.5,.5,.5,mpc_yaw_rate=.2,rpp_yaw_rate=.2))

    def test_empty_neighbor_set_cannot_bypass_guard(self):
        with self.assertRaises(ValueError):filter_command_multi(Motion(0,0,0,0,0),.1,0.,[],.25,.5,.5,.5)

if __name__=='__main__':unittest.main()
