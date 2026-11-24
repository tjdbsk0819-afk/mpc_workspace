"""Verify the generated MPC has all three peer slots plus C/D constraints."""
from pathlib import Path
import sys
import unittest
import casadi as ca
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'mpc_planning/solver_generator'),str(ROOT/'mpc_planning/mpc_planner_modules/scripts')]
from ellipsoid_constraints import EllipsoidConstraintModule
from workspace_bounds import WorkspaceBoundsModule
from static_obstacle import StaticObstacleModule

class Values(dict):
    def get(self,key):return self[key]

class HardConstraintTests(unittest.TestCase):
    def test_third_peer_has_an_independent_hard_constraint(self):
        settings=yaml.safe_load((ROOT/'mpc_planning/mpc_planner_antbot/config/settings.yaml').read_text())
        constraint=EllipsoidConstraintModule(settings).constraints[0]
        self.assertEqual(constraint.nh,3)
        params=Values(ego_disc_radius=.5,ego_disc_0_offset=0.)
        for i in range(3):
            params.update({f'ellipsoid_obst_{i}_{key}':value for key,value in
                dict(x=10.,y=0.,psi=0.,major=0.,minor=0.,chi=1.,r=.65).items()})
        model=Values(x=0.,y=0.,psi=0.)
        values=constraint.get_constraints(model,params,settings,1)
        self.assertTrue(all(float(ca.DM(v))>=1. for v in values))
        params['ellipsoid_obst_2_x']=.5
        values=constraint.get_constraints(model,params,settings,1)
        self.assertGreaterEqual(float(ca.DM(values[0])),1.)
        self.assertGreaterEqual(float(ca.DM(values[1])),1.)
        self.assertLess(float(ca.DM(values[2])),1.)
    def test_corridor_and_static_obstacle_still_constrain_all_robot_planners(self):
        bounds=WorkspaceBoundsModule({}).constraints[0]
        wall=Values(workspace_xmin=-3.45,workspace_xmax=3.45,workspace_ymin=-.15,workspace_ymax=.15)
        self.assertTrue(all(v>=0 for v in bounds.get_constraints(Values(x=0.,y=0.),wall,{},1)))
        self.assertTrue(any(v<0 for v in bounds.get_constraints(Values(x=0.,y=.2),wall,{},1)))
        static=StaticObstacleModule({}).constraints[0]
        cylinder=Values(static_enabled=1.,static_x=0.,static_y=0.,static_separation=1.1)
        self.assertLess(float(static.get_constraints(Values(x=.5,y=0.),cylinder,{},1)[0]),0.)
        self.assertGreaterEqual(float(static.get_constraints(Values(x=2.,y=0.),cylinder,{},1)[0]),0.)

if __name__=='__main__':unittest.main()
