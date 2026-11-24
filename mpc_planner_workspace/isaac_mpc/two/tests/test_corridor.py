"""Physical wall geometry, curved motion and braking-aware final/fallback checks."""
from pathlib import Path
import subprocess
import tempfile
import unittest
from simulation.helpers.scenario_config import planner_goals, scenario_layout, workspace_parameters

ROOT = Path(__file__).resolve().parents[1]


class CorridorTests(unittest.TestCase):
    def test_shared_geometry_and_goal_validation(self):
        self.assertEqual(scenario_layout('C').bounds, (-4., 4., -.7, .7))
        self.assertTrue(workspace_parameters('C')['workspace_bounds_enabled'])
        self.assertFalse(workspace_parameters('A')['workspace_bounds_enabled'])
        self.assertFalse(workspace_parameters('B')['workspace_bounds_enabled'])
        self.assertEqual(planner_goals('C')['robot_b_goal_x'], -3.)
        with self.assertRaises(ValueError):
            planner_goals('C', {'robot_a_goal_y': '.2'})

    def test_curves_braking_and_old_plan_cannot_cross_walls(self):
        source = r'''
#include "mpc_tracking.hpp"
#include "execution_policy.h"
#include <cassert>
int main() {
  antbot_workspace::Bounds b;b.configure(true,{-4.,4.,-.7,.7},.5,.05);
  const auto c=b.centers();assert(std::abs(c[2]+.15)<1e-12 && std::abs(c[3]-.15)<1e-12);
  assert(b.clearance(0.,0.)>.149 && b.clearance(0.,.2)<0.);
  assert(b.interval_safe(0.,0.,0.,.5,0.,0.,.2));
  // Both ends of the arc are at y=.145, but its middle leaves the safe strip.
  assert(!b.interval_safe(0.,.145,.2,.5,0.,-1.,.4));
  using namespace antbot_tracking;
  Motion ego{0.,.05,M_PI/2.,.3,0.};Neighbor peer{5.,5.,0.,0.,1.15};
  assert(wall_clearance(ego,.5,0.,b,.25,.5,.5,.5)<0.);
  const auto choice=filter_command(ego,.5,0.,{peer},.25,.5,.5,.5,0.,0.,b);
  assert(choice && choice->v<.5 && wall_clearance(ego,choice->v,choice->w,b,.25,.5,.5,.5)>=0.);
  ego.y=.12; // Even braking immediately cannot stop within the remaining strip.
  assert(!filter_command(ego,.5,0.,{peer},.25,.5,.5,.5,0.,0.,b));
  assert(wall_clearance(Motion{0.,.15,0.,0.,0.},0.,0.,b,.25,.5,.5,.5)>-1e-12);
  auto sample=[](double t){return AntbotExecution::Motion{0.,.09+.3*t,M_PI/2.,.3,0.,0.};};
  auto noise=[](double){return 0.;};
  const AntbotExecution::Motion measured{0.,.12,M_PI/2.,.3,0.,0.};
  assert(AntbotExecution::validate_remainder(.1,.2,6.,measured,5.,5.,0.,0.,1.15,1.,.5,.8,.5,sample,noise));
  assert(!AntbotExecution::validate_remainder(.1,.2,6.,measured,5.,5.,0.,0.,1.15,1.,.5,.8,.5,sample,noise,
    [&](double x,double y,double curve){return b.clearance(x,y)>=curve;}));
  bool invalid=false;try{b.configure(true,{-4.,4.,-.4,.4},.5,.05);}catch(const std::invalid_argument &){invalid=true;}
  assert(invalid);
}
'''
        with tempfile.TemporaryDirectory() as temp:
            cpp, exe = Path(temp)/'check.cpp', Path(temp)/'check'
            cpp.write_text(source)
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-I', str(ROOT/'path_following_controller/src'),
                            '-I', str(ROOT/'ros_messages/antbot_interfaces/include'),
                            '-I', str(ROOT/'mpc_planning/mpc_planner_antbot/src'),
                            str(cpp), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
