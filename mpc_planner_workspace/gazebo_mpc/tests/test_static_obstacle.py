"""D scenario geometry and deterministic static-circle motion/braking checks."""
from pathlib import Path
import json
import subprocess
import sys
import tempfile
import unittest
from simulation.helpers.scenario_config import scenario_layout, planner_goals, workspace_parameters

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'controller_bringup'))
from antbot_mpc_ros.peer_prediction_viewer import TopicStore


class StaticObstacleTests(unittest.TestCase):
    def test_shared_scene_and_goal_safety(self):
        self.assertEqual(scenario_layout('D').obstacles,((0.,0.,.4),))
        params=workspace_parameters('D')
        self.assertTrue(params['static_obstacle_enabled'])
        self.assertFalse(params['workspace_bounds_enabled'])
        self.assertEqual(params['static_obstacle'],[0.,0.,.4])
        self.assertEqual(planner_goals('D')['robot_b_goal_x'],-3.)
        with self.assertRaises(ValueError):planner_goals('D',{'robot_a_goal_x':'0'})
        for name in ('A','B','C'):
            self.assertFalse(workspace_parameters(name)['static_obstacle_enabled'])

    def test_between_knots_and_braking_collision(self):
        source=r'''
#include "mpc_tracking.hpp"
#include "execution_policy.h"
#include <cassert>
int main(){
  using namespace antbot_tracking;
  antbot_workspace::StaticCircle s;s.configure(true,{0.,0.,.4},.5,.15);
  assert(std::abs(s.separation()-1.05)<1e-12 && s.clearance(0.,0.)<0.);
  assert(s.clearance(-1.2,0.)>0. && s.clearance(1.2,0.)>0.);
  assert(s.segment_clearance(-1.2,0.,1.2,0.)<0.); // safe endpoints, unsafe segment
  assert(!s.interval_safe(-1.2,0.,0.,1.,0.,0.,2.4));
  assert(s.interval_safe(-1.2,1.2,0.,.5,0.,0.,.2,.05));
  Neighbor fixed{s.x,s.y,0.,0.,s.separation()+.005};
  assert(!fixed.forecast.gaussian()); // no Gaussian inflation for a known fixed object
  Motion moving{-1.1,0.,0.,.4,0.};
  assert(!filter_command(moving,.4,0.,{fixed},.25,.5,.5,.5,0.,0.));
  Motion safe{-2.,1.3,0.,.2,0.};
  assert(filter_command(safe,.2,0.,{fixed},.25,.5,.5,.5,0.,0.));
  auto sample=[](double t){return AntbotExecution::Motion{-1.23+.3*t,0.,0.,.3,0.,0.};};
  auto noise=[](double){return 0.;};
  const AntbotExecution::Motion measured{-1.2,0.,0.,.3,0.,0.};
  assert(AntbotExecution::validate_remainder(.1,.2,6.,measured,5.,5.,0.,0.,1.15,1.,.5,.8,.5,sample,noise));
  assert(!AntbotExecution::validate_remainder(.1,.2,6.,measured,5.,5.,0.,0.,1.15,1.,.5,.8,.5,sample,noise,
    [&,px=measured.x,py=measured.y](double x,double y,double curve) mutable {
      const bool valid=s.segment_clearance(px,py,x,y)>=curve;px=x;py=y;return valid;
    }));
  s.enabled=false;assert(s.interval_safe(-1.2,0.,0.,1.,0.,0.,2.4));
}
'''
        with tempfile.TemporaryDirectory() as temp:
            cpp,exe=Path(temp)/'check.cpp',Path(temp)/'check';cpp.write_text(source)
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',
                '-I',str(ROOT/'path_following_controller/src'),'-I',str(ROOT/'ros_messages/antbot_interfaces/include'),
                '-I',str(ROOT/'mpc_planning/mpc_planner_antbot/src'),
                str(cpp),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

    def test_viewer_static_geometry_validation(self):
        store=TopicStore();circle=dict(x=0.,y=0.,radius=.4,clearance_radius=1.05,planning_radius=1.1)
        value=dict(model='gaussian_random_walk',frame='world',steps=[],static_obstacles=[circle])
        store.update_distribution('robot_a',json.dumps(value))
        self.assertEqual(store.snapshot()['distributions']['robot_a']['static_obstacles'][0]['radius'],.4)
        circle['radius']=-1.;store.update_distribution('robot_a',json.dumps(value))
        self.assertNotIn('robot_a',store.snapshot()['distributions'])
