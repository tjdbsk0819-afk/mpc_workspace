"""Random-walk mean, covariance, CV comparison and probability-aware guard."""
from pathlib import Path
import json
import math
import subprocess
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'controller_bringup'))
from antbot_mpc_ros.cv_model import WorldPlanarState
from antbot_mpc_ros.peer_prediction import distribution
from antbot_mpc_ros.peer_prediction_viewer import TopicStore


class GaussianPredictionTests(unittest.TestCase):
    def test_cpp_python_mean_variance_and_guard(self):
        source=r'''
#include "mpc_tracking.hpp"
#include <cassert>
#include <iomanip>
#include <iostream>
int main(){
 using namespace antbot_prediction;
 Model rw{"gaussian_random_walk",.3,.2,.05};rw.validate();
 Model cv{"cv_gaussian",.3,.2,.05};cv.validate();
 assert(!rw.uses_velocity() && cv.uses_velocity());
 for(double t:{0.,.1,.2,.6,6.}){
   const auto m=rw.mean(3.,2.,100.,-90.,t);
   assert(m[0]==3. && m[1]==2.);
   std::cout<<std::setprecision(17)<<rw.sigma(t)<<' '<<rw.radius(t)<<'\n';
 }
 using namespace antbot_tracking;
 Motion ego{0.,0.,0.,0.,0.};Neighbor n{1.25,0.,-100.,0.,1.15};
 // Nominal stationary obstacle permits hold; Gaussian growth can reject it.
 n.vx=0.;assert(clearance(ego,0.,0.,n,.25,.5,.5,.5)>n.separation);
 n.forecast=rw;assert(clearance(ego,0.,0.,n,.25,.5,.5,.5)<n.separation);
 Neighbor moving{4.,0.,-100.,0.,1.15,rw,0.};
 assert(clearance(ego,.2,0.,moving,.25,.5,.5,.5)>moving.separation);
 moving.forecast=Model{};
 assert(clearance(ego,.2,0.,moving,.25,.5,.5,.5)<moving.separation);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            src=Path(tmp)/'check.cpp';exe=Path(tmp)/'check';src.write_text(source)
            subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',
                            '-I',str(ROOT/'path_following_controller/src'),
                            '-I',str(ROOT/'ros_messages/antbot_interfaces/include'),
                            str(src),'-o',str(exe)],check=True)
            rows=subprocess.run([str(exe)],check=True,capture_output=True,text=True).stdout.splitlines()
        for t,row in zip((0.,.1,.2,.6,6.),rows):
            sigma,radius=map(float,row.split())
            self.assertAlmostEqual(sigma,.3*math.sqrt(.2*t))
            self.assertAlmostEqual(radius,math.sqrt(-2*math.log(.05))*sigma)
        state=WorldPlanarState(1_000_000_000,'robot_b','world',3.,2.,100.,-90.)
        p=distribution(state,1_100_000_000,200_000_000,30,'gaussian_random_walk')
        self.assertTrue(all((v['x'],v['y'])==(3.,2.) for v in p))
        self.assertTrue(all(a['sigma']<b['sigma'] for a,b in zip(p,p[1:])))
        self.assertAlmostEqual(p[0]['sigma'],.3*math.sqrt(.2*.1))
        self.assertEqual(state.vx,100.) # state telemetry is preserved
        cv=distribution(state,1_100_000_000,200_000_000,30,'cv_gaussian')
        self.assertAlmostEqual(cv[0]['x'],13.)

    def test_distribution_viewer_invalidates_stale_or_invalid_data(self):
        store=TopicStore(robots=('robot_a','robot_b'))
        value=dict(model='gaussian_random_walk',frame='world',steps=[dict(t=.2,x=1.,y=2.,sigma=.06,radius=.1469)])
        store.update_distribution('robot_a',json.dumps(value))
        self.assertEqual(store.snapshot()['distributions']['robot_a']['steps'][0]['x'],1.)
        store.update_distribution('robot_a','[]')
        self.assertNotIn('robot_a',store.snapshot()['distributions'])
        store.update_distribution('robot_a',json.dumps({**value,'steps':[]}))
        self.assertFalse(store.snapshot()['distributions']['robot_a']['steps'])

    def test_bad_models_and_parameters_are_rejected(self):
        state=WorldPlanarState(0,'robot_b','world',0.,0.,1.,0.)
        for model,noise,risk in [('unknown',.3,.05),('gaussian_random_walk',0.,.05),('gaussian_random_walk',.3,1.)]:
            with self.assertRaises(ValueError):distribution(state,0,200_000_000,30,model,noise,risk)
