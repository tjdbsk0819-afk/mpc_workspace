"""Exercise arrival latching and old-plan validation against changing neighbors."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ExecutionPolicyTests(unittest.TestCase):
    def test_arrival_and_revalidated_fallback(self):
        source = r'''
#include "execution_policy.h"
#include <cassert>
using namespace AntbotExecution;
int main() {
  Arrival goal;
  goal.update(1.,.1,.3,.5,0.,.2,.05,.3);
  assert(!goal.aligning && !goal.reached); // cannot finish at driving speed
  goal.update(2.,.1,.3,0.,0.,.2,.05,.3);
  assert(goal.aligning && !goal.reached); // must resolve heading
  goal.update(3.,.1,.01,0.,0.,.2,.05,.3);
  goal.update(3.31,.1,.01,0.,0.,.2,.05,.3);
  assert(goal.reached);
  goal.update(4.,.4,1.,0.,0.,.2,.05,.3);
  assert(goal.reached); // drift never restarts a completed mission
  goal.update(0.,.4,1.,0.,0.,.2,.05,.3);
  assert(!goal.reached && !goal.aligning); // new clock episode
  goal.reset();
  goal.update(1.,.1,2.*M_PI-.01,0.,0.,.2,.05,.3);
  goal.update(1.31,.1,2.*M_PI-.01,0.,0.,.2,.05,.3);
  assert(goal.reached); // periodic final yaw

  auto sample=[](double t){return Motion{.4*t,0.,0.,.4,0.,0.};};
  auto none=[](double){return 0.;};
  auto noise=[](double){return .2;};
  const Motion measured{.04,0.,0.,.4,0.,0.};
  auto check=[&](double age,Motion current,double x,double y,double vx,double vy,auto inflation) {
    return validate_remainder(age,.2,2.,current,x,y,vx,vy,1.2,1.,.5,.8,.5,sample,inflation);
  };
  assert(check(.1,measured,5.,5.,0.,0.,noise));
  assert(check(.1,Motion{.04,0.,0.,.32,0.,0.},5.,5.,0.,0.,noise));
  // Small speed tracking error converges over the actual short execution
  // prefix; it must not demand an identical stale terminal stop six seconds later.
  assert(!check(.201,measured,5.,5.,0.,0.,noise)); // republish cannot renew epoch
  assert(!check(-.01,measured,5.,5.,0.,0.,noise));
  assert(!check(.1,Motion{.2,0.,0.,.4,0.,0.},5.,5.,0.,0.,none));
  assert(!check(.1,Motion{.04,0.,.3,.4,0.,0.},5.,5.,0.,0.,none));
  assert(check(.1,measured,.4,1.3,0.,0.,none));
  assert(!check(.1,measured,.4,1.3,0.,0.,noise)); // nominal-only safety is insufficient
  assert(!check(.1,measured,2.,0.,-1.,0.,none)); // new incoming peer invalidates old path
  assert(!check(.1,Motion{.04,0.,0.,.6,0.,0.},5.,5.,0.,0.,none));
  const double nan=std::numeric_limits<double>::quiet_NaN();
  assert(!check(.1,Motion{nan,0.,0.,.4,0.,0.},5.,5.,0.,0.,none));
  assert(!check(.1,measured,5.,5.,0.,0.,[&](double){return nan;}));
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            cpp, exe = Path(tmp)/'check.cpp', Path(tmp)/'check'
            cpp.write_text(source)
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-I', str(ROOT/'mpc_planning/mpc_planner_antbot/src'),
                            str(cpp), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)
