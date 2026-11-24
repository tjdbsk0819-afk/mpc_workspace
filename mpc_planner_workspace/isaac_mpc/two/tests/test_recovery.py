"""Compile the actual C++ recovery policy without ROS; test progress and seed selection."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class RecoveryTests(unittest.TestCase):
    def test_progress_seeds_and_feasible_selection(self):
        source = r'''
#include "replanning.h"
#include <cassert>
#include <iostream>
using namespace AntbotRecovery;
int main() {
  ProgressMonitor m;
  assert(!m.update(0., 3., true));
  assert(!m.update(2.9, 3., true));
  assert(m.update(3., 3., true));
  assert(!m.update(6., 3., true)); // recovery cooldown
  assert(m.update(9., 3., true));
  m.reset();
  for (int k=0;k<100;k++) assert(!m.update(k*.1, 4.-k*.01, true));
  m.reset(); assert(!m.update(0., 3., true));
  assert(!m.update(100., 3., false)); // missing input is not evidence of a stall
  assert(!m.update(101., 3., true));
  assert(!m.update(0., 3., true)); // clock rewind
  assert(!m.update(.1, 3., true));
  assert(m.update(3.1, 3., true));
  std::cout << "PASS: progress, dwell, cooldown, input gaps and clock reset\n";
  for (double peak : {0., .8, -.8, 1.4, -1.4}) {
    const auto a=make_guess(-.7,0.,0.,0.,3.,0.,peak,30,.2,.3,.5,.8);
    const auto b=make_guess(.7,0.,M_PI,0.,-3.,0.,peak,30,.2,.3,.5,.8);
    assert(a.size()==31 && b.size()==31);
    for (size_t k=0;k<a.size();k++) {
      for (const auto &v:a[k]) assert(std::isfinite(v.second));
      assert(std::abs(a[k].at("a"))<=.5+1e-10);
      assert(std::abs(a[k].at("w"))<=.8+1e-10);
      assert(a[k].at("v")>=0. && a[k].at("v")<=.3+1e-10);
      // Rotating the entire problem rotates the seed; no IDs or global left bias.
      assert(std::abs(a[k].at("x")+b[k].at("x"))<1e-9);
      assert(std::abs(a[k].at("y")+b[k].at("y"))<1e-9);
      if(k+1<a.size()) {
        assert(std::abs(a[k+1].at("v")-a[k].at("v")-.2*a[k].at("a"))<1e-10);
        assert(std::abs(a[k+1].at("psi")-a[k].at("psi")-.2*a[k].at("w"))<1e-10);
      }
    }
  }
  const auto l=make_guess(0,0,0,0,3,0,1.4,30,.2,.3,.5,.8);
  const auto r=make_guess(0,0,0,0,3,0,-1.4,30,.2,.3,.5,.8);
  assert(l.back().at("y")>.5 && r.back().at("y")<-.5);
  std::cout << "PASS: mirrored dynamically integrated seeds obey input and speed bounds\n";
  const auto braking=make_guess(2.4,0.,0.,.2,3.,0.,0.,30,.2,1.,.5,.8,.35,.08,.25,.8);
  assert(std::abs(braking.back().at("v"))<.01);
  assert(std::abs(braking.back().at("x")-3.)<.2);
  assert(braking.back().at("x")<3.);
  std::cout << "PASS: goal-aware recovery seed decelerates before the original target\n";
  const double inf=std::numeric_limits<double>::infinity();
  assert(select({inf,inf},0,.05)==-1);
  assert(select({inf,2.},0,.05)==1); // no hysteresis for infeasible previous
  assert(select({2.02,2.},0,.05)==0);
  assert(select({3.,2.},0,.05)==1);
  std::cout << "PASS: feasibility filtering and cost hysteresis\n";
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            cpp, exe = Path(tmp)/'check.cpp', Path(tmp)/'check'
            cpp.write_text(source)
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                            '-I', str(ROOT/'mpc_planning/mpc_planner_antbot/src'),
                            str(cpp), '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    unittest.main()
