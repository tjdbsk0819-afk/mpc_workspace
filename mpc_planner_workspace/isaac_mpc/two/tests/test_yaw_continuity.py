"""Exercise the production C++ previous-plan overlap/time policy without ROS."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class YawContinuityTests(unittest.TestCase):
    def test_time_alignment_expiry_and_mirrored_controls(self):
        source = r'''
#include "yaw_continuity.h"
#include <cassert>
#include <cmath>
int main() {
  AntbotYaw::History h; double value=999.;
  assert(!h.average(10.,0.,.2,.35,value));
  h.store(10.,.2,{.2,-.2,.6});
  assert(h.average(10.,0.,.2,.35,value) && std::abs(value-.2)<1e-12);
  assert(h.average(10.05,0.,.2,.35,value) && std::abs(value-.1)<1e-12);
  assert(h.average(10.1,0.,.2,.35,value) && std::abs(value)<1e-12);
  assert(h.average(10.15,0.,.2,.35,value) && std::abs(value+.1)<1e-12);
  assert(h.average(10.1,.2,.2,.35,value) && std::abs(value-.2)<1e-12);
  assert(!h.average(10.1,.4,.2,.35,value)); // no extrapolation past old horizon
  assert(!h.average(10.36,0.,.2,.35,value));
  assert(!h.average(9.9,0.,.2,.35,value)); // new episode
  assert(!h.average(10.,0.,0.,.35,value));
  AntbotYaw::History mirrored; mirrored.store(10.,.2,{-.2,.2,-.6});
  double other;
  for (double now : {10.,10.05,10.1,10.15}) {
    assert(h.average(now,0.,.2,.35,value));
    assert(mirrored.average(now,0.,.2,.35,other));
    assert(std::abs(value+other)<1e-12);
  }
  h.clear();assert(!h.average(10.,0.,.2,.35,value));
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
