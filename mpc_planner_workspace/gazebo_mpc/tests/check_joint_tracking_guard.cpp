#include "antbot_swerve_controller/joint_tracking_guard.hpp"
#include <cassert>
#include <iostream>

using antbot::swerve_drive_controller::JointTrackingGuard;
using D = JointTrackingGuard::Decision;

int main()
{
  JointTrackingGuard guard;
  // A short lag/error spike must not reset a moving profile.
  assert(guard.observe(1., true, true, false, false) == D::TRACK);
  assert(guard.observe(1.08, true, false, false, false) == D::TRACK);
  // Controller callbacks on the SAME feedback cannot mature an error timer.
  for (int i=0; i<1000; ++i) {
    assert(guard.observe(1.08, true, true, false, false) == D::TRACK);
  }
  assert(guard.observe(2., true, true, false, false) == D::TRACK);
  assert(guard.observe(2.10, true, true, false, false) == D::TRACK);
  assert(guard.observe(2.12, true, true, false, false) == D::HOLD_STOP);
  // Neither a zero command alone nor a stopped sample with an old moving
  // applied command authorizes restart.
  assert(guard.observe(2.2, true, true, true, false) == D::HOLD_STOP);
  assert(guard.observe(2.4, true, false, false, true) == D::HOLD_STOP);
  assert(guard.observe(2.6, false, false, true, true) == D::HOLD_STOP);
  assert(guard.observe(2.8, true, false, true, true) == D::HOLD_STOP);
  for (int i=0; i<1000; ++i) {
    assert(guard.observe(2.8, true, false, true, true) == D::HOLD_STOP);
  }
  // A new non-stopped sample restarts the stopped dwell.
  assert(guard.observe(2.88, true, false, true, false) == D::HOLD_STOP);
  assert(guard.observe(3., true, false, true, true) == D::HOLD_STOP);
  assert(guard.observe(3.10, true, false, true, true) == D::RESTART);
  assert(!guard.stopping());
  // Old snapshots after a simulation reset cannot reuse a previous dwell.
  assert(guard.observe(.1, true, false, true, true) == D::HOLD_STOP);
  assert(guard.observe(.12, true, false, true, true) == D::HOLD_STOP);
  assert(guard.observe(.22, true, false, true, true) == D::RESTART);
  std::cout << "PASS: transient error, duplicate samples, persistent fault, applied stop, physical stop dwell, clock reset\n";
}
