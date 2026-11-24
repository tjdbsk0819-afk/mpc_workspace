// Exercise the actual vendored RPP sampler without requiring ROS/Isaac.
#include "../isaac/path_following_controller/src/mpc_tracking.hpp"
#include <iostream>
#include <stdexcept>

int main() {
  if(!antbot_tracking::use_rotation_profile(.017,-.083,0.) ||
     !antbot_tracking::use_rotation_profile(.007,-.054,.002) ||
     antbot_tracking::use_rotation_profile(.1,-.083,0.) ||
     antbot_tracking::use_rotation_profile(.035,-.083,0.) ||
     antbot_tracking::use_rotation_profile(.017,-.083,.1) ||
     antbot_tracking::use_rotation_profile(.017,.01,0.))
    throw std::runtime_error("Near-rest MPC turns must use pure rotation without changing moving/straight commands");
  antbot_tracking::Trajectory trajectory;
  int n;
  double dt;
  if (!(std::cin >> n >> dt) || n < 2) return 2;
  for (int k = 0; k < n; ++k) {
    double x, y, yaw, v, a, w;
    std::cin >> x >> y >> yaw >> v >> a >> w;
    trajectory.times.push_back(k*dt);
    trajectory.poses.push_back({x,y,yaw});
    trajectory.velocities.push_back(v);
    trajectory.accelerations.push_back(a);
    trajectory.rates.push_back(w);
  }
  trajectory.validate();
  for (int k = 0; k < n-1; ++k) {
    double x, y, yaw, v;
    std::cin >> x >> y >> yaw >> v;
    const auto p = trajectory.sample((k+.5)*dt);
    if (std::abs(x-p.x)>1e-7 || std::abs(y-p.y)>1e-7 ||
        std::abs(yaw-p.yaw)>1e-7 || std::abs(v-p.v)>1e-7)
      throw std::runtime_error("Python MPC and native RPP time sampling disagree");
  }
  return std::cin ? 0 : 3;
}
