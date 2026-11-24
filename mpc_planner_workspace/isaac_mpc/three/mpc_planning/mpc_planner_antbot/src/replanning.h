#pragma once

// Homogeneous recovery policy. No robot IDs, priorities, peer intentions or commands.
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace AntbotRecovery
{
using Guess = std::vector<std::map<std::string, double>>;

struct ProgressMonitor
{
  double window{3.}, minimum_progress{.08}, cooldown{6.};
  double baseline{std::numeric_limits<double>::infinity()};
  double since{-1.}, last_attempt{-1e9}, previous_time{-1.};

  void reset() { baseline = std::numeric_limits<double>::infinity(); since = previous_time = -1.; last_attempt = -1e9; }
  void invalidate() { baseline = std::numeric_limits<double>::infinity(); since = previous_time = -1.; }

  bool update(double now, double goal_distance, bool eligible)
  {
    if (!eligible || !std::isfinite(goal_distance) || (previous_time >= 0. && now < previous_time))
    { invalidate(); return false; }
    previous_time = now;
    if (since < 0. || baseline-goal_distance >= minimum_progress)
    { baseline = goal_distance; since = now; return false; }
    if (now-since < window || now-last_attempt < cooldown) return false;
    since = now; baseline = goal_distance; last_attempt = now;
    return true;
  }
};

inline double angle(double value) { return std::atan2(std::sin(value), std::cos(value)); }

// Dynamically integrated unicycle initial guesses, NOT executable paths. Both
// signs are tried by every robot. The optimizer alone decides the final trajectory.
inline Guess make_guess(double x, double y, double yaw, double speed, double goal_x,
                        double goal_y, double peak, int n, double dt,
                        double max_speed, double max_acceleration, double max_yaw_rate)
{
  Guess guess;
  double s = 0.;
  const double direction = yaw + angle(std::atan2(goal_y-y, goal_x-x)-yaw);
  const double heading = yaw;
  for (int k = 0; k <= n; ++k)
  {
    const double t = k*dt;
    const double blend = std::min(1., t/1.5);
    const double desired = heading + blend*(direction-heading) + peak*std::sin(M_PI*t/(n*dt));
    const double w = std::clamp(2.*angle(desired-yaw), -max_yaw_rate, max_yaw_rate);
    const double a = std::clamp((max_speed-speed)/dt, -max_acceleration, max_acceleration);
    guess.push_back({{"x",x},{"y",y},{"psi",yaw},{"v",speed},{"spline",s},{"a",a},{"w",w}});
    // RK4 integration of x'=v cos(psi), y'=v sin(psi), v'=a, psi'=w, s'=v.
    const double mid_v = speed+.5*a*dt, end_v = speed+a*dt;
    x += dt/6.*(speed*std::cos(yaw)+4.*mid_v*std::cos(yaw+.5*w*dt)+end_v*std::cos(yaw+w*dt));
    y += dt/6.*(speed*std::sin(yaw)+4.*mid_v*std::sin(yaw+.5*w*dt)+end_v*std::sin(yaw+w*dt));
    s += speed*dt+.5*a*dt*dt; speed = end_v; yaw += w*dt;
  }
  return guess;
}

// Cost hysteresis only, shared by all robots. A previous candidate survives
// tiny cost differences; an infeasible previous candidate is never retained.
inline int select(const std::vector<double> &costs, int previous, double switch_cost)
{
  int best = -1;
  for (size_t i = 0; i < costs.size(); ++i)
    if (std::isfinite(costs[i]) && (best < 0 || costs[i] < costs[best])) best = static_cast<int>(i);
  if (best >= 0 && previous >= 0 && previous < static_cast<int>(costs.size()) &&
      std::isfinite(costs[previous]) && costs[previous] <= costs[best]+switch_cost) return previous;
  return best;
}
}
