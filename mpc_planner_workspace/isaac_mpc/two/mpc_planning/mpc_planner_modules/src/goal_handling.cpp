#include <mpc_planner_modules/goal_handling.h>
#include <mpc_planner_util/parameters.h>
namespace MPCPlanner {
void GoalHandling::setParameters(const RealTimeData &data, const ModuleData &, int k) {
  const auto &goal = data.arrival;
  _solver->setParameter(k, "arrival_x", goal.x);
  _solver->setParameter(k, "arrival_y", goal.y);
  _solver->setParameter(k, "arrival_yaw", goal.yaw);
  _solver->setParameter(k, "arrival_align", goal.align ? 1. : 0.);
  // Stage zero remains the measured state. Subsequent states stop translating
  // during final heading alignment; obstacle constraints remain active.
  _solver->setParameter(k, "arrival_speed_cap", goal.align ? 0. : CONFIG["antbot_limits"]["speed"].as<double>());
}
}
