#include <mpc_planner_modules/static_obstacle.h>
#include <cmath>
#include <stdexcept>
namespace MPCPlanner {
void StaticCircleConstraints::setParameters(const RealTimeData &data, const ModuleData &, int k) {
  const auto &s=data.static_obstacle;
  if(!std::isfinite(s.x) || !std::isfinite(s.y) || !std::isfinite(s.separation) || s.separation<=0.)
    throw std::invalid_argument("Invalid OCP static obstacle");
  _solver->setParameter(k,"static_enabled",s.enabled?1.:0.);
  _solver->setParameter(k,"static_x",s.x);
  _solver->setParameter(k,"static_y",s.y);
  _solver->setParameter(k,"static_separation",s.separation);
}
}
