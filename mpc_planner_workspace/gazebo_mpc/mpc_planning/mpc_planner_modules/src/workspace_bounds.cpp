#include <mpc_planner_modules/workspace_bounds.h>
#include <cmath>
#include <stdexcept>
namespace MPCPlanner {
void WorkspaceBounds::setParameters(const RealTimeData &data, const ModuleData &, int k) {
  const auto &b=data.workspace_bounds;
  if(!std::isfinite(b[0]) || !std::isfinite(b[1]) || !std::isfinite(b[2]) || !std::isfinite(b[3]) ||
     b[0]>=b[1] || b[2]>=b[3]) throw std::invalid_argument("Invalid OCP workspace bounds");
  _solver->setParameter(k,"workspace_xmin",b[0]);
  _solver->setParameter(k,"workspace_xmax",b[1]);
  _solver->setParameter(k,"workspace_ymin",b[2]);
  _solver->setParameter(k,"workspace_ymax",b[3]);
}
}
