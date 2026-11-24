#pragma once
#include <mpc_planner_modules/controller_module.h>
namespace MPCPlanner {
class WorkspaceBounds : public ControllerModule {
public:
  explicit WorkspaceBounds(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::CONSTRAINT, solver, "workspace_bounds") {}
  void setParameters(const RealTimeData &data, const ModuleData &, int k) override;
};
}
