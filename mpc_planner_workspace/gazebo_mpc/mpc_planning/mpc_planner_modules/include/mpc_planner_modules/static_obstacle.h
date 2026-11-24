#pragma once
#include <mpc_planner_modules/controller_module.h>
namespace MPCPlanner {
class StaticCircleConstraints : public ControllerModule {
public:
  explicit StaticCircleConstraints(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::CONSTRAINT, solver, "static_obstacle") {}
  void setParameters(const RealTimeData &data, const ModuleData &, int k) override;
};
}
