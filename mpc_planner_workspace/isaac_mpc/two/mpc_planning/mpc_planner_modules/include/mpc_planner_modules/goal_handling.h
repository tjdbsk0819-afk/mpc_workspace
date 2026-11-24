#pragma once
#include <mpc_planner_modules/controller_module.h>
namespace MPCPlanner {
class GoalHandling : public ControllerModule {
public:
  explicit GoalHandling(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::CONSTRAINT, solver, "goal_handling") {}
  void setParameters(const RealTimeData &data, const ModuleData &, int k) override;
};
}
