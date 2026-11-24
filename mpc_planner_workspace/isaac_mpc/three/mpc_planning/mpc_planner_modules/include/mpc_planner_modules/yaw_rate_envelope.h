#pragma once
#include <mpc_planner_modules/controller_module.h>
namespace MPCPlanner {
class YawRateEnvelope : public ControllerModule {
public:
  explicit YawRateEnvelope(std::shared_ptr<Solver> solver)
      : ControllerModule(ModuleType::CONSTRAINT, solver, "yaw_rate_envelope") {}
  void setParameters(const RealTimeData &data, const ModuleData &, int k) override;
};
}
