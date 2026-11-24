#include <mpc_planner_modules/yaw_rate_envelope.h>
#include <mpc_planner_util/parameters.h>
#include <stdexcept>
#include <cmath>
namespace MPCPlanner {
void YawRateEnvelope::setParameters(const RealTimeData &data, const ModuleData &, int k) {
  const auto &yaw = data.yaw_continuity;
  const double limit = CONFIG["antbot_limits"]["yaw_rate"].as<double>();
  const double lower = k == 0 && yaw.first_enabled ? yaw.first_lower : -limit;
  const double upper = k == 0 && yaw.first_enabled ? yaw.first_upper : limit;
  const double reference = k < static_cast<int>(yaw.reference.size()) ? yaw.reference[k] : 0.;
  const double weight = k < static_cast<int>(yaw.weights.size()) ? yaw.weights[k] : 0.;
  if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper ||
      !std::isfinite(reference) || !std::isfinite(weight) || weight < 0.)
    throw std::invalid_argument("Invalid yaw continuity parameters");
  _solver->setParameter(k, "yaw_reference", reference);
  _solver->setParameter(k, "yaw_continuity_weight", weight);
  _solver->setParameter(k, "yaw_lower", lower);
  _solver->setParameter(k, "yaw_upper", upper);
}
}
