#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace AntbotYaw {
// Previous selected OCP controls are piecewise constant over their stage.
// Average overlapping stages instead of comparing equal indices after replanning.
struct History {
  double stamp{0.}, dt{0.};
  std::vector<double> rates;
  void clear() { rates.clear(); }
  void store(double now, double step, const std::vector<double> &values) {
    stamp = now; dt = step; rates = values;
  }
  bool average(double now, double offset, double duration, double timeout, double &value) const {
    const double age = now-stamp, begin = age+offset, end = begin+duration;
    if (rates.empty() || !std::isfinite(age) || !std::isfinite(offset) ||
        !std::isfinite(duration) || !std::isfinite(timeout) || age < -1e-9 || age > timeout ||
        dt <= 0. || duration <= 0. || begin < -1e-9 || end > rates.size()*dt+1e-9) return false;
    double integral = 0.;
    for (size_t i=0; i<rates.size(); ++i) {
      const double overlap = std::max(0., std::min(end,(i+1)*dt)-std::max(begin,i*dt));
      if (overlap > 0.) {
        if (!std::isfinite(rates[i])) return false;
        integral += overlap*rates[i];
      }
    }
    value = integral/duration;
    return std::isfinite(value);
  }
};
}
