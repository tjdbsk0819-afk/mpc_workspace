#ifndef ANTBOT_SWERVE_CONTROLLER__JOINT_TRACKING_GUARD_HPP_
#define ANTBOT_SWERVE_CONTROLLER__JOINT_TRACKING_GUARD_HPP_

#include <cmath>
#include <limits>

namespace antbot::swerve_drive_controller
{
// Timers advance on NEW physics samples, never repeated controller callbacks.
class JointTrackingGuard
{
public:
  enum class Decision {TRACK, HOLD_STOP, RESTART};
  double error_duration{0.12};
  double stopped_duration{0.10};

  void reset()
  {
    stopping_ = false;
    last_sample_ = bad_since_ = stopped_since_ = stop_started_ = unset();
  }
  bool stopping() const {return stopping_;}
  double stop_time() const {return stop_started_;}
  void stop(double time)
  {
    if (stopping_) {return;}
    stopping_ = true;
    stop_started_ = time;
    bad_since_ = stopped_since_ = unset();
  }
  Decision observe(double sample_time, bool matched_command, bool error,
                   bool zero_command_applied, bool physically_stopped)
  {
    if (!std::isfinite(sample_time)) {return Decision::HOLD_STOP;}
    if (std::isfinite(last_sample_) && sample_time < last_sample_ - 1e-9) {
      reset(); stop(sample_time);
    }
    if (std::isfinite(last_sample_) && sample_time <= last_sample_ + 1e-9) {
      return stopping_ ? Decision::HOLD_STOP : Decision::TRACK;
    }
    last_sample_ = sample_time;
    if (stopping_) {
      if (matched_command && zero_command_applied && physically_stopped &&
          sample_time > stop_started_ + 1e-9) {
        if (!std::isfinite(stopped_since_)) {stopped_since_ = sample_time;}
        if (sample_time - stopped_since_ + 1e-9 >= stopped_duration) {
          stopping_ = false;
          bad_since_ = stopped_since_ = unset();
          return Decision::RESTART;
        }
      } else {stopped_since_ = unset();}
      return Decision::HOLD_STOP;
    }
    if (matched_command && error) {
      if (!std::isfinite(bad_since_)) {bad_since_ = sample_time;}
      if (sample_time - bad_since_ + 1e-9 >= error_duration) {
        stop(sample_time);
        return Decision::HOLD_STOP;
      }
    } else {bad_since_ = unset();}
    return Decision::TRACK;
  }
private:
  static double unset() {return std::numeric_limits<double>::quiet_NaN();}
  bool stopping_{false};
  double last_sample_{unset()}, bad_since_{unset()}, stopped_since_{unset()}, stop_started_{unset()};
};
}  // namespace antbot::swerve_drive_controller
#endif
