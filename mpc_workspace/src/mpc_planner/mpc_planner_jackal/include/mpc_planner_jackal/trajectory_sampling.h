#ifndef ANTBOT_TRAJECTORY_SAMPLING_H
#define ANTBOT_TRAJECTORY_SAMPLING_H

#include <algorithm>
#include <cmath>
#include <vector>

namespace AntbotMPC
{
struct TimedPose
{
    double time;
    double x;
    double y;
    double yaw;
};

inline bool validTrajectory(const std::vector<TimedPose> &poses)
{
    if (poses.size() < 2)
        return false;
    for (size_t i = 0; i < poses.size(); ++i)
    {
        const auto &p = poses[i];
        if (!std::isfinite(p.time) || !std::isfinite(p.x) || !std::isfinite(p.y) ||
            !std::isfinite(p.yaw) || (i > 0 && p.time <= poses[i - 1].time))
            return false;
    }
    return true;
}

// Times are absolute ROS times. Extend the final segment only within a bounded
// tail, to account for transport delay between peers with equal horizons.
inline bool sampleTrajectory(const std::vector<TimedPose> &poses, double time,
                             double max_tail, TimedPose &out)
{
    if (!validTrajectory(poses) || !std::isfinite(time) || !std::isfinite(max_tail) ||
        max_tail < 0.0 || time < poses.front().time || time > poses.back().time + max_tail)
        return false;
    auto upper = std::upper_bound(poses.begin(), poses.end(), time,
        [](double t, const TimedPose &p) { return t < p.time; });
    size_t right = upper == poses.end() ? poses.size() - 1 : upper - poses.begin();
    if (right == 0)
        right = 1;
    const auto &a = poses[right - 1];
    const auto &b = poses[right];
    const double alpha = (time - a.time) / (b.time - a.time);
    const double dyaw = std::atan2(std::sin(b.yaw - a.yaw), std::cos(b.yaw - a.yaw));
    out = {time, a.x + alpha * (b.x - a.x), a.y + alpha * (b.y - a.y), a.yaw + alpha * dyaw};
    return std::isfinite(out.x) && std::isfinite(out.y) && std::isfinite(out.yaw);
}

inline TimedPose transformPose(const TimedPose &p, double x, double y, double yaw)
{
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {p.time, c * p.x - s * p.y + x, s * p.x + c * p.y + y, p.yaw + yaw};
}
} // namespace AntbotMPC
#endif
