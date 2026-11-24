#include <mpc_planner_jackal/trajectory_sampling.h>
#include <cstdlib>
#include <limits>

void check(bool condition)
{
    if (!condition)
        std::abort();
}

int main()
{
    using namespace AntbotMPC;
    constexpr double pi = 3.14159265358979323846;
    const std::vector<TimedPose> poses = {{10, 0, 0, 170 * pi / 180}, {12, 2, 4, -170 * pi / 180}};
    TimedPose sampled;
    check(sampleTrajectory(poses, 11, 0.5, sampled));
    check(std::abs(sampled.x - 1) < 1e-9 && std::abs(sampled.y - 2) < 1e-9);
    check(std::abs(std::cos(sampled.yaw) + 1) < 1e-9); // Short rotation through pi.
    const auto transformed = transformPose(sampled, 6, 0, pi);
    check(std::abs(transformed.x - 5) < 1e-9 && std::abs(transformed.y + 2) < 1e-9);
    check(sampleTrajectory(poses, 12.5, 0.5, sampled));
    check(std::abs(sampled.x - 2.5) < 1e-9);
    check(!sampleTrajectory(poses, 12.50001, 0.5, sampled));
    check(!sampleTrajectory(poses, 9.9, 0.5, sampled));
    check(!sampleTrajectory(poses, 11, -1, sampled));
    check(!sampleTrajectory({}, 11, 1, sampled));
    const std::vector<TimedPose> stopped = {{10, 4, 5, 0.3}, {11, 4, 5, 0.3}};
    check(sampleTrajectory(stopped, 11.4, 0.5, sampled));
    check(sampled.x == 4 && sampled.y == 5);
    auto duplicate = poses;
    duplicate[1].time = duplicate[0].time;
    check(!validTrajectory(duplicate));
    auto invalid = poses;
    invalid[1].x = std::numeric_limits<double>::quiet_NaN();
    check(!validTrajectory(invalid));
    check(!sampleTrajectory(poses, std::numeric_limits<double>::infinity(), 1, sampled));
}
