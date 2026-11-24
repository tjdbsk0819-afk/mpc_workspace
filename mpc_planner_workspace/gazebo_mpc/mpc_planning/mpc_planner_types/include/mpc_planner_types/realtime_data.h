#ifndef MPC_REALTIME_DATA_TYPES_H
#define MPC_REALTIME_DATA_TYPES_H

#include <mpc_planner_types/data_types.h>

#include <chrono>
#include <array>

namespace costmap_2d
{
    class Costmap2D;
}

namespace MPCPlanner
{

    struct RealTimeData
    {

        std::vector<Disc> robot_area;
        FixedSizeTrajectory past_trajectory;

        std::vector<DynamicObstacle> dynamic_obstacles;
        // Center-position limits; wide defaults retain unbounded A/B behavior.
        std::array<double,4> workspace_bounds{{-2000.,2000.,-2000.,2000.}};
        struct StaticObstacleData {
            bool enabled{false};
            double x{0.},y{0.},separation{1.};
        } static_obstacle;

        // Optional AntBot OCP module; other planner configurations ignore it.
        struct YawContinuityData {
            std::vector<double> reference, weights;
            bool first_enabled{false};
            double first_lower{0.}, first_upper{0.};
        } yaw_continuity;

        struct ArrivalData {
            double x{0.}, y{0.}, yaw{0.};
            bool align{false};
        } arrival;

        costmap_2d::Costmap2D *costmap{nullptr}; // Costmap for static obstacles

        ReferencePath reference_path;
        Boundary left_bound, right_bound;

        Eigen::Vector2d goal;
        bool goal_received{false};

        // Feedback data
        double intrusion;

        std::chrono::system_clock::time_point planning_start_time;

        RealTimeData() = default;

        void reset()
        {
            // Copy data that should remain at reset
            std::vector<Disc> robot_area_copy = robot_area;

            *this = RealTimeData();

            robot_area = robot_area_copy;
            goal_received = false;
        }
    };

}
#endif
