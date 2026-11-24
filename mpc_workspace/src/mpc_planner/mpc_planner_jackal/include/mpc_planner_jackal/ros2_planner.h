#ifndef JACKAL_PLANNER_H
#define JACKAL_PLANNER_H

#include <mpc_planner/planner.h>

#include <mpc_planner_solver/solver_interface.h>

#include <mpc_planner_types/realtime_data.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <ros_tools/profiling.h>

#include <mpc_planner_jackal/trajectory_sampling.h>

#include <memory>
#include <chrono>

using namespace MPCPlanner;

class JackalPlanner : public rclcpp::Node
{
public:
    JackalPlanner();

    void initializeSubscribersAndPublishers();

    void Loop();

    void stateCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void pathCallback(nav_msgs::msg::Path::SharedPtr msg);

private:
    std::unique_ptr<Planner> _planner;

    RealTimeData _data;
    State _state;
    bool _state_received{false};
    bool _empty_world_test{false};
    nav_msgs::msg::Odometry::SharedPtr _peer_state;
    std::chrono::steady_clock::time_point _last_peer_time;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _peer_sub;
    bool _peer_enabled{false};
    double _peer_offset_x{0.0}, _peer_offset_y{0.0}, _peer_yaw{0.0};
    double _peer_radius{0.5};
    bool _trajectory_exchange{false};
    double _trajectory_timeout{0.5};
    std::vector<AntbotMPC::TimedPose> _peer_trajectory;
    std::chrono::steady_clock::time_point _last_trajectory_time;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _peer_trajectory_sub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _trajectory_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _peer_prediction_pub;
    std::chrono::steady_clock::time_point _last_state_time;

    rclcpp::TimerBase::SharedPtr _timer;

    std::unique_ptr<RosTools::Benchmarker> _benchmarker;

    // Subscribers and publishers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _state_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _goal_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr _path_sub;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _peer_pose_pub;

    bool isPathTheSame(nav_msgs::msg::Path::SharedPtr path);

    void publishTrajectory(bool stationary, const rclcpp::Time &stamp);
    void publishStop();
    void peerTrajectoryCallback(nav_msgs::msg::Path::SharedPtr msg);
    bool loadPeerPrediction(MPCPlanner::Prediction &prediction);
    void visualize();
};

#endif // JACKAL_PLANNER_H
