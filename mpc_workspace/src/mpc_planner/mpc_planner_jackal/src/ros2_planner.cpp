#include <mpc_planner_jackal/ros2_planner.h>

#include <ros_tools/visuals.h>
#include <mpc_planner_util/parameters.h>
#include <mpc_planner/data_preparation.h>
#include <ros_tools/logging.h>
#include <mpc_planner_util/load_yaml.hpp>

#include <ros_tools/convertions.h>
#include <ros_tools/ros2_wrappers.h>
#include <rclcpp/create_timer.hpp>

#include <algorithm>
#include <cmath>
#include <functional>

using namespace MPCPlanner;
using namespace rclcpp;

JackalPlanner::JackalPlanner() : Node("jackal_planner")
{
    STATIC_NODE_POINTER.init(this);

    LOG_INFO("Started Jackal Planner");

    // Initialize the configuration
    Configuration::getInstance().initialize(SYSTEM_CONFIG_PATH(__FILE__, "settings"));

    CONFIG["diagnostics"] = declare_parameter<bool>("mpc_diagnostics", false);
    _empty_world_test = declare_parameter<bool>("empty_world_test", false);
    const double reference_velocity = declare_parameter<double>("reference_velocity", 0.2);
    const double velocity_weight = declare_parameter<double>(
        "velocity_weight", CONFIG["weights"]["velocity"].as<double>());
    if (!std::isfinite(velocity_weight) || velocity_weight < 0.0)
        throw std::invalid_argument("velocity_weight must be finite and nonnegative");
    CONFIG["weights"]["velocity"] = velocity_weight;
    const auto peer_topic = declare_parameter<std::string>("peer_odom_topic", "");
    _peer_enabled = !peer_topic.empty();
    _peer_offset_x = declare_parameter<double>("peer_offset_x", 0.0);
    _peer_offset_y = declare_parameter<double>("peer_offset_y", 0.0);
    _peer_yaw = declare_parameter<double>("peer_yaw", 0.0);
    _peer_radius = declare_parameter<double>("peer_radius", 0.5);
    const auto peer_trajectory_topic = declare_parameter<std::string>("peer_trajectory_topic", "");
    _trajectory_exchange = !peer_trajectory_topic.empty();
    _trajectory_timeout = declare_parameter<double>("peer_trajectory_timeout", 0.5);
    if (!std::isfinite(_trajectory_timeout) || _trajectory_timeout <= 0.0)
        throw std::invalid_argument("peer_trajectory_timeout must be finite and positive");
    if (_trajectory_exchange && !_peer_enabled)
        throw std::invalid_argument("peer_trajectory_topic requires peer_odom_topic");
    if (_trajectory_exchange)
    {
        _peer_trajectory_sub = create_subscription<nav_msgs::msg::Path>(
            peer_trajectory_topic, rclcpp::QoS(1).reliable(),
            std::bind(&JackalPlanner::peerTrajectoryCallback, this, std::placeholders::_1));
        LOG_INFO("Peer planned trajectory input enabled: " << peer_trajectory_topic);
    }
    if (_peer_enabled && _empty_world_test)
        throw std::runtime_error("peer_odom_topic and empty_world_test are mutually exclusive");
    if (_peer_enabled)
    {
        _peer_sub = create_subscription<nav_msgs::msg::Odometry>(
            peer_topic, rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                if (msg->header.frame_id != "odom")
                    return;
                _peer_state = msg;
                _last_peer_time = std::chrono::steady_clock::now();
            });
    }
    if (_empty_world_test || _peer_enabled)
    {
        CONFIG["weights"]["reference_velocity"] = reference_velocity;
        if (_empty_world_test)
            LOG_WARN("Empty-world test enabled: real obstacles are not received by this node");
        else
            LOG_INFO("Peer obstacle input enabled: " << peer_topic);
    }
    _data.robot_area = {Disc(0., CONFIG["robot_radius"].as<double>())};

    // Initialize the planner
    _planner = std::make_unique<Planner>();

    // Initialize the ROS interface
    initializeSubscribersAndPublishers();

    _benchmarker = std::make_unique<RosTools::Benchmarker>("loop");

    // Start the control loop
    _timer = create_timer(
        this,
        this->get_clock(),
        Duration::from_seconds(1.0 / CONFIG["control_frequency"].as<double>()),
        std::bind(&JackalPlanner::Loop, this));

    LOG_DIVIDER();
}

void JackalPlanner::initializeSubscribersAndPublishers()
{
    LOG_INFO("initializeSubscribersAndPublishers");

    _state_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/input/state", rclcpp::SensorDataQoS(),
        std::bind(&JackalPlanner::stateCallback, this, std::placeholders::_1));

    _goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/input/goal", 1,
        std::bind(&JackalPlanner::goalCallback, this, std::placeholders::_1));

    _path_sub = this->create_subscription<nav_msgs::msg::Path>(
        "~/input/reference_path", 1,
        std::bind(&JackalPlanner::pathCallback, this, std::placeholders::_1));

    _peer_pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>("~/debug/peer_pose", 1);
    _trajectory_pub = create_publisher<nav_msgs::msg::Path>("~/output/predicted_trajectory", 1);
    _peer_prediction_pub = create_publisher<nav_msgs::msg::Path>("~/debug/peer_prediction", 1);

    _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
        "~/output/command", 1);
}

void JackalPlanner::Loop()
{
    LOG_DEBUG("============= Loop =============");

    if (!_state_received || _data.reference_path.x.empty() ||
        std::chrono::steady_clock::now() - _last_state_time > std::chrono::seconds(1))
    {
        publishStop();
        return;
    }
    const auto &path = _data.reference_path;
    if (std::hypot(_state.get("x") - path.x.back(), _state.get("y") - path.y.back()) < 0.25)
    {
        publishStop();
        return;
    }
    if (_peer_enabled)
    {
        if (!_peer_state || std::chrono::steady_clock::now() - _last_peer_time > std::chrono::seconds(1))
        {
            publishStop();
            return;
        }
        // Fixed transform from peer local odom to this robot's local odom.
        const auto &pose = _peer_state->pose.pose;
        const auto &twist = _peer_state->twist.twist;
        const double c = std::cos(_peer_yaw), s = std::sin(_peer_yaw);
        Eigen::Vector2d pos(c * pose.position.x - s * pose.position.y + _peer_offset_x,
                            s * pose.position.x + c * pose.position.y + _peer_offset_y);
        const double heading = RosTools::quaternionToAngle(pose.orientation) + _peer_yaw;
        Eigen::Vector2d vel(std::cos(heading) * twist.linear.x - std::sin(heading) * twist.linear.y,
                            std::sin(heading) * twist.linear.x + std::cos(heading) * twist.linear.y);
        geometry_msgs::msg::PoseStamped peer_pose;
        peer_pose.header.stamp = now();
        peer_pose.header.frame_id = "odom"; // This robot's local odom frame.
        peer_pose.pose.position.x = pos.x();
        peer_pose.pose.position.y = pos.y();
        peer_pose.pose.orientation = RosTools::angleToQuaternion(heading);
        _peer_pose_pub->publish(peer_pose);
        _data.dynamic_obstacles.clear();
        _data.dynamic_obstacles.emplace_back(0, pos, heading, _peer_radius);
        auto &prediction = _data.dynamic_obstacles.back().prediction;
        prediction = getConstantVelocityPrediction(
            pos, vel, CONFIG["integrator_step"].as<double>(), CONFIG["N"].as<int>());
        if (_trajectory_exchange && !loadPeerPrediction(prediction))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Waiting for a fresh, time-aligned peer trajectory; commanding stop");
            publishStop();
            return;
        }
        ensureObstacleSize(_data.dynamic_obstacles, _state);

    }
    _data.planning_start_time = std::chrono::system_clock::now();
    if (_empty_world_test)
    {
        _data.dynamic_obstacles.clear();
        ensureObstacleSize(_data.dynamic_obstacles, _state);
    }
    if (_peer_enabled || _empty_world_test)
        _planner->onDataReceived(_data, "dynamic obstacles");
    const auto plan_stamp = now();
    const auto solve_start = std::chrono::steady_clock::now();
    _benchmarker->start();

    // Print the state
    if (!CONFIG["diagnostics"].as<bool>())
        _state.print();

    auto output = _planner->solveMPC(_state, _data);

    LOG_VALUE_DEBUG("Success", output.success);

    geometry_msgs::msg::Twist cmd;
    if (output.success)
    {
        // Publish the command
        cmd.linear.x = _planner->getSolution(1, "v");  // = x1
        cmd.angular.z = _planner->getSolution(0, "w"); // = u0
        LOG_VALUE_DEBUG("Commanded v", cmd.linear.x);
        LOG_VALUE_DEBUG("Commanded w", cmd.angular.z);
    }
    else
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }
    _cmd_pub->publish(cmd);
    publishTrajectory(!output.success, plan_stamp);
    _benchmarker->stop();
    if (CONFIG["diagnostics"].as<bool>())
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "MPC timing: solve_ms=%.3f",
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - solve_start).count());
    }

    _planner->visualize(_state, _data);
    visualize();

    LOG_DEBUG("============= End Loop =============");
}

void JackalPlanner::publishStop()
{
    _cmd_pub->publish(geometry_msgs::msg::Twist());
    // Do not relabel a stale position as a fresh plan. A healthy stationary
    // robot does keep publishing, including after arrival or a failed optimization.
    if (_state_received && std::chrono::steady_clock::now() - _last_state_time <= std::chrono::seconds(1))
        publishTrajectory(true, now());
}

void JackalPlanner::publishTrajectory(bool stationary, const rclcpp::Time &stamp)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = stamp;
    const int steps = CONFIG["N"].as<int>();
    const double dt = CONFIG["integrator_step"].as<double>();
    for (int k = 0; k < steps; ++k)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = path.header.frame_id;
        pose.header.stamp = stamp + rclcpp::Duration::from_seconds(k * dt);
        const bool measured = stationary || k == 0;
        pose.pose.position.x = measured ? _state.get("x") : _planner->getSolution(k, "x");
        pose.pose.position.y = measured ? _state.get("y") : _planner->getSolution(k, "y");
        pose.pose.orientation = RosTools::angleToQuaternion(
            measured ? _state.get("psi") : _planner->getSolution(k, "psi"));
        path.poses.push_back(pose);
    }
    _trajectory_pub->publish(path);
}

void JackalPlanner::peerTrajectoryCallback(nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->header.frame_id != "odom" || msg->poses.size() < 2 ||
        msg->header.stamp != msg->poses.front().header.stamp)
        return;
    std::vector<AntbotMPC::TimedPose> poses;
    poses.reserve(msg->poses.size());
    for (const auto &pose : msg->poses)
    {
        const auto &q = pose.pose.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (pose.header.frame_id != "odom" || pose.header.stamp.sec < 0 ||
            pose.header.stamp.nanosec >= 1000000000u || !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01)
            return;
        poses.push_back({rclcpp::Time(pose.header.stamp, RCL_ROS_TIME).seconds(),
                         pose.pose.position.x, pose.pose.position.y,
                         RosTools::quaternionToAngle(q)});
    }
    if (!AntbotMPC::validTrajectory(poses))
        return;
    const double age = now().seconds() - poses.front().time;
    if (age < -0.05 || age > _trajectory_timeout)
        return;
    _peer_trajectory = std::move(poses);
    _last_trajectory_time = std::chrono::steady_clock::now();
}

bool JackalPlanner::loadPeerPrediction(MPCPlanner::Prediction &prediction)
{
    if (_peer_trajectory.empty() ||
        std::chrono::duration<double>(std::chrono::steady_clock::now() - _last_trajectory_time).count() > _trajectory_timeout)
        return false;
    const double current_time = now().seconds();
    const double age = current_time - _peer_trajectory.front().time;
    if (age < -0.05 || age > _trajectory_timeout)
        return false;
    const double dt = CONFIG["integrator_step"].as<double>();
    // A small permitted clock lead is clamped; never extrapolate backwards.
    const double start = std::max(current_time, _peer_trajectory.front().time);
    nav_msgs::msg::Path debug;
    debug.header.frame_id = "odom";
    // Retain the source epoch so observers can identify which peer plan was used.
    debug.header.stamp = rclcpp::Time(static_cast<int64_t>(_peer_trajectory.front().time * 1e9), RCL_ROS_TIME);
    for (size_t k = 0; k < prediction.modes[0].size(); ++k)
    {
        AntbotMPC::TimedPose sampled;
        if (!AntbotMPC::sampleTrajectory(_peer_trajectory, start + k * dt, _trajectory_timeout + dt, sampled))
            return false;
        const auto transformed = AntbotMPC::transformPose(sampled, _peer_offset_x, _peer_offset_y, _peer_yaw);
        prediction.modes[0][k].position = Eigen::Vector2d(transformed.x, transformed.y);
        prediction.modes[0][k].angle = transformed.yaw;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "odom";
        pose.header.stamp = rclcpp::Time(static_cast<int64_t>(sampled.time * 1e9), RCL_ROS_TIME);
        pose.pose.position.x = transformed.x;
        pose.pose.position.y = transformed.y;
        pose.pose.orientation = RosTools::angleToQuaternion(transformed.yaw);
        debug.poses.push_back(pose);
    }
    _peer_prediction_pub->publish(debug);
    return true;
}

void JackalPlanner::stateCallback(nav_msgs::msg::Odometry::SharedPtr msg)
{
    _state_received = true;
    _last_state_time = std::chrono::steady_clock::now();
    _state.set("x", msg->pose.pose.position.x);
    _state.set("y", msg->pose.pose.position.y);
    _state.set("psi", RosTools::quaternionToAngle(msg->pose.pose.orientation));
    _state.set("v", std::sqrt(std::pow(msg->twist.twist.linear.x, 2.) + std::pow(msg->twist.twist.linear.y, 2.)));
}

void JackalPlanner::goalCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    LOG_DEBUG("Goal callback");
    _data.goal(0) = msg->pose.position.x;
    _data.goal(1) = msg->pose.position.y;
    _data.goal_received = true;
}

bool JackalPlanner::isPathTheSame(nav_msgs::msg::Path::SharedPtr msg)
{
    // Check if the path is the same
    if (_data.reference_path.x.size() != msg->poses.size())
        return false;

    // Check up to the first two points
    int num_points = std::min(2, (int)_data.reference_path.x.size());
    for (int i = 0; i < num_points; i++)
    {
        if (!_data.reference_path.pointInPath(i, msg->poses[i].pose.position.x, msg->poses[i].pose.position.y))
            return false;
    }
    return true;
}

void JackalPlanner::pathCallback(nav_msgs::msg::Path::SharedPtr msg)
{
    LOG_DEBUG("Path callback");

    if (msg->header.frame_id != "odom" || msg->poses.size() < 4)
    {
        LOG_WARN("Reference path must contain at least four points in odom frame");
        return;
    }
    if (isPathTheSame(msg))
        return;

    _data.reference_path.clear();

    for (auto &pose : msg->poses)
    {
        _data.reference_path.x.push_back(pose.pose.position.x);
        _data.reference_path.y.push_back(pose.pose.position.y);
    }
    _data.reference_path.psi.push_back(0.0);
    _planner->onDataReceived(_data, "reference_path");
}

void JackalPlanner::visualize()
{
    auto &publisher = VISUALS.getPublisher("angle");
    auto &line = publisher.getNewLine();

    line.addLine(Eigen::Vector2d(_state.get("x"), _state.get("y")),
                 Eigen::Vector2d(_state.get("x") + 1.0 * std::cos(_state.get("psi")), _state.get("y") + 1.0 * std::sin(_state.get("psi"))));
    publisher.publish();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto jackal_planner = std::make_shared<JackalPlanner>();
    VISUALS.init(jackal_planner.get());

    rclcpp::spin(jackal_planner);

    return 0;
}

/*
// These are tests
Solver solver;

solver.setParameter(0, "acceleration", 3.0);
solver.setParameter(0, "angular_velocity", 4.0);
assert(solver.getParameter(0, "angular_velocity") == 4.0);
assert(solver.getParameter(0, "acceleration") == 3.0);

solver.setXinit("x", 10.0);
solver.setXinit("y", 20.0);
assert(solver._params.xinit[0] == 10.0);
assert(solver._params.xinit[1] == 20.0);

// Test run for the solver
solver.setXinit("x", 0.0);
solver.setXinit("y", 0.0);
solver.setXinit("v", 1.0);

for (int k = 0; k < solver.N; k++)
{
solver.setParameter(k, "acceleration", 1.0);
solver.setParameter(k, "angular_velocity", 3.0);
assert(solver.getParameter(k, "acceleration") == 1.0);
assert(solver.getParameter(k, "angular_velocity") == 3.0);
}

int exitflag = solver.solve();
std::cout << "Exit flag: " << exitflag << std::endl;
assert(exitflag == 1);

// Read the solution
std::cout << solver.getOutput(0, "a") << std::endl;
std::cout << solver.getOutput(0, "w") << std::endl;
std::cout << solver.getOutput(2, "x") << std::endl;
std::cout << solver.getOutput(2, "y") << std::endl;
*/