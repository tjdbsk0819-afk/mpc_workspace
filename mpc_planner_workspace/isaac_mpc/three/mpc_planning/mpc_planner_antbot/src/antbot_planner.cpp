// AntBot ROS I/O and homogeneous multistart recovery around the upstream MPC OCP.
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <antbot_interfaces/msg/peer_state.hpp>
#include <antbot_interfaces/msg/mpc_plan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mpc_planner/planner.h>
#include <mpc_planner/data_preparation.h>
#include <mpc_planner_solver/state.h>
#include <mpc_planner_solver/solver_interface.h>
#include <mpc_planner_types/realtime_data.h>
#include <mpc_planner_util/parameters.h>
#include <ros_tools/ros2_wrappers.h>
#include <ros_tools/profiling.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <limits>
#include <sstream>
#include <set>
#include "replanning.h"
#include "yaw_continuity.h"

using namespace std::chrono_literals;
using MPCPlanner::State;
using MPCPlanner::RealTimeData;
using MPCPlanner::Planner;
using Path = nav_msgs::msg::Path;
using Odom = nav_msgs::msg::Odometry;
using Peer = antbot_interfaces::msg::PeerState;
using MpcPlan = antbot_interfaces::msg::MpcPlan;

class AntbotPlanner : public rclcpp::Node
{
public:
  AntbotPlanner() : Node("mpc_planner")
  {
    STATIC_NODE_POINTER.init(this);
    const auto share = ament_index_cpp::get_package_share_directory("mpc_planner_antbot");
    Configuration::getInstance().initialize(declare_parameter("settings_file", share + "/config/settings.yaml"));
    ego_ = declare_parameter("ego_robot", std::string("robot_a"));
    peer_ids_ = declare_parameter("peer_robots", std::vector<std::string>{"robot_b", "robot_c"});
    peers_.resize(peer_ids_.size()); peer_walls_.resize(peer_ids_.size());
    goal_ = declare_parameter("goal", std::vector<double>{3.0, 0.0});
    timeout_ = declare_parameter("state_timeout", 0.5);
    tolerance_ = declare_parameter("goal_tolerance", 0.2);
    if (goal_.size() != 2 || !std::isfinite(goal_[0]) || !std::isfinite(goal_[1]) ||
        peer_ids_.empty() || std::set<std::string>(peer_ids_.begin(),peer_ids_.end()).size()!=peer_ids_.size() ||
        std::find(peer_ids_.begin(),peer_ids_.end(),ego_)!=peer_ids_.end() || timeout_ <= 0. || tolerance_ <= 0.)
      throw std::runtime_error("Invalid AntBot robot/goal/timeout configuration");
    n_ = CONFIG["N"].as<int>();
    dt_ = CONFIG["integrator_step"].as<double>();
    radius_ = CONFIG["robot_radius"].as<double>();
    peer_radius_ = CONFIG["obstacle_radius"].as<double>() + CONFIG["antbot_safety_margin"].as<double>();
    tracking_margin_ = CONFIG["antbot_tracking_margin"].as<double>();
    if (!std::isfinite(tracking_margin_) || tracking_margin_ < 0.)
      throw std::runtime_error("Invalid AntBot tracking margin");
    if (n_ < 3 || n_ != SOLVER_N || dt_ <= 0. || CONFIG["max_obstacles"].as<size_t>() != peer_ids_.size())
      throw std::runtime_error("AntBot bridge requires one obstacle slot per configured peer and N>=3");
    const auto yaw = CONFIG["yaw_continuity"];
    yaw_enabled_ = yaw["enabled"].as<bool>();
    yaw_weight_ = yaw["weight"].as<double>();
    yaw_timeout_ = yaw["reference_timeout"].as<double>();
    yaw_decay_ = yaw["decay_time"].as<double>();
    yaw_acceleration_ = yaw["max_yaw_acceleration"].as<double>();
    yaw_first_horizon_ = yaw["first_command_horizon"].as<double>();
    yaw_recovery_relax_ = yaw["recovery_relax_time"].as<double>();
    if (!std::isfinite(yaw_weight_) || yaw_weight_ < 0.) throw std::runtime_error("Invalid yaw continuity weight");
    for (double v : {yaw_timeout_, yaw_decay_, yaw_acceleration_, yaw_first_horizon_, yaw_recovery_relax_})
      if (!std::isfinite(v) || v <= 0.) throw std::runtime_error("Invalid yaw continuity setting");
    if (yaw_first_horizon_ > dt_) throw std::runtime_error("Yaw command horizon exceeds MPC stage");
    data_.robot_area = {MPCPlanner::Disc(0., radius_)};
    recovery_enabled_ = declare_parameter("recovery_enabled", true);
    progress_.window = declare_parameter("recovery_stall_time", 3.0);
    progress_.minimum_progress = declare_parameter("recovery_progress_distance", 0.08);
    progress_.cooldown = declare_parameter("recovery_cooldown", 6.0);
    switch_cost_ = declare_parameter("recovery_switch_cost", 0.05);
    if (!std::isfinite(progress_.window) || progress_.window <= 0. ||
        !std::isfinite(progress_.minimum_progress) || progress_.minimum_progress <= 0. ||
        !std::isfinite(progress_.cooldown) || progress_.cooldown < progress_.window ||
        !std::isfinite(switch_cost_) || switch_cost_ < 0.)
      throw std::runtime_error("Invalid MPC recovery parameters");
    // Same bank, costs and selection for every robot. No ID-dependent roles.
    for (size_t i = 0; i < peaks_.size(); ++i) {
      planners_.push_back(std::make_unique<Planner>());
      candidate_pubs_.push_back(create_publisher<Path>("mpc_candidate_"+std::to_string(i), 1));
    }
    recovery_pub_ = create_publisher<std_msgs::msg::String>("recovery_status", 1);
    path_pub_ = create_publisher<Path>("mpc_path", 1);
    plan_pub_ = create_publisher<MpcPlan>("mpc_plan", 1);
    for (const auto &peer : peer_ids_)
      prediction_pubs_.push_back(create_publisher<Path>("mpc_peer_prediction/"+peer, 1));
    reference_pub_ = create_publisher<Path>("mpc_reference_path", 1);
    status_pub_ = create_publisher<std_msgs::msg::String>("mpc_status", 1);
    odom_sub_ = create_subscription<Odom>("odom", rclcpp::SensorDataQoS(),
      [this](Odom::ConstSharedPtr msg) {
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;
        const auto &v = msg->twist.twist.linear;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (msg->header.frame_id != "world" || msg->child_frame_id != ego_ + "/base_link" ||
            !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(norm) ||
            std::abs(norm - 1.) > 0.1 || !std::isfinite(v.x) || !std::isfinite(v.y) ||
            !std::isfinite(msg->twist.twist.angular.z) || std::abs(v.y) > .05) {
          odom_.reset(); return;
        }
        if ((rclcpp::Time(msg->header.stamp)-now()).seconds() > timeout_) { odom_.reset(); return; }
        // A duplicate sample must not extend the receive timeout.
        if (odom_ && rclcpp::Time(msg->header.stamp) <= rclcpp::Time(odom_->header.stamp)) return;
        odom_ = msg; odom_wall_ = std::chrono::steady_clock::now();
      });
    for (size_t i=0; i<peer_ids_.size(); ++i) {
      peer_subs_.push_back(create_subscription<Peer>("peers/"+peer_ids_[i]+"/state", 1,
        [this,i](Peer::ConstSharedPtr msg) {
          if (!msg->valid || msg->header.frame_id != "world" || msg->robot_id != peer_ids_[i] ||
              !std::isfinite(msg->x) || !std::isfinite(msg->y) ||
              !std::isfinite(msg->vx) || !std::isfinite(msg->vy)) { peers_[i].reset(); return; }
          if ((rclcpp::Time(msg->header.stamp)-now()).seconds() > timeout_) { peers_[i].reset(); return; }
          if (peers_[i] && rclcpp::Time(msg->header.stamp)<=rclcpp::Time(peers_[i]->header.stamp)) return;
          peers_[i]=msg; peer_walls_[i]=std::chrono::steady_clock::now();
        }));
    }
    reference_sub_ = create_subscription<Path>("reference_path", 1,
      [this](Path::ConstSharedPtr msg) { set_reference(*msg, true); });
    replan_service_ = create_service<std_srvs::srv::Trigger>("~/replan_from_current",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!odom_ || !fresh(odom_->header.stamp, now(), odom_wall_)) {
          response->success = false; response->message = "Fresh ego odometry required"; return;
        }
        // Reset only the optimization/reference; never issue a motion command.
        reset_planners();
        progress_.reset();
        stop_output("replanning", now());
        response->success = true; response->message = "Reference reset; original goal preserved";
      });
    const double frequency = CONFIG["control_frequency"].as<double>();
    timer_ = create_wall_timer(std::chrono::duration<double>(1. / frequency), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "Original tud-amr Planner + acados: %s observes %zu peers; path output only",
                ego_.c_str(), peer_ids_.size());
  }

private:
  struct Neighbor { Eigen::Vector2d position, velocity; };
  void status(const std::string &value)
  {
    std_msgs::msg::String msg; msg.data = value; status_pub_->publish(msg);
    if (value != last_status_) RCLCPP_INFO(get_logger(), "%s", value.c_str());
    last_status_ = value;
  }

  Path empty_path(const rclcpp::Time &stamp) const
  {
    Path msg; msg.header.frame_id = "world"; msg.header.stamp = stamp; return msg;
  }

  void stop_output(const std::string &reason, const rclcpp::Time &stamp)
  {
    yaw_history_.clear(); // Never anchor a new plan to a failed/stale execution.
    path_pub_->publish(empty_path(stamp));
    for (auto &pub : candidate_pubs_) pub->publish(empty_path(stamp));
    for (auto &pub : prediction_pubs_) pub->publish(empty_path(stamp));
    MpcPlan plan;
    plan.header.frame_id = "world"; plan.header.stamp = stamp;
    plan.path = empty_path(stamp); plan.valid = false; plan.status = reason;
    plan.minimum_separation = radius_ + peer_radius_;
    plan_pub_->publish(plan);
    status(reason);
  }

  void set_reference(const Path &path, bool update_goal = false)
  {
    if (path.header.frame_id != "world" || path.poses.size() < 6) return;
    MPCPlanner::ReferencePath reference;
    for (const auto &pose : path.poses) {
      const auto &p = pose.pose.position;
      if ((!pose.header.frame_id.empty() && pose.header.frame_id != "world") ||
          !std::isfinite(p.x) || !std::isfinite(p.y)) return;
      if (!reference.x.empty() && std::hypot(reference.x.back()-p.x, reference.y.back()-p.y) < 1e-4) return;
      reference.x.push_back(p.x); reference.y.push_back(p.y);
    }
    reference.psi.push_back(0.);
    data_.reference_path = reference;
    for (auto &planner : planners_) planner->onDataReceived(data_, "reference_path");
    reference_ = path;
    if (update_goal) {
      yaw_history_.clear();
      goal_ = {reference.x.back(), reference.y.back()};
      progress_.reset(); searching_ = false; selected_ = 0;
      std::fill(seeds_.begin(), seeds_.end(), true);
    }
  }

  void make_reference(const rclcpp::Time &stamp)
  {
    const Eigen::Vector2d start(state_.get("x"), state_.get("y"));
    const Eigen::Vector2d delta = Eigen::Vector2d(goal_[0], goal_[1]) - start;
    if (delta.norm() < tolerance_) return;
    // Extend beyond the goal so the original spline has enough nonzero segments.
    // Actual completion is checked against goal_, never this extension's endpoint.
    const auto direction = delta.normalized();
    const double length = delta.norm() + 4.;
    auto path = empty_path(stamp);
    const int count = static_cast<int>(std::ceil(length / 0.5));
    for (int k = 0; k <= count; ++k) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      const auto p = start + direction * (length * k / count);
      pose.pose.position.x = p.x(); pose.pose.position.y = p.y(); pose.pose.orientation.w = 1.;
      path.poses.push_back(pose);
    }
    set_reference(path);
  }

  bool fresh(const builtin_interfaces::msg::Time &stamp, const rclcpp::Time &now,
             std::chrono::steady_clock::time_point received) const
  {
    const double age = (now - rclcpp::Time(stamp)).seconds();
    const double wall_age = std::chrono::duration<double>(std::chrono::steady_clock::now()-received).count();
    return age >= -1e-6 && age <= timeout_ && wall_age <= timeout_;
  }

  void reset_planners()
  {
    yaw_history_.clear();
    yaw_relax_until_ = now().seconds()+yaw_recovery_relax_;
    // Upstream reset clears its State/Data arguments. Preserve current GT inputs
    // and robot geometry, and rebuild every candidate's reference explicitly.
    for (auto &planner : planners_) {
      auto scratch_state = state_;
      auto scratch_data = data_;
      planner->reset(scratch_state, scratch_data, false);
    }
    state_.set("spline", 0.);
    reference_ = Path(); selected_ = 0; searching_ = false;
    std::fill(seeds_.begin(), seeds_.end(), true);
  }

  void recovery_status(const std::string &phase, double distance,
                       const std::vector<double> &costs = {})
  {
    std::ostringstream out;
    out << "{\"phase\":\"" << phase << "\",\"enabled\":" << (recovery_enabled_ ? "true" : "false")
        << ",\"episode\":" << recovery_episode_ << ",\"selected\":" << selected_
        << ",\"goal_distance\":" << distance << ",\"planning_buffer\":" << active_tracking_margin_ << ",\"costs\":[";
    for (size_t i = 0; i < costs.size(); ++i) {
      if (i) out << ',';
      if (std::isfinite(costs[i])) out << costs[i]; else out << "null";
    }
    const auto &yaw = data_.yaw_continuity;
    out << "],\"yaw_continuity_active\":" <<
        (!yaw.weights.empty() && yaw.weights[0] > 0. ? "true" : "false")
        << ",\"yaw_reference_first\":" << (yaw.reference.empty() ? 0. : yaw.reference[0])
        << ",\"yaw_weight_first\":" << (yaw.weights.empty() ? 0. : yaw.weights[0])
        << ",\"yaw_first_lower\":" << yaw.first_lower
        << ",\"yaw_first_upper\":" << yaw.first_upper
        << ",\"yaw_first_solution\":";
    if (std::isfinite(yaw_solution_)) out << yaw_solution_; else out << "null";
    out << "}";
    std_msgs::msg::String msg; msg.data = out.str(); recovery_pub_->publish(msg);
  }

  void tick()
  {
    yaw_solution_ = std::numeric_limits<double>::quiet_NaN();
    const auto stamp = now();
    if (last_time_ >= 0 && stamp.nanoseconds() < last_time_) {
      reset_planners(); progress_.reset(); odom_.reset();
      for (auto &peer : peers_) peer.reset();
    }
    last_time_ = stamp.nanoseconds();
    if (!odom_ || !fresh(odom_->header.stamp, stamp, odom_wall_) || !peers_fresh(stamp)) {
      progress_.invalidate();
      recovery_status("waiting_for_inputs", -1.);
      stop_output("waiting_for_fresh_ego_and_peer", stamp); return;
    }
    const auto &p = odom_->pose.pose.position;
    const auto &q = odom_->pose.pose.orientation;
    const double measured_yaw = std::atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z));
    const double ego_age = std::max(0.,(stamp-rclcpp::Time(odom_->header.stamp)).seconds());
    const double measured_w = odom_->twist.twist.angular.z;
    const double measured_v = odom_->twist.twist.linear.x;
    // Same measured-motion midpoint propagation as the execution adapter:
    // both ego and peer states now refer to the plan's header epoch.
    state_.set("x",p.x+measured_v*ego_age*std::cos(measured_yaw+.5*measured_w*ego_age));
    state_.set("y",p.y+measured_v*ego_age*std::sin(measured_yaw+.5*measured_w*ego_age));
    state_.set("psi",measured_yaw+measured_w*ego_age);
    // Odometry twist is body-frame: the unicycle's signed forward speed.
    state_.set("v", odom_->twist.twist.linear.x);
    if (std::hypot(state_.get("x")-goal_[0], state_.get("y")-goal_[1]) <= tolerance_) {
      progress_.invalidate(); searching_ = false;
      recovery_status("goal_reached", 0.);
      stop_output("goal_reached", stamp); return;
    }
    const double distance = std::hypot(state_.get("x")-goal_[0], state_.get("y")-goal_[1]);
    if (progress_.update(stamp.seconds(), distance, recovery_enabled_)) {
      reset_planners();
      searching_ = true; ++recovery_episode_;
      RCLCPP_INFO(get_logger(), "MPC recovery %d: reset reference and optimize %zu initial guesses; goal unchanged",
                  recovery_episode_, peaks_.size());
    }
    if (reference_.poses.empty()) make_reference(stamp);
    reference_pub_->publish(reference_);

    auto &yaw = data_.yaw_continuity;
    yaw = RealTimeData::YawContinuityData();
    yaw.reference.assign(n_, 0.); yaw.weights.assign(n_, 0.);
    const double yaw_limit = CONFIG["antbot_limits"]["yaw_rate"].as<double>();
    yaw.first_enabled = yaw_enabled_;
    const double change = yaw_acceleration_*yaw_first_horizon_;
    yaw.first_lower = yaw_enabled_ ? std::max(-yaw_limit,measured_w-change) : -yaw_limit;
    yaw.first_upper = yaw_enabled_ ? std::min(yaw_limit,measured_w+change) : yaw_limit;
    if (yaw.first_lower > yaw.first_upper) {
      stop_output("yaw_rate_outside_execution_envelope", stamp); return;
    }
    if (yaw_enabled_ && stamp.seconds() >= yaw_relax_until_) {
      for (int k=0; k<n_; ++k) {
        if (yaw_history_.average(stamp.seconds(),k*dt_,dt_,yaw_timeout_,yaw.reference[k]))
          yaw.weights[k] = yaw_weight_*std::exp(-k*dt_/yaw_decay_);
      }
    }

    std::vector<Neighbor> neighbors;
    double minimum_distance=std::numeric_limits<double>::infinity();
    for (const auto &peer : peers_) {
      const Eigen::Vector2d velocity(peer->vx,peer->vy);
      const double age=(stamp-rclcpp::Time(peer->header.stamp)).seconds();
      const Eigen::Vector2d position=Eigen::Vector2d(peer->x,peer->y)+velocity*age;
      neighbors.push_back({position,velocity});
      minimum_distance=std::min(minimum_distance,(state_.getPos()-position).norm());
    }
    if (minimum_distance < radius_+peer_radius_) {
      recovery_status("insufficient_current_clearance",distance);
      stop_output("insufficient_current_clearance",stamp);return;
    }
    active_tracking_margin_=std::min(tracking_margin_,.5*std::max(0.,minimum_distance-radius_-peer_radius_));
    data_.dynamic_obstacles.clear();
    for (size_t i=0;i<neighbors.size();++i) {
      const auto &peer=neighbors[i];
      data_.dynamic_obstacles.emplace_back(i,peer.position,0.,peer_radius_+active_tracking_margin_);
      // Solver stage k uses prediction[k-1], so the first point is now+dt.
      data_.dynamic_obstacles.back().prediction=MPCPlanner::getConstantVelocityPrediction(
          peer.position+peer.velocity*dt_,peer.velocity,dt_,n_);
      auto predicted=empty_path(stamp);
      for (int k=0;k<n_;++k) {
        geometry_msgs::msg::PoseStamped pose;pose.header.frame_id="world";
        pose.header.stamp=stamp+rclcpp::Duration::from_seconds((k+1)*dt_);
        const auto point=data_.dynamic_obstacles.back().prediction.modes[0][k].position;
        pose.pose.position.x=point.x();pose.pose.position.y=point.y();pose.pose.orientation.w=1.;
        predicted.poses.push_back(pose);
      }
      prediction_pubs_[i]->publish(predicted);
    }
    for (auto &planner : planners_) planner->onDataReceived(data_,"dynamic obstacles");
    const auto batch_start = std::chrono::steady_clock::now();
    std::vector<double> costs(peaks_.size(), std::numeric_limits<double>::infinity());
    std::vector<Path> paths(peaks_.size(), empty_path(stamp));
    std::vector<double> speeds(peaks_.size()), rates(peaks_.size());
    // Run the selected warm start first, then all alternatives during recovery.
    std::vector<int> order{selected_};
    if (searching_) for (size_t i = 0; i < peaks_.size(); ++i)
      if (static_cast<int>(i) != selected_) order.push_back(static_cast<int>(i));
    for (int index : order) {
      // One shared wall-time budget. Never publish a partially aged batch.
      if (std::chrono::duration<double>(std::chrono::steady_clock::now()-batch_start).count() >= 1./CONFIG["control_frequency"].as<double>()) break;
      auto &planner = *planners_[index];
      Planner::InitialGuess guess;
      if (searching_ && seeds_[index]) {
        const auto limits = CONFIG["antbot_limits"];
        guess = AntbotRecovery::make_guess(state_.get("x"), state_.get("y"), state_.get("psi"), state_.get("v"),
            goal_[0], goal_[1], peaks_[index], n_, dt_,
            CONFIG["weights"]["reference_velocity"].as<double>(),
            limits["acceleration"].as<double>(), limits["yaw_rate"].as<double>());
      }
      data_.planning_start_time = std::chrono::system_clock::now();
      const auto result = planner.solveMPC(state_, data_, guess);
      if (!result.success) { seeds_[index] = true; continue; }
      seeds_[index] = false;
      auto &path = paths[index];
      if (!read_and_validate(planner, stamp, neighbors, path)) {
        seeds_[index] = true; path = empty_path(stamp); continue;
      }
      speeds[index] = planner.getSolution(1, "v");
      rates[index] = planner.getSolution(0, "w");
      const double objective = planner.getObjective();
      if (!std::isfinite(objective) || !std::isfinite(speeds[index]) || !std::isfinite(rates[index])) {
        path = empty_path(stamp); seeds_[index] = true; continue;
      }
      costs[index] = objective;
    }
    const int best = AntbotRecovery::select(costs, selected_, switch_cost_);
    if ((now()-stamp).seconds() > dt_ ||
        std::chrono::duration<double>(std::chrono::steady_clock::now()-batch_start).count() > dt_ ||
        !peers_fresh(now()) || !fresh(odom_->header.stamp, now(), odom_wall_)) {
      recovery_status("solution_expired", distance, costs);
      stop_output("solution_expired", stamp); return;
    }
    for (size_t i = 0; i < paths.size(); ++i) candidate_pubs_[i]->publish(paths[i]);
    if (best < 0) {
      recovery_status(searching_ ? "no_feasible_candidate" : "solver_failed", distance, costs);
      stop_output("solver_failed", stamp); return;
    }
    selected_ = best;
    yaw_solution_ = rates[best];
    recovery_status(searching_ ? "tracking_candidate" : "tracking", distance, costs);
    MpcPlan plan;
    plan.header = paths[best].header; plan.path = paths[best]; plan.valid = true; plan.status = "tracking";
    plan.target_speed = std::max(0., speeds[best]); plan.target_yaw_rate = rates[best];
    plan.max_acceleration = CONFIG["antbot_limits"]["acceleration"].as<double>();
    plan.max_yaw_rate = CONFIG["antbot_limits"]["yaw_rate"].as<double>();
    for (int k = 0; k < n_; ++k) {
      plan.linear_velocities.push_back(planners_[best]->getSolution(k, "v"));
      plan.linear_accelerations.push_back(planners_[best]->getSolution(k, "a"));
      plan.angular_velocities.push_back(planners_[best]->getSolution(k, "w"));
    }
    plan.minimum_separation = radius_ + peer_radius_;
    yaw_history_.store(stamp.seconds(),dt_,plan.angular_velocities);
    plan_pub_->publish(plan);
    path_pub_->publish(paths[best]);
    status("tracking");
  }

  bool read_and_validate(const Planner &planner, const rclcpp::Time &stamp,
                         const std::vector<Neighbor> &neighbors, Path &path)
  {
    const auto limits = CONFIG["antbot_limits"];
    for (int k = 0; k < n_; ++k) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.header.stamp = stamp + rclcpp::Duration::from_seconds(k*dt_);
      pose.pose.position.x = planner.getSolution(k, "x");
      pose.pose.position.y = planner.getSolution(k, "y");
      const double psi = planner.getSolution(k, "psi"), v = planner.getSolution(k, "v");
      const double a = planner.getSolution(k, "a"), w = planner.getSolution(k, "w");
      if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
          !std::isfinite(psi) || !std::isfinite(v) || !std::isfinite(a) || !std::isfinite(w) ||
          v < -.011 || v > limits["speed"].as<double>()+.001 ||
          std::abs(a) > limits["acceleration"].as<double>()+.001 ||
          std::abs(w) > limits["yaw_rate"].as<double>()+.001) return false;
      // SQP_RTI can return a successful QP with nonlinear residuals. Check the
      // actual first OCP input too; never post-filter an optimized trajectory.
      if (k == 0 && data_.yaw_continuity.first_enabled &&
          (w < data_.yaw_continuity.first_lower-1e-6 || w > data_.yaw_continuity.first_upper+1e-6)) return false;
      pose.pose.orientation.z = std::sin(psi/2.); pose.pose.orientation.w = std::cos(psi/2.);
      path.poses.push_back(pose);
    }
    if (std::hypot(path.poses[0].pose.position.x-state_.get("x"),
                   path.poses[0].pose.position.y-state_.get("y")) > .01) return false;
    // Validate the optimized output, never the initial guess. Check CV clearance
    // at and between knots; RPP tracking still needs its independent command gate.
    for (const auto &peer : neighbors) {
    for (int k = 0; k < n_-1; ++k) {
      const auto &a = path.poses[k].pose.position;
      const auto &b = path.poses[k+1].pose.position;
      const Eigen::Vector2d relative = Eigen::Vector2d(a.x, a.y)-peer.position-peer.velocity*(k*dt_);
      const Eigen::Vector2d change = Eigen::Vector2d(b.x-a.x,b.y-a.y)-peer.velocity*dt_;
      const double fraction = change.squaredNorm() < 1e-12 ? 0. :
          std::clamp(-relative.dot(change)/change.squaredNorm(), 0., 1.);
      if ((relative+fraction*change).norm() < radius_+peer_radius_+active_tracking_margin_-.001) return false;
    }
    }
    return true;
  }

  bool peers_fresh(const rclcpp::Time &stamp) const {
    for (size_t i=0;i<peers_.size();++i)
      if (!peers_[i] || !fresh(peers_[i]->header.stamp,stamp,peer_walls_[i])) return false;
    return true;
  }

  State state_;
  RealTimeData data_;
  std::vector<std::unique_ptr<Planner>> planners_;
  const std::vector<double> peaks_{0., .8, -.8, 1.4, -1.4};
  std::vector<bool> seeds_{true, true, true, true, true};
  std::vector<rclcpp::Publisher<Path>::SharedPtr> candidate_pubs_;
  AntbotRecovery::ProgressMonitor progress_;
  bool recovery_enabled_{true}, searching_{false};
  int selected_{0}, recovery_episode_{0};
  double switch_cost_;
  double tracking_margin_{0.}, active_tracking_margin_{0.};
  AntbotYaw::History yaw_history_;
  bool yaw_enabled_{true};
  double yaw_weight_, yaw_timeout_, yaw_decay_, yaw_acceleration_, yaw_first_horizon_, yaw_recovery_relax_;
  double yaw_relax_until_{-1.}, yaw_solution_{std::numeric_limits<double>::quiet_NaN()};
  std::string ego_, last_status_;
  std::vector<std::string> peer_ids_;
  std::vector<double> goal_;
  double timeout_, tolerance_, dt_, radius_, peer_radius_;
  int n_;
  int64_t last_time_{-1};
  Odom::ConstSharedPtr odom_;
  std::vector<Peer::ConstSharedPtr> peers_;
  std::chrono::steady_clock::time_point odom_wall_;
  std::vector<std::chrono::steady_clock::time_point> peer_walls_;
  Path reference_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<Peer>::SharedPtr> peer_subs_;
  rclcpp::Subscription<Path>::SharedPtr reference_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_service_;
  rclcpp::Publisher<Path>::SharedPtr path_pub_, reference_pub_;
  std::vector<rclcpp::Publisher<Path>::SharedPtr> prediction_pubs_;
  rclcpp::Publisher<MpcPlan>::SharedPtr plan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, recovery_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<AntbotPlanner>()); }
  catch (const std::exception &e) { std::cerr << "AntBot MPC: " << e.what() << std::endl; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown();
  return 0;
}
