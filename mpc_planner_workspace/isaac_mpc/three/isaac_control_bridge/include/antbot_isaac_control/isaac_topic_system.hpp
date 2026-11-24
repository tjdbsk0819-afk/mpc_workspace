// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#ifndef ANTBOT_ISAAC_CONTROL__ISAAC_TOPIC_SYSTEM_HPP_
#define ANTBOT_ISAAC_CONTROL__ISAAC_TOPIC_SYSTEM_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace antbot_isaac_control
{

class IsaacTopicSystem : public hardware_interface::SystemInterface
{
public:
  IsaacTopicSystem() = default;
  ~IsaacTopicSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kModuleCount = 4;

  bool validate_interfaces();
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr message);
  void start_executor();
  void stop_executor();
  void publish_command(bool stop_wheels);
  std::string hardware_parameter_or(
    const std::string & name, const std::string & fallback) const;
  double hardware_parameter_as_double(const std::string & name, double fallback) const;

  rclcpp::Logger logger_{rclcpp::get_logger("antbot_isaac_topic_system")};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  std::string state_topic_{"/joint_states"};
  std::string command_topic_{"/joint_command"};
  double state_timeout_sec_{0.5};
  double activation_timeout_sec_{5.0};

  std::vector<std::string> steering_joint_names_;
  std::vector<std::string> wheel_joint_names_;

  std::map<std::string, double> state_values_;
  std::map<std::string, double> latest_state_values_;
  std::map<std::string, double> command_values_;

  std::mutex state_mutex_;
  bool received_complete_state_{false};
  std::chrono::steady_clock::time_point last_state_receive_time_{};
};

}  // namespace antbot_isaac_control

#endif  // ANTBOT_ISAAC_CONTROL__ISAAC_TOPIC_SYSTEM_HPP_
