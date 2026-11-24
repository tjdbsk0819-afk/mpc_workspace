// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include "antbot_isaac_control/isaac_topic_system.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace antbot_isaac_control
{

namespace
{

bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces,
  const std::string & interface_name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&interface_name](const auto & interface) {return interface.name == interface_name;});
}

std::string interface_key(const std::string & joint_name, const std::string & interface_name)
{
  return joint_name + "/" + interface_name;
}

}  // namespace

IsaacTopicSystem::~IsaacTopicSystem()
{
  stop_executor();
}

hardware_interface::CallbackReturn IsaacTopicSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  state_topic_ = hardware_parameter_or("state_topic", "/joint_states");
  command_topic_ = hardware_parameter_or("command_topic", "/joint_command");
  state_timeout_sec_ = hardware_parameter_as_double("state_timeout_sec", 0.5);
  activation_timeout_sec_ = hardware_parameter_as_double("activation_timeout_sec", 5.0);

  if (state_timeout_sec_ <= 0.0 || activation_timeout_sec_ <= 0.0) {
    RCLCPP_ERROR(logger_, "State and activation timeouts must be positive.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!validate_interfaces()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  state_values_.clear();
  latest_state_values_.clear();
  command_values_.clear();
  for (const auto & joint : info_.joints) {
    for (const auto & interface : joint.state_interfaces) {
      const auto key = interface_key(joint.name, interface.name);
      state_values_[key] = 0.0;
      latest_state_values_[key] = 0.0;
    }
    for (const auto & interface : joint.command_interfaces) {
      command_values_[interface_key(joint.name, interface.name)] = 0.0;
    }
  }

  node_ = rclcpp::Node::make_shared("antbot_isaac_topic_system");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  auto state_qos = rclcpp::SensorDataQoS().keep_last(1);
  state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    state_topic_, state_qos,
    std::bind(&IsaacTopicSystem::joint_state_callback, this, std::placeholders::_1));
  command_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    command_topic_, rclcpp::QoS(1).reliable().durability_volatile());

  RCLCPP_INFO(
    logger_, "Initialized Isaac topic system: state=%s, command=%s",
    state_topic_.c_str(), command_topic_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacTopicSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (auto & item : state_values_) {
    item.second = 0.0;
  }
  for (auto & item : latest_state_values_) {
    item.second = 0.0;
  }
  for (auto & item : command_values_) {
    item.second = 0.0;
  }
  received_complete_state_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacTopicSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  start_executor();

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(activation_timeout_sec_);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (received_complete_state_) {
        for (const auto & joint_name : steering_joint_names_) {
          command_values_[interface_key(
            joint_name, hardware_interface::HW_IF_POSITION)] =
            latest_state_values_.at(interface_key(
              joint_name, hardware_interface::HW_IF_POSITION));
        }
        for (const auto & joint_name : wheel_joint_names_) {
          command_values_[interface_key(
            joint_name, hardware_interface::HW_IF_VELOCITY)] = 0.0;
        }
        RCLCPP_INFO(logger_, "Activated after receiving a complete Isaac joint state.");
        return hardware_interface::CallbackReturn::SUCCESS;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  RCLCPP_ERROR(
    logger_, "No complete %s message received within %.1f seconds.",
    state_topic_.c_str(), activation_timeout_sec_);
  stop_executor();
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn IsaacTopicSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  publish_command(true);
  stop_executor();
  RCLCPP_INFO(logger_, "Deactivated Isaac topic system with zero wheel command.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
IsaacTopicSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(state_values_.size());
  for (const auto & joint : info_.joints) {
    for (const auto & interface : joint.state_interfaces) {
      const auto key = interface_key(joint.name, interface.name);
      interfaces.emplace_back(joint.name, interface.name, &state_values_.at(key));
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
IsaacTopicSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(command_values_.size());
  for (const auto & joint : info_.joints) {
    for (const auto & interface : joint.command_interfaces) {
      const auto key = interface_key(joint.name, interface.name);
      interfaces.emplace_back(joint.name, interface.name, &command_values_.at(key));
    }
  }
  return interfaces;
}

hardware_interface::return_type IsaacTopicSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!received_complete_state_) {
    return hardware_interface::return_type::ERROR;
  }

  const double age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_state_receive_time_).count();
  if (age > state_timeout_sec_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *node_->get_clock(), 2000,
      "Isaac joint state is stale (%.3f s > %.3f s).", age, state_timeout_sec_);
    return hardware_interface::return_type::ERROR;
  }

  for (const auto & item : latest_state_values_) {
    state_values_.at(item.first) = item.second;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IsaacTopicSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  publish_command(false);
  return hardware_interface::return_type::OK;
}

bool IsaacTopicSystem::validate_interfaces()
{
  steering_joint_names_.clear();
  wheel_joint_names_.clear();

  for (const auto & joint : info_.joints) {
    const bool steering =
      has_interface(joint.command_interfaces, hardware_interface::HW_IF_POSITION) &&
      has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION);
    const bool wheel =
      has_interface(joint.command_interfaces, hardware_interface::HW_IF_VELOCITY) &&
      has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY);

    if (steering == wheel) {
      RCLCPP_ERROR(
        logger_,
        "Joint '%s' must expose exactly one supported pair: position/position "
        "or velocity/velocity.", joint.name.c_str());
      return false;
    }
    if (joint.command_interfaces.size() != 1 || joint.state_interfaces.size() != 1) {
      RCLCPP_ERROR(
        logger_, "Joint '%s' must have one command and one state interface.",
        joint.name.c_str());
      return false;
    }
    if (steering) {
      steering_joint_names_.push_back(joint.name);
    } else {
      wheel_joint_names_.push_back(joint.name);
    }
  }

  if (steering_joint_names_.size() != kModuleCount ||
    wheel_joint_names_.size() != kModuleCount)
  {
    RCLCPP_ERROR(
      logger_, "Expected four steering and four wheel joints, got %zu and %zu.",
      steering_joint_names_.size(), wheel_joint_names_.size());
    return false;
  }
  return true;
}

void IsaacTopicSystem::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  std::unordered_map<std::string, std::size_t> indices;
  indices.reserve(message->name.size());
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    indices[message->name[index]] = index;
  }

  std::map<std::string, double> updates;
  for (const auto & joint_name : steering_joint_names_) {
    const auto index_it = indices.find(joint_name);
    if (index_it == indices.end() || index_it->second >= message->position.size()) {
      return;
    }
    const double value = message->position[index_it->second];
    if (!std::isfinite(value)) {
      return;
    }
    updates[interface_key(joint_name, hardware_interface::HW_IF_POSITION)] = value;
  }
  for (const auto & joint_name : wheel_joint_names_) {
    const auto index_it = indices.find(joint_name);
    if (index_it == indices.end() || index_it->second >= message->velocity.size()) {
      return;
    }
    const double value = message->velocity[index_it->second];
    if (!std::isfinite(value)) {
      return;
    }
    updates[interface_key(joint_name, hardware_interface::HW_IF_VELOCITY)] = value;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);
  for (const auto & item : updates) {
    latest_state_values_[item.first] = item.second;
  }
  received_complete_state_ = true;
  last_state_receive_time_ = std::chrono::steady_clock::now();
}

void IsaacTopicSystem::start_executor()
{
  if (executor_thread_.joinable()) {
    return;
  }
  executor_thread_ = std::thread([this]() {executor_->spin();});
}

void IsaacTopicSystem::stop_executor()
{
  if (executor_ != nullptr) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
}

void IsaacTopicSystem::publish_command(bool stop_wheels)
{
  if (command_publisher_ == nullptr || node_ == nullptr) {
    return;
  }

  sensor_msgs::msg::JointState message;
  message.header.stamp = node_->now();
  message.name.reserve(steering_joint_names_.size() + wheel_joint_names_.size());
  message.name.insert(
    message.name.end(), steering_joint_names_.begin(), steering_joint_names_.end());
  message.name.insert(
    message.name.end(), wheel_joint_names_.begin(), wheel_joint_names_.end());

  const auto nan = std::numeric_limits<double>::quiet_NaN();
  message.position.assign(message.name.size(), nan);
  message.velocity.assign(message.name.size(), nan);

  for (std::size_t index = 0; index < steering_joint_names_.size(); ++index) {
    const auto & name = steering_joint_names_[index];
    double value = command_values_.at(interface_key(name, hardware_interface::HW_IF_POSITION));
    if (!std::isfinite(value)) {
      std::lock_guard<std::mutex> lock(state_mutex_);
      value = latest_state_values_.at(interface_key(name, hardware_interface::HW_IF_POSITION));
    }
    message.position[index] = value;
  }

  const auto wheel_offset = steering_joint_names_.size();
  for (std::size_t index = 0; index < wheel_joint_names_.size(); ++index) {
    const auto & name = wheel_joint_names_[index];
    double value = stop_wheels ? 0.0 :
      command_values_.at(interface_key(name, hardware_interface::HW_IF_VELOCITY));
    if (!std::isfinite(value)) {
      value = 0.0;
    }
    message.velocity[wheel_offset + index] = value;
  }

  command_publisher_->publish(message);
}

std::string IsaacTopicSystem::hardware_parameter_or(
  const std::string & name, const std::string & fallback) const
{
  const auto parameter = info_.hardware_parameters.find(name);
  return parameter == info_.hardware_parameters.end() ? fallback : parameter->second;
}

double IsaacTopicSystem::hardware_parameter_as_double(
  const std::string & name, double fallback) const
{
  const auto text = hardware_parameter_or(name, "");
  if (text.empty()) {
    return fallback;
  }
  try {
    return std::stod(text);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      logger_, "Invalid hardware parameter %s='%s'; using %.3f.",
      name.c_str(), text.c_str(), fallback);
    return fallback;
  }
}

}  // namespace antbot_isaac_control

PLUGINLIB_EXPORT_CLASS(
  antbot_isaac_control::IsaacTopicSystem,
  hardware_interface::SystemInterface)
