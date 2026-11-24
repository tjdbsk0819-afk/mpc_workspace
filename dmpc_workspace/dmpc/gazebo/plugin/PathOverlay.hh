#pragma once
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/marker_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <chrono>
#include <string>
#include <vector>

class PathOverlay {
 public:
  PathOverlay(rclcpp::Node::SharedPtr, const std::vector<std::string> &);
  ~PathOverlay();
  void SetGoals(const std::vector<std::array<double,3>> &);
  void Tick(bool showPlans, bool showPredictions);
  static gz::msgs::Marker Ribbon(const nav_msgs::msg::Path &,const std::string &,
                                 const std::array<double,3> &,bool dashed,double z);
 private:
  struct Channel {
    std::string ns;
    bool prediction{false},shown{false};
    std::array<double,3> color;
    double height{.09};
    gz::msgs::Marker marker;
    int64_t stamp{-1};
    std::chrono::steady_clock::time_point received;
    bool valid{false},gaussian_valid{false},gaussian_shown{false};
    gz::msgs::Marker gaussian;
    int64_t gaussian_stamp{-1};
    std::chrono::steady_clock::time_point gaussian_received;
  };
  void Receive(size_t,nav_msgs::msg::Path::ConstSharedPtr);
  void Distribution(size_t,const std::string &);
  void Send(const gz::msgs::Marker_V &);
  rclcpp::Node::SharedPtr node_;
  gz::transport::Node transport_;
  std::vector<Channel> channels_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> subscriptions_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> distributions_;
  std::vector<gz::msgs::Marker> goals_;
  int64_t last_clock_{-1};
};
