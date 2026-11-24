#pragma once
#include <gz/gui/Plugin.hh>
#include <gz/gui/qt.h>
#include <QTimer>
#include <QVariantList>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>
#include <vector>

class PathOverlay;

class RobotSpeedPanel : public gz::gui::Plugin {
  Q_OBJECT
  Q_PROPERTY(QVariantList rows READ Rows NOTIFY RowsChanged)
  Q_PROPERTY(double speedLimit READ SpeedLimit NOTIFY RowsChanged)
  Q_PROPERTY(bool showPlans READ ShowPlans WRITE SetShowPlans NOTIFY RowsChanged)
  Q_PROPERTY(bool showPredictions READ ShowPredictions WRITE SetShowPredictions NOTIFY RowsChanged)
 public:
  RobotSpeedPanel();
  ~RobotSpeedPanel() override;
  void LoadConfig(const tinyxml2::XMLElement *) override;
  QVariantList Rows() const;
  double SpeedLimit() const { return limit_; }
  bool ShowPlans() const { return show_plans_; }
  bool ShowPredictions() const { return show_predictions_; }
  void SetShowPlans(bool value) {show_plans_=value;emit RowsChanged();}
  void SetShowPredictions(bool value) {show_predictions_=value;emit RowsChanged();}
 signals:
  void RowsChanged();
 private:
  struct Sample {
    std::string name;
    double speed{0.};
    int64_t stamp{-1};
    bool valid{false};
    std::chrono::steady_clock::time_point received;
  };
  std::vector<Sample> samples_;
  double limit_{1.5};
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> subscriptions_;
  std::unique_ptr<PathOverlay> overlay_;
  bool show_plans_{true},show_predictions_{true};
  QTimer timer_;
};
