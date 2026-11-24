#include "RobotSpeedPanel.hh"
#include "PathOverlay.hh"
#include <gz/plugin/Register.hh>
#include <cmath>
#include <sstream>
#include <stdexcept>

RobotSpeedPanel::RobotSpeedPanel() = default;
RobotSpeedPanel::~RobotSpeedPanel() {
  timer_.stop();overlay_.reset();
  if (executor_ && node_) executor_->remove_node(node_);
  subscriptions_.clear();node_.reset();executor_.reset();
  if (context_ && context_->is_valid()) context_->shutdown("GUI panel closed");
}
void RobotSpeedPanel::LoadConfig(const tinyxml2::XMLElement *config) {
  std::string names="robot_a robot_b robot_c";
  if (config) {
    if (auto e=config->FirstChildElement("robots");e && e->GetText()) names=e->GetText();
    if (auto e=config->FirstChildElement("max_speed")) e->QueryDoubleText(&limit_);
  }
  if (!std::isfinite(limit_) || limit_<=0.) throw std::invalid_argument("Invalid speed limit");
  std::istringstream stream(names);std::string name;
  while (stream>>name) {
    if (name!="robot_a" && name!="robot_b" && name!="robot_c" && name!="robot_d")
      throw std::invalid_argument("Invalid robot name in speed panel");
    samples_.push_back(Sample{name});
  }
  if (samples_.size()<3 || samples_.size()>4) throw std::invalid_argument("Expected 3 or 4 robots");
  context_=std::make_shared<rclcpp::Context>();context_->init(0,nullptr);
  auto options=rclcpp::NodeOptions().context(context_).use_global_arguments(false);
  options.parameter_overrides({rclcpp::Parameter("use_sim_time",true)});
  node_=std::make_shared<rclcpp::Node>("gazebo_speed_panel",options);
  rclcpp::ExecutorOptions eo;eo.context=context_;
  executor_=std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);
  executor_->add_node(node_);
  for (size_t i=0;i<samples_.size();++i) {
    subscriptions_.push_back(node_->create_subscription<nav_msgs::msg::Odometry>(
      "/"+samples_[i].name+"/odom",rclcpp::SensorDataQoS().keep_last(1),
      [this,i](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        auto &s=samples_[i];const auto &v=msg->twist.twist.linear;
        const auto stamp=static_cast<int64_t>(msg->header.stamp.sec)*1000000000+msg->header.stamp.nanosec;
        if (msg->header.frame_id!="world" || msg->child_frame_id!=s.name+"/base_link" ||
            !std::isfinite(v.x) || !std::isfinite(v.y) || stamp<0) {s.valid=false;return;}
        if (stamp==s.stamp) return; // Replayed samples do not renew the display.
        s.speed=std::hypot(v.x,v.y);s.stamp=stamp;s.valid=true;
        s.received=std::chrono::steady_clock::now();
      }));
  }
  std::vector<std::string> robots;for(const auto &sample:samples_)robots.push_back(sample.name);
  overlay_=std::make_unique<PathOverlay>(node_,robots);
  std::vector<std::array<double,3>> goals;
  if(config)for(auto e=config->FirstChildElement("goal");e;e=e->NextSiblingElement("goal")) {
   std::array<double,3> goal{};
   if(e->QueryDoubleAttribute("x",&goal[0])==tinyxml2::XML_SUCCESS && e->QueryDoubleAttribute("y",&goal[1])==tinyxml2::XML_SUCCESS && e->QueryDoubleAttribute("yaw",&goal[2])==tinyxml2::XML_SUCCESS)goals.push_back(goal);
  }
  overlay_->SetGoals(goals);
  connect(&timer_,&QTimer::timeout,this,[this] {
    executor_->spin_some(std::chrono::milliseconds(4));overlay_->Tick(show_plans_,show_predictions_);emit RowsChanged();
  });
  timer_.start(100);emit RowsChanged();
}
QVariantList RobotSpeedPanel::Rows() const {
  static const char *colors[]={"#56b4ff","#ffb454","#6ddd9b","#d991ff"};
  QVariantList rows;const auto now=std::chrono::steady_clock::now();
  for (size_t i=0;i<samples_.size();++i) {
    const auto &s=samples_[i];const bool fresh=s.valid && std::chrono::duration<double>(now-s.received).count()<=.6;
    QVariantMap row;row["name"]=QString::fromStdString(s.name);row["color"]=colors[i];
    row["speed"]=s.speed;row["fresh"]=fresh;rows.push_back(row);
  }
  return rows;
}
GZ_ADD_PLUGIN(RobotSpeedPanel,gz::gui::Plugin)
