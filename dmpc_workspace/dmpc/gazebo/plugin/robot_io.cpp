// Gazebo physics adapter for the existing topic-based ros2_control system.
// Joint feedback is sampled after physics, with the original accepted command.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/plugin/Register.hh>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <antbot_interfaces/msg/joint_command_feedback.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>

namespace antbot_gazebo {
class RobotIO : public gz::sim::System, public gz::sim::ISystemConfigure,
                public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate {
  using JS=sensor_msgs::msg::JointState;
  using Feedback=antbot_interfaces::msg::JointCommandFeedback;
  std::string name_;
  gz::sim::Link base_;
  std::vector<gz::sim::Joint> joints_;
  std::vector<std::string> names_;
  std::array<double,8> targets_{};
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<JS>::SharedPtr sub_;
  rclcpp::Publisher<JS>::SharedPtr states_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<Feedback>::SharedPtr feedback_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_;
  JS::ConstSharedPtr pending_;
  JS applied_;
  builtin_interfaces::msg::Time applied_at_;
  double now_{0},last_time_{-1},next_state_{0},last_command_stamp_{-1};
  bool valid_{false};
  std::chrono::steady_clock::time_point received_;
  static builtin_interfaces::msg::Time stamp(double seconds) {
    auto ns=static_cast<int64_t>(std::max(0.,seconds)*1e9);
    builtin_interfaces::msg::Time t;t.sec=ns/1000000000;t.nanosec=ns%1000000000;return t;
  }
  static double seconds(const builtin_interfaces::msg::Time &t) {return t.sec+t.nanosec*1e-9;}
public:
  void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &) override {
    name_=sdf->Get<std::string>("robot_namespace");
    gz::sim::Model model(entity);
    base_=gz::sim::Link(model.LinkByName(ecm,"base_link"));
    if(!base_.Valid(ecm))throw std::runtime_error("Missing Gazebo base_link");
    base_.EnableVelocityChecks(ecm);
    for(const auto &prefix:{"steering_","wheel_"})for(const auto &corner:{"front_left","front_right","rear_left","rear_right"}) {
      auto name=std::string(prefix)+corner+"_joint";names_.push_back(name);
      gz::sim::Joint joint(model.JointByName(ecm,name));
      if(!joint.Valid(ecm))throw std::runtime_error("Missing Gazebo joint: "+name);
      joint.EnablePositionCheck(ecm);joint.EnableVelocityCheck(ecm);joints_.push_back(joint);
    }
    if(!rclcpp::ok())rclcpp::init(0,nullptr);
    auto options=rclcpp::NodeOptions().use_global_arguments(false);
    options.parameter_overrides({rclcpp::Parameter("use_sim_time",true)});
    node_=std::make_shared<rclcpp::Node>("gazebo_io","/"+name_,options);
    // Metadata used by readiness checks; simulator identity is explicit.
    node_->declare_parameter("backend",std::string("gazebo"));
    node_->declare_parameter("scenario",sdf->Get<std::string>("scenario"));
    node_->declare_parameter("robot_count",sdf->Get<int>("robot_count"));
    states_=node_->create_publisher<JS>("joint_states",rclcpp::SensorDataQoS());
    feedback_=node_->create_publisher<Feedback>("joint_feedback",rclcpp::SensorDataQoS());
    odom_=node_->create_publisher<nav_msgs::msg::Odometry>("odom",rclcpp::QoS(1).reliable());
    tf_=node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf",100);
    sub_=node_->create_subscription<JS>("joint_command",rclcpp::QoS(1).reliable(),
      [this](JS::ConstSharedPtr msg){pending_=msg;});
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    now_=std::chrono::duration<double>(info.simTime).count();
    rclcpp::spin_some(node_);
    if(now_<last_time_)last_command_stamp_=-1.;
    if(now_<last_time_ || info.paused) {valid_=false;pending_.reset();next_state_=0;}
    last_time_=now_;
    if(pending_) {
      auto msg=pending_;pending_.reset();
      bool ok=msg->name.size()==8 && msg->position.size()==8 && msg->velocity.size()==8 &&
        msg->header.stamp.sec>=0 && msg->header.stamp.nanosec<1000000000u &&
        seconds(msg->header.stamp)<=now_+.05 && now_-seconds(msg->header.stamp)<=.5 &&
        std::set<std::string>(msg->name.begin(),msg->name.end()).size()==8;
      std::array<double,8> values{};
      for(size_t i=0;i<8 && ok;++i) {
        auto it=std::find(msg->name.begin(),msg->name.end(),names_[i]);
        if(it==msg->name.end()){ok=false;break;}
        size_t k=it-msg->name.begin();values[i]=i<4?msg->position[k]:msg->velocity[k];
        ok=std::isfinite(values[i]) && std::abs(values[i])<=(i<4?.959+1e-6:50.);
      }
      if(ok && seconds(msg->header.stamp)>last_command_stamp_) {
        last_command_stamp_=seconds(msg->header.stamp);targets_=values;applied_=*msg;applied_at_=stamp(now_);valid_=true;received_=std::chrono::steady_clock::now();
      }
    }
    if(valid_ && (now_-seconds(applied_.header.stamp)>.5 ||
        std::chrono::duration<double>(std::chrono::steady_clock::now()-received_).count()>.5))valid_=false;
    if(info.paused)return;
    for(size_t i=0;i<8;++i) {
      auto pos=joints_[i].Position(ecm);
      double target=0.;
      if(valid_)target=i<4?(pos && !pos->empty()?std::clamp(25.*(targets_[i]-pos->front()),-5.,5.):0.):targets_[i];
      joints_[i].SetVelocity(ecm,{target});
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
    if(info.paused || now_+1e-9<next_state_)return;
    next_state_=now_+.02;
    auto pose=base_.WorldPose(ecm);auto linear=base_.WorldLinearVelocity(ecm);auto angular=base_.WorldAngularVelocity(ecm);
    if(!pose || !linear || !angular)return;
    JS state;state.header.stamp=stamp(now_);state.name=names_;
    for(const auto &joint:joints_) {
      auto p=joint.Position(ecm);auto v=joint.Velocity(ecm);
      if(!p || p->empty() || !v || v->empty())return;
      state.position.push_back(p->front());state.velocity.push_back(v->front());
    }
    states_->publish(state);
    Feedback feedback;feedback.state=state;feedback.command_valid=valid_;
    if(valid_){feedback.applied_command=applied_;feedback.applied_at=applied_at_;}
    feedback_->publish(feedback);
    nav_msgs::msg::Odometry odom;odom.header.stamp=state.header.stamp;odom.header.frame_id="world";
    odom.child_frame_id=name_+"/base_link";
    auto &p=odom.pose.pose.position;p.x=pose->Pos().X();p.y=pose->Pos().Y();p.z=pose->Pos().Z();
    auto &q=odom.pose.pose.orientation;q.x=pose->Rot().X();q.y=pose->Rot().Y();q.z=pose->Rot().Z();q.w=pose->Rot().W();
    auto vb=pose->Rot().RotateVectorReverse(*linear),wb=pose->Rot().RotateVectorReverse(*angular);
    auto &v=odom.twist.twist.linear;v.x=vb.X();v.y=vb.Y();v.z=vb.Z();
    auto &w=odom.twist.twist.angular;w.x=wb.X();w.y=wb.Y();w.z=wb.Z();odom_->publish(odom);
    geometry_msgs::msg::TransformStamped transform;transform.header=odom.header;transform.child_frame_id=odom.child_frame_id;
    transform.transform.translation.x=p.x;transform.transform.translation.y=p.y;transform.transform.translation.z=p.z;
    transform.transform.rotation=q;tf2_msgs::msg::TFMessage tf;tf.transforms.push_back(transform);tf_->publish(tf);
  }
};
}
GZ_ADD_PLUGIN(antbot_gazebo::RobotIO,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate,gz::sim::ISystemPostUpdate)
