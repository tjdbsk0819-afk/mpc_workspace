// MPC-native RPP execution: native Nav2 RPP plugin and final command checks in
// one node. No FollowPath action, external adapter or raw-command topic.
#include "mpc_tracking.hpp"
#include <boost/json/src.hpp>
#include <antbot_interfaces/msg/mpc_plan.hpp>
#include <antbot_interfaces/msg/peer_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/goal_checker.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <nav2_util/node_thread.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <set>
#include <string>

using Plan=antbot_interfaces::msg::MpcPlan;
using Peer=antbot_interfaces::msg::PeerState;
using Odom=nav_msgs::msg::Odometry;
using Twist=geometry_msgs::msg::Twist;
using Path=nav_msgs::msg::Path;
using Wall=std::chrono::steady_clock;
using antbot_tracking::Motion;
namespace tr=antbot_tracking;
namespace json=boost::json;

class MpcRppController : public nav2_util::LifecycleNode {
public:
  MpcRppController():nav2_util::LifecycleNode("mpc_rpp_controller","",rclcpp::NodeOptions()),
    controllers_("nav2_core","nav2_core::Controller"),goals_("nav2_core","nav2_core::GoalChecker") {}

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    ego_=parameter<std::string>("ego_robot","robot_a");
    peers_ids_=parameter<std::vector<std::string>>("peer_robots",{"robot_b","robot_c"});
    enabled_=parameter<bool>("enabled",true);
    for(auto p:{"plan_timeout","state_timeout","command_timeout","goal_frequency","output_frequency",
                "max_speed","max_yaw_rate","minimum_separation","braking_deceleration","reaction_time",
                "max_acceleration","max_yaw_acceleration","tracking_horizon","path_sample_period",
                "minimum_tracking_length","maximum_tracking_horizon","execution_buffer",
                "execution_gap_fraction","proximity_release_duration","alignment_horizon"}) {
      const std::map<std::string,double> defaults={
        {"plan_timeout",.35},{"state_timeout",.35},{"command_timeout",.15},{"goal_frequency",10.},
        {"output_frequency",20.},{"max_speed",1.},{"max_yaw_rate",.8},{"minimum_separation",1.15},
        {"braking_deceleration",.5},{"reaction_time",.25},{"max_acceleration",.5},{"max_yaw_acceleration",.5},
        {"tracking_horizon",.6},{"path_sample_period",.025},{"minimum_tracking_length",.05},
        {"maximum_tracking_horizon",2.},{"execution_buffer",.005},{"execution_gap_fraction",.9},
        {"proximity_release_duration",.15},{"alignment_horizon",.4}};
      settings_[p]=parameter<double>(p,defaults.at(p));
      if(!std::isfinite(cfg(p)) || cfg(p)<=0.) throw std::invalid_argument("Invalid parameter: "+std::string(p));
    }
    if(ego_.empty() || peers_ids_.empty() ||
       std::set<std::string>(peers_ids_.begin(),peers_ids_.end()).size()!=peers_ids_.size() ||
       std::find(peers_ids_.begin(),peers_ids_.end(),ego_)!=peers_ids_.end() || cfg("execution_gap_fraction")>=1.)
      throw std::invalid_argument("Invalid MPC-RPP robot/safety configuration");
    parameter<double>("controller_frequency",cfg("output_frequency"));
    gate_={static_cast<int64_t>(cfg("proximity_release_duration")*1e9),
           static_cast<int64_t>(2e9/cfg("output_frequency"))};
    cmd_=create_publisher<Twist>("cmd_vel",1);
    status_=create_publisher<std_msgs::msg::String>("control_status",1);
    diagnostics_=create_publisher<std_msgs::msg::String>("control_diagnostics",1);
    path_=create_publisher<Path>("rpp_tracking_path",1);
    plan_sub_=create_subscription<Plan>("mpc_plan",1,[this](Plan::ConstSharedPtr p){on_plan(p);});
    odom_sub_=create_subscription<Odom>("odom",rclcpp::SensorDataQoS(),[this](Odom::ConstSharedPtr p){on_odom(p);});
    for(const auto &id:peers_ids_) {
      peers_[id]=nullptr;
      peer_walls_[id]=Wall::time_point{};
      peer_subs_.push_back(create_subscription<Peer>("peers/"+id+"/state",1,
        [this,id](Peer::ConstSharedPtr p){on_peer(p,id);}));
    }
    enable_=create_service<std_srvs::srv::SetBool>("~/set_enabled",
      [this](std::shared_ptr<std_srvs::srv::SetBool::Request> q,
             std::shared_ptr<std_srvs::srv::SetBool::Response> r) {
        enabled_=q->data;set_parameter(rclcpp::Parameter("enabled",enabled_));
        plan_.reset();trajectory_.reset();alignment_.reset();invalid_reason_="waiting_for_new_plan";
        hold(enabled_?invalid_reason_:"disabled");r->success=true;r->message="New valid MPC plan required";
      });
    costmap_=std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "local_costmap",get_namespace(),"local_costmap",get_parameter("use_sim_time").as_bool());
    costmap_thread_=std::make_unique<nav2_util::NodeThread>(costmap_);
    if(costmap_->configure().id()!=lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      throw std::runtime_error("Costmap configuration failed");
    rpp_=controllers_.createSharedInstance("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController");
    goal_=goals_.createSharedInstance("nav2_controller::SimpleGoalChecker");
    goal_->initialize(shared_from_this(),"goal_checker",costmap_);
    rpp_->configure(shared_from_this(),"FollowPath",costmap_->getTfBuffer(),costmap_);
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    cmd_->on_activate();status_->on_activate();diagnostics_->on_activate();path_->on_activate();
    rpp_->activate();
    timer_=create_wall_timer(std::chrono::duration<double>(1./cfg("output_frequency")),[this]{tick();});
    RCLCPP_INFO(get_logger(),"MPC-native RPP active: %s, %zu peers; direct cmd_vel, no adapter/FollowPath",ego_.c_str(),peers_ids_.size());
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    timer_.reset();hold("disabled");rpp_->deactivate();
    if(costmap_active_) {costmap_->deactivate();costmap_active_=false;}
    cmd_->on_deactivate();status_->on_deactivate();diagnostics_->on_deactivate();path_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
    rpp_->cleanup();rpp_.reset();goal_.reset();costmap_->cleanup();costmap_thread_.reset();costmap_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  template<class T> T parameter(const std::string &key,const T &fallback) {
    if(!has_parameter(key)) declare_parameter<T>(key,fallback);
    return get_parameter(key).get_value<T>();
  }
  double cfg(const std::string &key) const { return settings_.at(key); }
  static int64_t stamp(const builtin_interfaces::msg::Time &t) {return int64_t(t.sec)*1000000000+t.nanosec;}
  static double yaw(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  }
  static bool finite(std::initializer_list<double> values) {
    return std::all_of(values.begin(),values.end(),[](double x){return std::isfinite(x);});
  }
  static bool quaternion(const geometry_msgs::msg::Quaternion &q) {
    return finite({q.x,q.y,q.z,q.w}) && std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)<=.1;
  }
  template<class T> bool fresh(const std::shared_ptr<const T> &msg,Wall::time_point received,
                               int64_t current,double timeout) const {
    if(!msg) return false;
    const double age=(current-stamp(msg->header.stamp))*1e-9;
    return age>=-1e-6 && age<=timeout && std::chrono::duration<double>(Wall::now()-received).count()<=timeout;
  }
  bool inputs_fresh(int64_t current) const {
    if(!fresh(odom_,odom_wall_,current,cfg("state_timeout"))) return false;
    for(const auto &id:peers_ids_)
      if(!fresh(peers_.at(id),peer_walls_.at(id),current,cfg("state_timeout"))) return false;
    return true;
  }
  void report(const std::string &text) {
    std_msgs::msg::String message;message.data=text;status_->publish(message);
    diagnostic_["status"]=text;message.data=json::serialize(diagnostic_);diagnostics_->publish(message);
    if(text!=last_status_) RCLCPP_INFO(get_logger(),"%s",text.c_str());
    last_status_=text;
  }
  void emit(double v,double w) {last_output_=Twist();last_output_.linear.x=v;last_output_.angular.z=w;cmd_->publish(last_output_);}
  void hold(const std::string &reason,bool reset_path=true) {
    gate_.interrupt();if(reset_path) has_path_=false;emit(0.,0.);report(reason);
  }
  void on_plan(Plan::ConstSharedPtr p) {
    if(!p->valid) {
      plan_.reset();trajectory_.reset();alignment_.reset();invalid_reason_="mpc_"+p->status;hold(invalid_reason_);return;
    }
    try {
      const auto &path=p->path;const int64_t epoch=stamp(p->header.stamp);
      if(p->header.frame_id!="world" || path.header.frame_id!="world" || epoch!=stamp(path.header.stamp) ||
         path.poses.size()<2 || path.poses.size()>1000 || p->status!="tracking" ||
         !finite({p->target_speed,p->target_yaw_rate,p->minimum_separation,p->max_acceleration,p->max_yaw_rate}) ||
         p->target_speed<0. || p->minimum_separation<=0. || p->max_acceleration<=0. || p->max_yaw_rate<=0. ||
         epoch>now().nanoseconds()+50000000) throw std::invalid_argument("Invalid MPC metadata");
      if(plan_ && epoch<=stamp(plan_->header.stamp)) return;
      tr::Trajectory t;
      t.velocities=p->linear_velocities;t.accelerations=p->linear_accelerations;t.rates=p->angular_velocities;
      for(const auto &pose:path.poses) {
        const auto &v=pose.pose.position;const auto &q=pose.pose.orientation;
        if((!pose.header.frame_id.empty() && pose.header.frame_id!="world") ||
           !finite({v.x,v.y,v.z}) || !quaternion(q)) throw std::invalid_argument("Invalid MPC pose");
        t.times.push_back((stamp(pose.header.stamp)-epoch)*1e-9);t.poses.push_back({v.x,v.y,yaw(q)});
      }
      t.validate();
      for(size_t k=0;k<t.times.size();++k)
        if(std::abs(t.accelerations[k])>p->max_acceleration+.001 ||
           std::abs(t.rates[k])>p->max_yaw_rate+.001 || t.velocities[k]<-.011 || t.velocities[k]>2.)
          throw std::invalid_argument("MPC motion limits violated");
      plan_=p;plan_wall_=Wall::now();trajectory_=std::move(t);
    } catch(const std::exception &e) {
      plan_.reset();trajectory_.reset();alignment_.reset();invalid_reason_=e.what();hold(invalid_reason_);
    }
  }
  void on_odom(Odom::ConstSharedPtr p) {
    const auto &v=p->twist.twist.linear;const auto &x=p->pose.pose.position;
    if(p->header.frame_id!="world" || p->child_frame_id!=ego_+"/base_link" ||
       !finite({x.x,x.y,v.x,v.y,p->twist.twist.angular.z}) || !quaternion(p->pose.pose.orientation)) {
      odom_.reset();hold("invalid_ego_odometry");return;
    }
    if(odom_ && stamp(p->header.stamp)<=stamp(odom_->header.stamp)) return;
    odom_=p;odom_wall_=Wall::now();
  }
  void on_peer(Peer::ConstSharedPtr p,const std::string &id) {
    if(!p->valid || p->robot_id!=id || p->header.frame_id!="world" || !finite({p->x,p->y,p->vx,p->vy})) {
      peers_[id].reset();hold("invalid_peer_state");return;
    }
    if(peers_[id] && stamp(p->header.stamp)<=stamp(peers_[id]->header.stamp)) return;
    peers_[id]=p;peer_walls_[id]=Wall::now();
  }
  void set_path(int64_t current,const Motion &ego,double age) {
    double horizon=cfg("tracking_horizon");auto window=trajectory_->window(age,horizon,cfg("path_sample_period"));
    while(horizon<cfg("maximum_tracking_horizon") &&
          std::hypot(window.back().x-window.front().x,window.back().y-window.front().y)<cfg("minimum_tracking_length") &&
          age+horizon<trajectory_->times.back()) {
      horizon=std::min(cfg("maximum_tracking_horizon"),horizon+.2);
      window=trajectory_->window(age,horizon,cfg("path_sample_period"));
    }
    tracking_mode_="tracking";
    const auto future=trajectory_->sample(std::min(age+1./cfg("output_frequency"),trajectory_->times.back()));
    if(future.v<=.005 && std::abs(future.w)>.02)
      if(const auto count=tr::rotation_prefix_count(window,future.v,future.w)) {
        window.resize(count);tracking_mode_="tracking_mpc_turn";
      }
    if(alignment_) {
      window={Motion{ego.x,ego.y,ego.yaw,0.,0.},Motion{ego.x,ego.y,*alignment_,0.,0.}};
      tracking_mode_="tracking_mpc_heading";
    }
    Path path;path.header.frame_id="world";path.header.stamp=rclcpp::Time(current,get_clock()->get_clock_type());
    for(const auto &p:window) {
      geometry_msgs::msg::PoseStamped pose;pose.header=path.header;
      pose.pose.position.x=p.x;pose.pose.position.y=p.y;
      pose.pose.orientation.z=std::sin(p.yaw/2.);pose.pose.orientation.w=std::cos(p.yaw/2.);path.poses.push_back(pose);
    }
    rpp_->setPlan(path);goal_->reset();path_->publish(path);
    has_path_=true;last_path_wall_=Wall::now();last_plan_stamp_=stamp(plan_->header.stamp);
  }
  void tick() {
    diagnostic_={};const int64_t current=now().nanoseconds();
    if(last_clock_>=0 && current<last_clock_) {
      plan_.reset();odom_.reset();trajectory_.reset();alignment_.reset();has_path_=false;
      for(auto &[id,p]:peers_) p.reset();invalid_reason_="clock_reset";
    }
    const double dt=last_clock_<0?1./cfg("output_frequency"):
      std::clamp((current-last_clock_)*1e-9,0.,2./cfg("output_frequency"));last_clock_=current;
    if(!enabled_) {hold("disabled");return;}
    if(!inputs_fresh(current)) {hold("waiting_for_fresh_gt");return;}
    if(!fresh(plan_,plan_wall_,current,cfg("plan_timeout"))) {hold(plan_?"expired_mpc_plan":invalid_reason_);return;}
    if(dt<=0.) {report("waiting_for_clock");return;}
    try { execute(current,dt); }
    catch(const std::exception &e) {diagnostic_["error"]=e.what();hold("rpp_execution_error");}
  }
  void execute(int64_t current,double dt) {
    const auto &p=odom_->pose.pose.position;const auto &v=odom_->twist.twist.linear;
    if(std::abs(v.y)>.05) {hold("unicycle_tracking_error");return;}
    const double accel=std::min(cfg("max_acceleration"),plan_->max_acceleration);
    Motion ego{p.x,p.y,yaw(odom_->pose.pose.orientation),v.x,odom_->twist.twist.angular.z};
    const double ego_age=std::max(0.,(current-stamp(odom_->header.stamp))*1e-9);
    ego=tr::step(ego,ego.v,ego.w,ego_age,accel,cfg("max_yaw_acceleration"));
    const double age=std::max(0.,(current-stamp(plan_->header.stamp))*1e-9);
    const auto reference=trajectory_->sample(age+dt);const double limit=std::min(cfg("max_speed"),std::max(0.,reference.v));
    if(limit<=.001 && std::abs(reference.w)<=.02) {hold("mpc_stop");return;}
    if(alignment_) {
      alignment_=tr::alignment_heading(*trajectory_,age,ego.yaw,reference.w,cfg("alignment_horizon"));
      if(reference.v>.01 || std::abs(reference.w)<=.02) alignment_.reset();
      if(!alignment_) has_path_=false;
    }
    if(!costmap_active_) {
      if(!costmap_->getTfBuffer()->canTransform("world",ego_+"/base_link",tf2::TimePointZero)) {
        hold("waiting_for_rpp_transform");return;
      }
      if(costmap_->activate().id()!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        hold("waiting_for_rpp_costmap");return;
      }
      costmap_active_=true;
    }
    const auto begin=Wall::now();
    if(!has_path_ || (std::chrono::duration<double>(begin-last_path_wall_).count()>=1./cfg("goal_frequency") &&
                     stamp(plan_->header.stamp)>last_plan_stamp_)) set_path(current,ego,age);
    rpp_->setSpeedLimit(std::max(.001,limit),false);
    geometry_msgs::msg::PoseStamped pose;
    if(!costmap_->getRobotPose(pose)) {hold("waiting_for_rpp_transform");return;}
    const Twist raw=rpp_->computeVelocityCommands(pose,odom_->twist.twist,goal_.get()).twist;
    if(!finite({raw.linear.x,raw.linear.y,raw.linear.z,raw.angular.x,raw.angular.y,raw.angular.z})) {
      hold("invalid_rpp_command");return;
    }
    if(std::chrono::duration<double>(Wall::now()-begin).count()>cfg("command_timeout") ||
       !inputs_fresh(now().nanoseconds()) || !fresh(plan_,plan_wall_,now().nanoseconds(),cfg("plan_timeout"))) {
      hold("expired_rpp_command");return;
    }
    const double target_v=std::min({limit,std::max(0.,raw.linear.x),std::max(0.,ego.v)+accel*dt});
    const double yaw_limit=std::min(cfg("max_yaw_rate"),plan_->max_yaw_rate);
    double yaw_request=std::clamp(raw.angular.z,-yaw_limit,yaw_limit);
    if(raw.linear.x>1e-4 && reference.v>.005) yaw_request*=std::min(1.,target_v/raw.linear.x);
    double target_w=tr::approach(last_output_.angular.z,yaw_request,cfg("max_yaw_acceleration")*dt);
    const double lead=cfg("max_yaw_acceleration")*cfg("reaction_time");
    target_w=std::clamp(std::clamp(target_w,ego.w-lead,ego.w+lead),-yaw_limit,yaw_limit);
    const double nominal=std::max(cfg("minimum_separation"),plan_->minimum_separation);
    std::vector<tr::Neighbor> neighbors;json::array checks;double maximum_age=0.,separation=1e9,checked=1e9;
    for(const auto &id:peers_ids_) {
      const auto &s=*peers_.at(id);const double peer_age=std::max(0.,(current-stamp(s.header.stamp))*1e-9);
      const double x=s.x+s.vx*peer_age,y=s.y+s.vy*peer_age,distance=std::hypot(ego.x-x,ego.y-y);
      const double bound=nominal+std::min(cfg("execution_buffer"),std::max(0.,(distance-nominal)*cfg("execution_gap_fraction")));
      neighbors.push_back({x,y,s.vx,s.vy,bound});maximum_age=std::max(maximum_age,peer_age);
      separation=std::min(separation,distance);checked=std::min(checked,bound);
      checks.push_back(json::object{{"robot_id",id},{"age",peer_age},{"separation",distance},{"checked_separation",bound}});
    }
    const auto sampled=trajectory_->sample(age);
    diagnostic_={{"plan_age",age},{"ego_age",ego_age},{"peer_age",maximum_age},{"profile_speed",reference.v},
      {"profile_yaw_rate",reference.w},{"position_error",std::hypot(ego.x-sampled.x,ego.y-sampled.y)},
      {"measured_speed",ego.v},{"measured_yaw_rate",ego.w},{"rpp_speed",raw.linear.x},{"rpp_yaw_rate",raw.angular.z},
      {"nominal_separation",nominal},{"checked_separation",checked},{"separation",separation},{"peer_checks",checks}};
    const auto selected=tr::filter_command(ego,target_v,target_w,neighbors,cfg("reaction_time"),accel,
      cfg("max_yaw_acceleration"),cfg("braking_deceleration"),reference.w,raw.angular.z);
    if(!selected) gate_.stop();
    const bool blocked=!selected || selected->v<=.001;
    const bool wrong_turn=std::abs(reference.w)>.02 &&
      (raw.angular.z*reference.w<=0. || std::abs(raw.angular.z)<.25*std::abs(reference.w));
    if(!alignment_ && blocked && wrong_turn) {
      alignment_=tr::alignment_heading(*trajectory_,age,ego.yaw,reference.w,cfg("alignment_horizon"));
      if(alignment_) {has_path_=false;hold("waiting_for_mpc_heading",false);return;}
    }
    if(!selected) {diagnostic_["candidate_rejected"]=true;hold("proximity_stop",false);return;}
    const auto &s=*selected;const bool rotation=s.scale==0. && std::abs(target_v)>1e-4 && std::abs(s.w)>1e-4;
    diagnostic_["command_speed"]=s.v;diagnostic_["command_yaw_rate"]=s.w;
    diagnostic_["predicted_clearance"]=s.clearance;diagnostic_["speed_scale"]=s.scale;
    diagnostic_["rotation_only"]=rotation;diagnostic_["candidate_speed"]=s.v;diagnostic_["candidate_yaw_rate"]=s.w;
    if(!gate_.allow(current)) {
      diagnostic_["command_speed"]=0.;diagnostic_["command_yaw_rate"]=0.;emit(0.,0.);report("proximity_recovery_wait");return;
    }
    emit(s.v,s.w);report(rotation?"tracking_safe_rotation":s.scale<1.?"tracking_speed_reduced":tracking_mode_);
  }

  pluginlib::ClassLoader<nav2_core::Controller> controllers_;
  pluginlib::ClassLoader<nav2_core::GoalChecker> goals_;
  nav2_core::Controller::Ptr rpp_;nav2_core::GoalChecker::Ptr goal_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
  std::map<std::string,double> settings_;std::string ego_,last_status_,tracking_mode_,invalid_reason_{"waiting_for_fresh_mpc_plan"};
  std::vector<std::string> peers_ids_;std::map<std::string,Peer::ConstSharedPtr> peers_;
  std::map<std::string,Wall::time_point> peer_walls_;
  Plan::ConstSharedPtr plan_;Odom::ConstSharedPtr odom_;std::optional<tr::Trajectory> trajectory_;
  Wall::time_point plan_wall_,odom_wall_,last_path_wall_;std::optional<double> alignment_;
  int64_t last_clock_{-1},last_plan_stamp_{-1};bool enabled_{true},has_path_{false},costmap_active_{false};
  tr::ReleaseGate gate_{150000000,100000000};Twist last_output_;json::object diagnostic_;
  rclcpp_lifecycle::LifecyclePublisher<Twist>::SharedPtr cmd_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_,diagnostics_;
  rclcpp_lifecycle::LifecyclePublisher<Path>::SharedPtr path_;
  rclcpp::Subscription<Plan>::SharedPtr plan_sub_;rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  std::vector<rclcpp::Subscription<Peer>::SharedPtr> peer_subs_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<MpcRppController>();
    if(node->configure().id()!=lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
       node->activate().id()!=lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      throw std::runtime_error("MPC RPP configuration/activation failed");
    rclcpp::spin(node->get_node_base_interface());
    if(rclcpp::ok()) {node->deactivate();node->cleanup();}
  } catch(const std::exception &e) {
    std::cerr<<"MPC RPP: "<<e.what()<<std::endl;
    if(rclcpp::ok()) rclcpp::shutdown();return 1;
  }
  if(rclcpp::ok()) rclcpp::shutdown();return 0;
}
