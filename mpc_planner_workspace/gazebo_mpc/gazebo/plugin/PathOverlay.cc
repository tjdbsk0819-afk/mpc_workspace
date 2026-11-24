#include "PathOverlay.hh"
#include <gz/msgs/boolean.pb.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>

namespace {
using Point=std::array<double,2>;
const std::array<std::array<double,3>,4> colors{{{.337,.706,1.},{1.,.706,.329},{.427,.867,.608},{.851,.569,1.}}};
void triangle(gz::msgs::Marker &m,const Point &a,const Point &b,const Point &c,double z) {
 for (const auto &p:{a,b,c}) {auto *v=m.add_point();v->set_x(p[0]);v->set_y(p[1]);v->set_z(z);}
}
void strip(gz::msgs::Marker &m,const Point &a,const Point &b,double width,double z) {
 const double dx=b[0]-a[0],dy=b[1]-a[1],length=std::hypot(dx,dy);
 if(length<1e-8)return;
 const double nx=-dy/length*width*.5,ny=dx/length*width*.5;
 const Point al{a[0]+nx,a[1]+ny},ar{a[0]-nx,a[1]-ny},bl{b[0]+nx,b[1]+ny},br{b[0]-nx,b[1]-ny};
 triangle(m,al,ar,br,z);triangle(m,al,br,bl,z);
}
gz::msgs::Marker deletion(const std::string &ns) {
 gz::msgs::Marker m;m.set_ns(ns);m.set_id(1);m.set_action(gz::msgs::Marker::DELETE_MARKER);return m;
}
}

gz::msgs::Marker PathOverlay::Ribbon(const nav_msgs::msg::Path &path,const std::string &ns,
                                     const std::array<double,3> &color,bool dashed,double z) {
 if(path.header.frame_id!="world" || path.poses.size()<2 || path.poses.size()>500)
  throw std::invalid_argument("Invalid path frame or length");
 std::vector<Point> points;
 double total=0.;
 for(const auto &pose:path.poses) {
  const auto &p=pose.pose.position;
  if((!pose.header.frame_id.empty() && pose.header.frame_id!="world") ||
     !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || std::abs(p.x)>1000. || std::abs(p.y)>1000.)
   throw std::invalid_argument("Invalid path coordinates");
  if(!points.empty())total+=std::hypot(p.x-points.back()[0],p.y-points.back()[1]);
  points.push_back({p.x,p.y});
 }
 if(total>100.)throw std::invalid_argument("Unbounded path length");
 gz::msgs::Marker marker;marker.set_ns(ns);marker.set_id(1);marker.set_action(gz::msgs::Marker::ADD_MODIFY);
 marker.set_type(gz::msgs::Marker::TRIANGLE_LIST);marker.set_visibility(gz::msgs::Marker::GUI);
 marker.mutable_lifetime()->set_nsec(700000000);
 auto *material=marker.mutable_material();material->set_lighting(false);
 for(auto *c:{material->mutable_diffuse(),material->mutable_ambient(),material->mutable_emissive()}) {
  c->set_r(color[0]);c->set_g(color[1]);c->set_b(color[2]);c->set_a(dashed?.8:1.);
 }
 double travelled=0.;
 for(size_t k=1;k<points.size();++k) {
  const auto &a=points[k-1],&b=points[k];const double dx=b[0]-a[0],dy=b[1]-a[1],length=std::hypot(dx,dy);
  if(length<1e-8)continue;
  if(!dashed)strip(marker,a,b,.045,z);
  else {
   // Dash length is measured in metres, independent of prediction knot spacing.
   double offset=0.;
   while(offset<length-1e-8) {
    const double phase=std::fmod(travelled+offset,.30);
    const bool draw=phase<.18;
    const double step=std::min(length-offset,(draw?.18:.30)-phase+1e-9);
    if(draw)strip(marker,{a[0]+dx*offset/length,a[1]+dy*offset/length},
                        {a[0]+dx*(offset+step)/length,a[1]+dy*(offset+step)/length},.025,z);
    offset+=step;
   }
  }
  travelled+=length;
 }
 return marker;
}
PathOverlay::PathOverlay(rclcpp::Node::SharedPtr node,const std::vector<std::string> &robots):node_(node) {
 channels_.reserve(robots.size()*robots.size());
 std::vector<std::string> topics;
 for(size_t ego=0;ego<robots.size();++ego) {
  Channel own;own.ns="antbot_paths/"+robots[ego]+"/plan";own.color=colors[ego];
  channels_.push_back(own);topics.push_back("/"+robots[ego]+"/mpc_path");
  for(size_t peer=0;peer<robots.size();++peer)if(peer!=ego) {
   Channel forecast;forecast.ns="antbot_paths/"+robots[ego]+"/prediction/"+robots[peer];
   forecast.prediction=true;forecast.color=colors[peer];forecast.height=.12+.004*ego;
   channels_.push_back(forecast);topics.push_back("/"+robots[ego]+"/mpc_peer_prediction/"+robots[peer]);
  }
 }
 for(size_t i=0;i<topics.size();++i)subscriptions_.push_back(node_->create_subscription<nav_msgs::msg::Path>(
  topics[i],rclcpp::QoS(1).best_effort(),[this,i](nav_msgs::msg::Path::ConstSharedPtr msg){Receive(i,msg);}));
 for(size_t i=0;i<topics.size();++i)if(channels_[i].prediction) {
  auto topic=topics[i];topic.replace(topic.find("mpc_peer_prediction"),19,"mpc_peer_distribution");
  distributions_.push_back(node_->create_subscription<std_msgs::msg::String>(topic,1,
   [this,i](std_msgs::msg::String::ConstSharedPtr msg){Distribution(i,msg->data);}));
 }
}
void PathOverlay::Receive(size_t index,nav_msgs::msg::Path::ConstSharedPtr path) {
 auto &c=channels_[index];const auto now=node_->get_clock()->now().nanoseconds();
 const auto stamp=static_cast<int64_t>(path->header.stamp.sec)*1000000000+path->header.stamp.nanosec;
 if(path->poses.empty()){c.valid=false;return;}
 if(path->header.stamp.sec<0 || path->header.stamp.nanosec>=1000000000u || now<=0 ||
    now-stamp>500000000 || stamp-now>100000000) {c.valid=false;return;}
 if(stamp==c.stamp)return; // A replay cannot keep an old overlay visible.
 try {c.marker=Ribbon(*path,c.ns,c.color,c.prediction,c.height);}
 catch(const std::invalid_argument &){c.valid=false;return;}
 c.valid=true;c.stamp=stamp;c.received=std::chrono::steady_clock::now();
}
void PathOverlay::SetGoals(const std::vector<std::array<double,3>> &goals) {
 goals_.clear();
 for(size_t i=0;i<goals.size();++i) {
  nav_msgs::msg::Path ring;ring.header.frame_id="world";
  const auto &g=goals[i];
  for(int k=0;k<=48;++k){geometry_msgs::msg::PoseStamped p;p.pose.position.x=g[0]+.35*std::cos(k*2*M_PI/48);p.pose.position.y=g[1]+.35*std::sin(k*2*M_PI/48);ring.poses.push_back(p);}
  auto marker=Ribbon(ring,"antbot_goals/robot_"+std::string(1,'a'+i),colors[i],false,.9);
  strip(marker,{g[0]-.15,g[1]},{g[0]+.15,g[1]},.035,.9);
  strip(marker,{g[0],g[1]-.15},{g[0],g[1]+.15},.035,.9);
  strip(marker,{g[0],g[1]},{g[0]+.40*std::cos(g[2]),g[1]+.40*std::sin(g[2])},.035,.9);
  goals_.push_back(marker);
 }
}
void PathOverlay::Distribution(size_t i,const std::string &text) {
 auto &c=channels_[i];c.gaussian_valid=false;
 const auto doc=QJsonDocument::fromJson(QByteArray::fromStdString(text));if(!doc.isObject())return;
 const auto obj=doc.object();const auto model=obj.value("model").toString();
 if(model!="cv_gaussian" && model!="gaussian_random_walk")return;
 const double stamp=obj.value("stamp").toDouble(-1);
 const double now=node_->get_clock()->now().seconds();
 if(!std::isfinite(stamp)||stamp<0||now-stamp>.5||stamp-now>.1)return;
 const auto ns=static_cast<int64_t>(stamp*1e9);if(ns==c.gaussian_stamp)return;
 const auto steps=obj.value("steps").toArray();if(steps.empty()||steps.size()>1000)return;
 gz::msgs::Marker combined;
 for(int index:{0,static_cast<int>(steps.size())/2,static_cast<int>(steps.size())-1}) {
  const auto step=steps[index].toObject();const double x=step.value("x").toDouble(NAN),y=step.value("y").toDouble(NAN),r=step.value("radius").toDouble(NAN);
  if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(r)||r<0||r>30||std::abs(x)>1000||std::abs(y)>1000)return;
  if(r<1e-6)continue;
  nav_msgs::msg::Path ring;ring.header.frame_id="world";
  for(int k=0;k<=64;++k){geometry_msgs::msg::PoseStamped p;p.pose.position.x=x+r*std::cos(k*2*M_PI/64);p.pose.position.y=y+r*std::sin(k*2*M_PI/64);ring.poses.push_back(p);}
  gz::msgs::Marker marker;
  try {marker=Ribbon(ring,c.ns+"/uncertainty",c.color,false,.105);}catch(const std::invalid_argument &){return;}
  if(combined.point_size()==0)combined=marker;
  else for(const auto &point:marker.point())*combined.add_point()=point;
 }
 if(combined.point_size()==0)return;
 combined.mutable_material()->mutable_diffuse()->set_a(.35);
 combined.mutable_material()->mutable_ambient()->set_a(.35);
 combined.mutable_material()->mutable_emissive()->set_a(.35);
 c.gaussian=combined;c.gaussian_valid=true;c.gaussian_stamp=ns;c.gaussian_received=std::chrono::steady_clock::now();
}
void PathOverlay::Send(const gz::msgs::Marker_V &batch) {
 if(batch.marker_size()==0)return;
 transport_.Request<gz::msgs::Marker_V,gz::msgs::Boolean>("/marker_array",batch,
  [](const gz::msgs::Boolean &,const bool){});
}
void PathOverlay::Tick(bool plans,bool predictions) {
 const auto now=node_->get_clock()->now().nanoseconds();const auto wall=std::chrono::steady_clock::now();
 const bool reset=last_clock_>=0 && now<last_clock_;last_clock_=now;
 gz::msgs::Marker_V batch;
 for(const auto &goal:goals_)*batch.add_marker()=goal;
 for(auto &c:channels_) {
  if(reset){c.valid=false;c.stamp=-1;c.gaussian_valid=false;c.gaussian_stamp=-1;}
  const bool fresh=c.valid && now>=c.stamp && now-c.stamp<=500000000 &&
   std::chrono::duration<double>(wall-c.received).count()<=.5;
  if(fresh && (c.prediction?predictions:plans) && c.marker.point_size()>0) {
   *batch.add_marker()=c.marker;c.shown=true;
  } else if(c.shown){*batch.add_marker()=deletion(c.ns);c.shown=false;}
  const bool uncertainty=c.gaussian_valid && predictions && now>=c.gaussian_stamp && now-c.gaussian_stamp<=500000000 && std::chrono::duration<double>(wall-c.gaussian_received).count()<=.5;
  if(uncertainty){*batch.add_marker()=c.gaussian;c.gaussian_shown=true;}
  else if(c.gaussian_shown){*batch.add_marker()=deletion(c.ns+"/uncertainty");c.gaussian_shown=false;}
 }
 Send(batch);
}
PathOverlay::~PathOverlay() {
 gz::msgs::Marker_V batch;
 for(const auto &goal:goals_)*batch.add_marker()=deletion(goal.ns());
 for(const auto &c:channels_){if(c.shown)*batch.add_marker()=deletion(c.ns);if(c.gaussian_shown)*batch.add_marker()=deletion(c.ns+"/uncertainty");}
 Send(batch);
}
