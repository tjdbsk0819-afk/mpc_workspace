#pragma once
// Time-aligned MPC execution and the same bounded command/CV guard used before
// integration. This code does not generate alternative avoidance paths.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>
#include <antbot_interfaces/peer_prediction.hpp>
#include <antbot_interfaces/workspace_bounds.hpp>
#include <antbot_interfaces/static_obstacle.hpp>

namespace antbot_tracking {
inline double wrap(double x) { return std::atan2(std::sin(x),std::cos(x)); }
inline double approach(double x,double target,double amount) {
  return x+std::clamp(target-x,-amount,amount);
}
struct Motion { double x,y,yaw,v,w; };
inline bool use_rotation_profile(double requested_speed,double rate,double measured_speed,double threshold=.02) {
  return std::abs(requested_speed)<=threshold && std::abs(measured_speed)<=threshold && std::abs(rate)>.02;
}
inline Motion step(Motion s,double v,double w,double dt,double accel,double yaw_accel) {
  const double nv=approach(s.v,v,accel*dt),nw=approach(s.w,w,yaw_accel*dt);
  const double vm=(s.v+nv)/2.,wm=(s.w+nw)/2.;
  return {s.x+vm*std::cos(s.yaw+wm*dt/2.)*dt,
          s.y+vm*std::sin(s.yaw+wm*dt/2.)*dt,s.yaw+wm*dt,nv,nw};
}
struct Trajectory {
  std::vector<double> times,velocities,accelerations,rates;
  std::vector<std::array<double,3>> poses;
  void validate() const {
    const size_t n=times.size();
    if(n<2 || n!=poses.size() || n!=velocities.size() || n!=accelerations.size() || n!=rates.size())
      throw std::invalid_argument("Missing MPC motion profile");
    if(std::abs(times[0])>1e-9) throw std::invalid_argument("Invalid MPC time origin");
    for(size_t k=0;k<n;++k) {
      if(!std::isfinite(times[k]) || (k && times[k]<=times[k-1]))
        throw std::invalid_argument("Invalid MPC time grid");
      for(double x:{poses[k][0],poses[k][1],poses[k][2],velocities[k],accelerations[k],rates[k]})
        if(!std::isfinite(x)) throw std::invalid_argument("Nonfinite MPC motion profile");
    }
  }
  Motion sample(double seconds) const {
    if(times.empty() || seconds < -1e-6 || seconds>times.back()+1e-6)
      throw std::invalid_argument("MPC motion profile expired");
    const double t=std::clamp(seconds,0.,times.back());
    const auto it=std::upper_bound(times.begin(),times.end(),t);
    const size_t k=it==times.begin()?0:static_cast<size_t>(it-times.begin()-1);
    const double tau=t-times[k],v=velocities[k],a=accelerations[k],w=rates[k],yaw=poses[k][2];
    const double mid=v+.5*a*tau,end=v+a*tau;
    return {poses[k][0]+tau/6.*(v*std::cos(yaw)+4*mid*std::cos(yaw+.5*w*tau)+end*std::cos(yaw+w*tau)),
            poses[k][1]+tau/6.*(v*std::sin(yaw)+4*mid*std::sin(yaw+.5*w*tau)+end*std::sin(yaw+w*tau)),
            yaw+w*tau,end,w};
  }
  std::vector<Motion> window(double age,double horizon,double period) const {
    const double end=std::min(age+horizon,times.back());
    if(end<=age) throw std::invalid_argument("Empty MPC tracking window");
    const int n=std::max(1,static_cast<int>(std::ceil((end-age)/period)));
    std::vector<Motion> result;
    for(int k=0;k<=n;++k) result.push_back(sample(age+(end-age)*k/n));
    return result;
  }
};
inline std::optional<double> alignment_heading(const Trajectory &t,double age,double yaw,double rate,double horizon) {
  double end=std::min(age+horizon,t.times.back());
  for(size_t k=0;k<t.times.size();++k)
    if(age<t.times[k] && t.times[k]<end && t.rates[k]*rate<=0.) { end=t.times[k];break; }
  const double target=t.sample(end).yaw,error=wrap(target-yaw);
  if(std::abs(error)>.005 && error*rate>0.) return target;
  return {};
}
inline size_t rotation_prefix_count(const std::vector<Motion> &p,double speed,double rate) {
  if(std::abs(speed)>.005 || std::abs(rate)<=.02 || p.size()<2) return 0;
  size_t count=0;
  for(size_t k=1;k<p.size();++k) {
    if(std::hypot(p[k].x-p[0].x,p[k].y-p[0].y)>.004) break;
    const double delta=wrap(p[k].yaw-p[0].yaw);
    if(delta*rate>0. && std::abs(delta)>=.006) count=k+1;
  }
  return count;
}
struct Neighbor {
  double x,y,vx,vy,separation;
  antbot_prediction::Model forecast{};
  double prediction_age{0.};
};
inline double clearance(Motion current,double v,double w,const Neighbor &p,
                        double hold,double accel,double yaw_accel,double brake) {
  const double horizon=hold+std::max(std::abs(current.v),std::abs(v))/brake+
    std::max(std::abs(current.w),std::abs(w))/yaw_accel;
  const int count=std::max(1,static_cast<int>(std::ceil(horizon/.005)));
  const double dt=horizon/count;
  double closest=std::hypot(current.x-p.x,current.y-p.y)-p.forecast.radius(p.prediction_age);
  double rx=current.x-p.x,ry=current.y-p.y;
  for(int k=1;k<=count;++k) {
    const double t=k*dt;const bool holding=(k-1)*dt<hold;
    current=step(current,holding?v:0.,holding?w:0.,dt,holding?accel:brake,yaw_accel);
    const auto mean=p.forecast.mean(p.x,p.y,p.vx,p.vy,t);
    const double nx=current.x-mean[0],ny=current.y-mean[1],dx=nx-rx,dy=ny-ry,norm=dx*dx+dy*dy;
    const double f=norm<1e-15?0.:std::clamp(-(rx*dx+ry*dy)/norm,0.,1.);
    closest=std::min(closest,std::hypot(rx+f*dx,ry+f*dy)-p.forecast.radius(p.prediction_age+t));rx=nx;ry=ny;
  }
  return closest;
}
struct Selected { double v,w,clearance,scale; };
inline double wall_clearance(Motion current,double v,double w,const antbot_workspace::Bounds &bounds,
                             double hold,double accel,double yaw_accel,double brake) {
  if(!bounds.enabled) return std::numeric_limits<double>::infinity();
  const double horizon=hold+std::max(std::abs(current.v),std::abs(v))/brake+
    std::max(std::abs(current.w),std::abs(w))/yaw_accel;
  const int count=std::max(1,static_cast<int>(std::ceil(horizon/.005)));
  const double dt=horizon/count;double closest=bounds.clearance(current.x,current.y);
  for(int k=1;k<=count;++k) {
    const bool holding=(k-1)*dt<hold;
    const double initial=bounds.clearance(current.x,current.y),old_v=current.v,old_w=current.w;
    current=step(current,holding?v:0.,holding?w:0.,dt,holding?accel:brake,yaw_accel);
    const double bend=(std::abs(current.v-old_v)/dt+std::max(std::abs(old_v),std::abs(current.v))*
        std::max(std::abs(old_w),std::abs(current.w)))*dt*dt/8.;
    closest=std::min(closest,std::min(initial,bounds.clearance(current.x,current.y))-bend);
  }
  return closest;
}
inline std::optional<Selected> filter_command(Motion state,double v,double w,
    const std::vector<Neighbor> &neighbors,double hold,double accel,double yaw_accel,double brake,
    double mpc_rate,double rpp_rate,const antbot_workspace::Bounds &bounds={}) {
  if(neighbors.empty()) throw std::invalid_argument("At least one neighbor required");
  auto checked=[&](double cv,double cw)->std::optional<double> {
    if(wall_clearance(state,cv,cw,bounds,hold,accel,yaw_accel,brake)<0.) return {};
    double closest=std::numeric_limits<double>::infinity();
    for(const auto &p:neighbors) {
      const double d=clearance(state,cv,cw,p,hold,accel,yaw_accel,brake);
      if(d<p.separation) return {};
      closest=std::min(closest,d);
    }
    return closest;
  };
  const bool moving=std::abs(v)>1e-4;
  for(double scale:{1.,.75,.5,.25,.125}) {
    const double cv=v*scale,cw=moving?w*scale:w;
    if(auto d=checked(cv,cw)) return Selected{cv,cw,*d,scale};
  }
  if(moving && std::abs(w)>1e-4 && std::isfinite(mpc_rate) && std::isfinite(rpp_rate) && w*mpc_rate>0. && w*rpp_rate>0.)
    if(auto d=checked(0.,w)) return Selected{0.,w,*d,0.};
  if(auto d=checked(0.,0.)) return Selected{0.,0.,*d,0.};
  return {};
}
struct ReleaseGate {
  int64_t duration,maximum_gap,safe_since{-1},previous{-1};bool blocked{false};
  void interrupt() { safe_since=previous=-1; }
  void stop() { blocked=true;interrupt(); }
  bool allow(int64_t now) {
    if(!blocked) return true;
    if(previous<0 || now<previous || now-previous>maximum_gap) safe_since=now;
    previous=now;
    if(now-safe_since<duration) return false;
    blocked=false;interrupt();return true;
  }
};
} // namespace antbot_tracking
