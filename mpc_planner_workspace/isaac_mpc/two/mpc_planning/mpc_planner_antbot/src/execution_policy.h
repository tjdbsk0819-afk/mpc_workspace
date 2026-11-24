#pragma once
#include <algorithm>
#include <cmath>
#include <limits>

namespace AntbotExecution {
inline double wrap(double a) {return std::atan2(std::sin(a),std::cos(a));}

struct Arrival {
  bool aligning{false}, reached{false};
  double stable_since{-1.}, previous{-1.};
  void reset() {*this=Arrival{};}
  void update(double time,double distance,double yaw_error,double speed,double rate,
              double position_tolerance,double yaw_tolerance,double dwell) {
    if(previous>=0. && time<previous) reset();
    previous=time;
    if(reached) return;
    if(aligning && distance>position_tolerance+.03) {aligning=false;stable_since=-1.;}
    if(!aligning && distance<=.75*position_tolerance && std::abs(speed)<=.04) aligning=true;
    if(aligning && distance<=position_tolerance && std::abs(wrap(yaw_error))<=yaw_tolerance &&
       std::abs(speed)<=.03 && std::abs(rate)<=.04) {
      if(stable_since<0.) stable_since=time;
      reached=time-stable_since>=dwell;
    } else stable_since=-1.;
  }
};

struct Motion {double x,y,yaw,v,a,w;};
struct UnboundedWorkspace {
  bool operator()(double,double,double) const {return true;}
};

// Reintegrate a bounded old-plan prefix from CURRENT measured GT, then brake.
// Checking its entire 6-second tail incorrectly requires tracking an old stop
// state after this fallback has already expired. The original epoch never changes.
template<class Sample,class Inflation,class WorkspaceCheck=UnboundedWorkspace>
bool validate_remainder(double age,double maximum_age,double end,Motion measured,
                        double peer_x,double peer_y,double peer_vx,double peer_vy,
                        double separation,double max_speed,double acceleration,double yaw_limit,double yaw_acceleration,
                        Sample sample,Inflation inflation,WorkspaceCheck within={}) {
  for(double v:{age,maximum_age,end,measured.x,measured.y,measured.yaw,measured.v,
                measured.w,peer_x,peer_y,peer_vx,peer_vy,separation,max_speed,acceleration,yaw_limit,yaw_acceleration})
    if(!std::isfinite(v)) return false;
  // Current RPP plan timeout (0.35s) + command reaction allowance (0.25s).
  constexpr double hold=.6;
  if(age<0. || age>maximum_age+1e-9 || end-age<hold || acceleration<=0. || yaw_acceleration<=0.) return false;
  if(!within(measured.x,measured.y,0.)) return false;
  const auto initial=sample(age);
  if(std::hypot(initial.x-measured.x,initial.y-measured.y)>.04 ||
     std::abs(wrap(initial.yaw-measured.yaw))>.10 || std::abs(initial.v-measured.v)>.10) return false;
  const double horizon=hold+std::max(max_speed,std::abs(measured.v))/acceleration+yaw_limit/yaw_acceleration;
  double t=0.,rx=measured.x-peer_x,ry=measured.y-peer_y;
  while(t<horizon-1e-9) {
    const double dt=std::min(.02,horizon-t);
    double target_v=0.,target_w=0.;
    if(t<hold-1e-9) {
      const auto u=sample(age+t+dt);
      if(!std::isfinite(u.v) || !std::isfinite(u.a) || !std::isfinite(u.w) ||
         std::abs(u.a)>acceleration+.001 || std::abs(u.w)>yaw_limit+.001) return false;
      target_v=std::clamp(u.v,0.,max_speed);target_w=u.w;
    }
    const double nv=measured.v+std::clamp(target_v-measured.v,-acceleration*dt,acceleration*dt);
    const double nw=measured.w+std::clamp(target_w-measured.w,-yaw_acceleration*dt,yaw_acceleration*dt);
    const double mid=(measured.v+nv)/2.,rate=(measured.w+nw)/2.;
    measured.x+=mid*std::cos(measured.yaw+.5*rate*dt)*dt;
    measured.y+=mid*std::sin(measured.yaw+.5*rate*dt)*dt;
    measured.yaw+=rate*dt;measured.v=nv;measured.w=nw;t+=dt;
    if(t<=hold+1e-9) {
      const auto reference=sample(age+t);
      if(std::hypot(measured.x-reference.x,measured.y-reference.y)>.10) return false;
    }
    const double nx=measured.x-peer_x-peer_vx*t,ny=measured.y-peer_y-peer_vy*t;
    const double dx=nx-rx,dy=ny-ry,norm=dx*dx+dy*dy;
    const double f=norm<1e-15?0.:std::clamp(-(rx*dx+ry*dy)/norm,0.,1.);
    // Upper endpoint uncertainty and a chord-error bound cover the interval.
    const double curve=(max_speed*yaw_limit+acceleration)*dt*dt/8.;
    if(!within(measured.x,measured.y,curve)) return false;
    const double uncertainty=inflation(t);
    if(!std::isfinite(uncertainty) || uncertainty<0. ||
       std::hypot(rx+f*dx,ry+f*dy)<separation+uncertainty+curve) return false;
    rx=nx;ry=ny;
  }
  return true;
}
}
