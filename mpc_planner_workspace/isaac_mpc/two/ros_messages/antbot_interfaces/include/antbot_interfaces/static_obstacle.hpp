#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace antbot_workspace {
// A known stationary cylinder: no CV drift and no Gaussian inflation.
struct StaticCircle {
  bool enabled{false};double x{0.},y{0.},radius{.4},robot_radius{.5},margin{.15};
  void configure(bool active,const std::vector<double> &circle,double body,double buffer) {
    if(circle.size()!=3) throw std::invalid_argument("static_obstacle must be [x,y,radius]");
    enabled=active;x=circle[0];y=circle[1];radius=circle[2];robot_radius=body;margin=buffer;
    for(double v:{x,y,radius,robot_radius,margin})
      if(!std::isfinite(v)) throw std::invalid_argument("Nonfinite static obstacle");
    if(radius<=0. || robot_radius<=0. || margin<0.) throw std::invalid_argument("Invalid static obstacle radius/margin");
  }
  double separation() const {return radius+robot_radius+margin;}
  double clearance(double px,double py) const {
    if(!enabled) return std::numeric_limits<double>::infinity();
    if(!std::isfinite(px) || !std::isfinite(py)) return -std::numeric_limits<double>::infinity();
    return std::hypot(px-x,py-y)-separation();
  }
  double segment_clearance(double ax,double ay,double bx,double by) const {
    if(!enabled) return std::numeric_limits<double>::infinity();
    const double dx=bx-ax,dy=by-ay,norm=dx*dx+dy*dy;
    const double f=norm<1e-15?0.:std::clamp(((x-ax)*dx+(y-ay)*dy)/norm,0.,1.);
    return clearance(ax+f*dx,ay+f*dy);
  }
  bool interval_safe(double px,double py,double yaw,double v,double a,double w,double duration,double buffer=0.) const {
    if(!enabled) return true;
    for(double value:{px,py,yaw,v,a,w,duration,buffer}) if(!std::isfinite(value)) return false;
    if(duration<=0. || buffer<0.) return false;
    const int count=std::max(1,static_cast<int>(std::ceil(duration/.01)));const double dt=duration/count;
    for(int k=0;k<count;++k) {
      const double vm=v+.5*a*dt,ve=v+a*dt;
      const double nx=px+dt/6.*(v*std::cos(yaw)+4.*vm*std::cos(yaw+.5*w*dt)+ve*std::cos(yaw+w*dt));
      const double ny=py+dt/6.*(v*std::sin(yaw)+4.*vm*std::sin(yaw+.5*w*dt)+ve*std::sin(yaw+w*dt));
      const double bend=(std::abs(a)+std::max(std::abs(v),std::abs(ve))*std::abs(w))*dt*dt/8.;
      if(segment_clearance(px,py,nx,ny)+1e-7<buffer+bend) return false;
      px=nx;py=ny;yaw+=w*dt;v=ve;
    }
    return true;
  }
};
}
