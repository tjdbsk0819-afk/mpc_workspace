#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace antbot_workspace {
// Inner faces of four physical walls, inset by the same circular footprint
// and tracking allowance in the OCP and final command/braking checks.
struct Bounds {
  bool enabled{false};
  std::array<double,4> walls{{-2000.,2000.,-2000.,2000.}};
  double radius{.5},margin{.05};
  void configure(bool active,const std::vector<double> &limits,double body,double buffer) {
    if(limits.size()!=4) throw std::invalid_argument("workspace_bounds must be [xmin,xmax,ymin,ymax]");
    enabled=active;std::copy(limits.begin(),limits.end(),walls.begin());radius=body;margin=buffer;
    for(double v:{walls[0],walls[1],walls[2],walls[3],radius,margin})
      if(!std::isfinite(v)) throw std::invalid_argument("Nonfinite workspace bounds");
    if(radius<=0. || margin<0. || walls[0]>=walls[1] || walls[2]>=walls[3] ||
       (enabled && (walls[1]-walls[0]<=2.*(radius+margin) || walls[3]-walls[2]<=2.*(radius+margin))))
      throw std::invalid_argument("Workspace is too small for the robot footprint and margin");
  }
  std::array<double,4> centers() const {
    if(!enabled) return {{-2000.,2000.,-2000.,2000.}};
    const double inset=radius+margin;
    return {{walls[0]+inset,walls[1]-inset,walls[2]+inset,walls[3]-inset}};
  }
  double clearance(double x,double y) const {
    if(!enabled) return std::numeric_limits<double>::infinity();
    if(!std::isfinite(x) || !std::isfinite(y)) return -std::numeric_limits<double>::infinity();
    const auto b=centers();return std::min({x-b[0],b[1]-x,y-b[2],b[3]-y});
  }
  // Sample a constant-acceleration/constant-yaw-rate MPC interval with the
  // same Simpson interpolation as the executor. A conservative chord-error
  // allowance covers the curve between the <=10ms samples.
  bool interval_safe(double x,double y,double yaw,double v,double a,double w,double duration) const {
    if(!enabled) return true;
    const int count=std::max(1,static_cast<int>(std::ceil(duration/.01)));
    const double dt=duration/count;
    if(clearance(x,y)<-1e-7) return false;
    for(int k=0;k<count;++k) {
      const double vm=v+.5*a*dt,ve=v+a*dt;
      const double nx=x+dt/6.*(v*std::cos(yaw)+4.*vm*std::cos(yaw+.5*w*dt)+ve*std::cos(yaw+w*dt));
      const double ny=y+dt/6.*(v*std::sin(yaw)+4.*vm*std::sin(yaw+.5*w*dt)+ve*std::sin(yaw+w*dt));
      const double bend=(std::abs(a)+std::max(std::abs(v),std::abs(ve))*std::abs(w))*dt*dt/8.;
      if(std::min(clearance(x,y),clearance(nx,ny))+1e-7<bend) return false;
      x=nx;y=ny;yaw+=w*dt;v=ve;
    }
    return true;
  }
};
}
