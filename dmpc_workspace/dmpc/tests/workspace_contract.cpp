// Check real IPOPT output with the same native wall guard used by RPP.
#include <antbot_interfaces/workspace_bounds.hpp>
#include <iostream>

int main() {
  int count, expected; double dt;
  if(!(std::cin >> count >> dt >> expected) || count<2) return 2;
  antbot_workspace::Bounds bounds;
  bounds.configure(true, {-4.,4.,-.7,.7}, .5, .05);
  bool accepted=true;
  for(int k=0;k<count;++k) {
    double x,y,yaw,v,a,w;
    if(!(std::cin >> x >> y >> yaw >> v >> a >> w)) return 3;
    if(bounds.clearance(x,y)<-1e-7 ||
       (k+1<count && !bounds.interval_safe(x,y,yaw,v,a,w,dt))) accepted=false;
  }
  return accepted==static_cast<bool>(expected)?0:1;
}
