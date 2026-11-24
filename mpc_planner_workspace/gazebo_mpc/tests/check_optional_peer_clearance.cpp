#include "mpc_tracking.hpp"
#include <cassert>
#include <cmath>
int main() {
  using namespace antbot_tracking;
  Motion ego{0.,0.,0.,0.,0.};
  bool rejected=false;
  try { filter_command(ego,.5,0.,{},.25,.7,.5,.5,0.,0.); }
  catch(const std::invalid_argument &) {rejected=true;}
  assert(rejected);
  auto forward=filter_command(ego,.5,0.,{},.25,.7,.5,.5,0.,0.,{},true);
  assert(forward && forward->v==.5 && std::isfinite(forward->clearance));
  antbot_workspace::Bounds bounds;
  bounds.configure(true,{-1.,1.,-1.,1.},.1,.05);
  ego.x=1.2;
  assert(!filter_command(ego,.5,0.,{},.25,.7,.5,.5,0.,0.,bounds,true));
  ego.x=0.;Neighbor obstacle{.2,0.,0.,0.,1.};
  assert(!filter_command(ego,.5,0.,{obstacle},.25,.7,.5,.5,0.,0.,{},true));
}
