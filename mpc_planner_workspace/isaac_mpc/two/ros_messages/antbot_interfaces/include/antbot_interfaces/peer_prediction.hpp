#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace antbot_prediction {
// Random walk: p(t)=p(0)+W(t). There is NO velocity drift in its mean.
// q=noise^2*dt matches the old isotropic upstream variance at k*dt.
struct Model {
  std::string name{"cv"};
  double noise{.3},dt{.2},risk{.05};
  void validate() const {
    if(name!="cv" && name!="cv_gaussian" && name!="gaussian_random_walk")
      throw std::invalid_argument("Unknown peer prediction model: "+name);
    if(!std::isfinite(noise) || noise<=0. || !std::isfinite(dt) || dt<=0. ||
       !std::isfinite(risk) || risk<=0. || risk>=1.)
      throw std::invalid_argument("Invalid Gaussian peer prediction parameters");
  }
  bool gaussian() const {return name!="cv";}
  bool uses_velocity() const {return name!="gaussian_random_walk";}
  std::array<double,2> mean(double x,double y,double vx,double vy,double elapsed) const {
    return uses_velocity()?std::array<double,2>{x+vx*elapsed,y+vy*elapsed}:std::array<double,2>{x,y};
  }
  double sigma(double elapsed) const {
    return gaussian()?noise*std::sqrt(dt*std::max(0.,elapsed)):0.;
  }
  double radius(double elapsed) const {return std::sqrt(-2.*std::log(risk))*sigma(elapsed);}
};
}
