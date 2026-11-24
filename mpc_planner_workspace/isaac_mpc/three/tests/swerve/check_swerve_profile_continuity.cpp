// Regression tests against the actual Swerve motion planner, without Isaac.
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include "antbot_swerve_controller/swerve_motion_control.hpp"
using namespace antbot::swerve_drive_controller;

void require(bool ok, const char *message) {if (!ok) throw std::runtime_error(message);}
void near(double a, double b, double tol=1e-8) {require(std::abs(a-b)<tol,"state continuity violated");}

SwerveMotionControl planner()
{
  SwerveMotionControl p;
  p.configure(4,.103,{{.265,.256},{.265,-.256},{-.265,.256},{-.265,-.256}},
    {.0555,-.0555,.0555,-.0555},{0.,0.,0.,0.},
    {"s0","s1","s2","s3","w0","w1","w2","w3"},5.,10.,18.7254,
    {-.959,-.959,-.959,-.959},{.959,.959,.959,.959},0,.087266,1.,.01);
  return p;
}

int main()
{
  // Moving towards, away from, past, or exactly at the new target.
  std::mt19937 rng(41);
  std::uniform_real_distribution<double> position(-.6,.6), velocity(-2.,2.);
  for (int i=0;i<600;++i) {
    const double start=position(rng), target=position(rng), v=velocity(rng);
    TrapezoidalProfile p(start,target,5.,10.,0.,v);
    require(p.isValid(),"moving-start profile invalid");
    near(p.getPositionAt(0.),start); near(p.getVelocityAt(0.),v);
    near(p.getPositionAt(p.total_time+1.),target); near(p.getVelocityAt(p.total_time+1.),0.);
    const double dt=p.total_time/2000.;
    double previous=p.getVelocityAt(0.);
    for(int k=1;k<=2000;++k) {
      const double t=k*dt, next=p.getVelocityAt(t);
      require(std::abs(next)<=5.+1e-7,"steering speed limit exceeded");
      require(std::abs(next-previous)<=10.*dt+1e-7,"steering acceleration limit exceeded");
      require(std::abs(p.getAccelerationAt(t))<=10.+1e-8,"analytical acceleration exceeded");
      previous=next;
    }
    TrapezoidalProfile held(start,target,5.,10.,p.total_time+1.,v);
    near(held.getVelocityAt(0.),v);
    near(held.getPositionAt(p.total_time+.5),target);
    near(held.getVelocityAt(p.total_time+.5),0.);
  }
  std::cout<<"PASS: 600 moving-start/reversal profiles preserve initial state and velocity/acceleration bounds\n";
  require(!TrapezoidalProfile(.95,.95,5.,10.,0.,2.).staysWithin(-.959,.959),
          "unsafe braking overshoot accepted");
  require(TrapezoidalProfile(.5,.8,5.,10.,0.,.5).staysWithin(-.959,.959),"safe braking rejected");
  require(!TrapezoidalProfile(0.,.1,5.,10.,0.,6.).isValid(),"overspeed state accepted");
  std::cout<<"PASS: joint limits include braking overshoot; invalid initial speed rejected\n";

  auto p=planner();
  std::vector<double> q(4,0.),wheels(4,0.);
  geometry_msgs::msg::Twist cmd;cmd.linear.x=.2;cmd.angular.z=.1;
  auto motion=p.plan(cmd,q,wheels);
  require(!motion.trajectory.points.empty(),"no initial profile");
  MotionState state;
  double maximum_wheel=0., maximum_angle=0.;
  for(int k=0;k<250;++k) {
    state=p.sample(.02);
    require(state.valid,"missing analytical state");
    cmd.linear.x=.2+.005*std::sin(k*.7); // retarget at every 50Hz update
    motion=p.plan(cmd,state.steering_positions,state.wheel_velocities,&state);
    require(!motion.trajectory.points.empty(),"retarget failed");
    auto start=p.sample(0.);auto next=p.sample(.02);
    for(int j=0;j<4;++j) {
      near(start.steering_positions[j],state.steering_positions[j]);
      near(start.steering_velocities[j],state.steering_velocities[j]);
      near(start.wheel_velocities[j],state.wheel_velocities[j]);
      require(std::abs(next.steering_velocities[j]-state.steering_velocities[j])<=.2+1e-7,
              "retarget resets steering velocity");
      require(std::abs(next.wheel_velocities[j]-state.wheel_velocities[j])<=18.7254*.02+1e-7,
              "retarget breaks wheel acceleration bound");
      maximum_wheel=std::max(maximum_wheel,std::abs(next.wheel_velocities[j]));
      maximum_angle=std::max(maximum_angle,std::abs(next.steering_positions[j]));
    }
  }
  require(maximum_wheel>1.,"frequent retarget starves wheel acceleration");
  require(maximum_angle>.1,"frequent retarget starves steering motion");
  std::cout<<"PASS: 250 consecutive 50Hz retargets advance steering and drive without discontinuities\n";
  state=p.sample(.02);
  auto lagged=state.steering_positions;for(auto &q:lagged)q-=.01;
  motion=p.plan(cmd,lagged,std::vector<double>(4,0.),&state);
  auto continued=p.sample(0.);
  for(int j=0;j<4;++j) near(continued.steering_positions[j],state.steering_positions[j]);
  std::cout<<"PASS: warm retarget carries the active reference instead of rewinding to delayed feedback\n";
  p.reset();require(!p.sample(.02).valid,"reset retained old profile");
  motion=p.plan(cmd,std::vector<double>(4,0.),std::vector<double>(4,0.));
  auto cold=p.sample(0.);
  for(int j=0;j<4;++j) {near(cold.steering_positions[j],0.);near(cold.steering_velocities[j],0.);near(cold.wheel_velocities[j],0.);}
  std::cout<<"PASS: reset discards continuation and starts from new measured state\n";

  p.reset();
  q={-.802671,.802671,.802671,-.802671};wheels.assign(4,0.);
  double smallest_spin_drive=1.;
  for (int k=0;k<150;++k) {
    cmd=geometry_msgs::msg::Twist();
    cmd.angular.z=-(.006+.008*(.5+.5*std::sin(k*.7)));
    require(p.determine_target_mode(cmd)==TargetDrivingType::SPINNING,
      "small nonzero spin was classified as stopped");
    motion=p.plan(cmd,q,wheels);
    require(!motion.trajectory.points.empty(),"small spin generated no profile");
    state=p.sample(.02);q=state.steering_positions;wheels=state.wheel_velocities;
    smallest_spin_drive=std::min(smallest_spin_drive,std::abs(wheels[0]));
  }
  require(smallest_spin_drive>.01,"aligned small spin was starved by mode confirmation");
  cmd=geometry_msgs::msg::Twist();
  require(p.determine_target_mode(cmd)==TargetDrivingType::STOPPED,"explicit zero did not stop");
  std::cout<<"PASS: 150 spins across the old 0.01 threshold remain executable; explicit zero still stops\n";
}
