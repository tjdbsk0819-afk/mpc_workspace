# Atomic original MPC output for the RPP adapter. Times use the Isaac clock.
std_msgs/Header header
nav_msgs/Path path
# Legacy first-step summary; execution uses the timestamped profiles below.
float64 target_speed
float64 target_yaw_rate
# Entry k corresponds to path.poses[k].header.stamp. The unicycle uses
# x'=v*cos(yaw), y'=v*sin(yaw), v'=a, yaw'=w; controls are held per MPC interval.
float64[] linear_velocities
float64[] linear_accelerations
float64[] angular_velocities
float64 max_acceleration
float64 max_yaw_rate
float64 minimum_separation
bool valid
string status
