# Ground-truth planar state of the observed robot, in header.frame_id.
# header.stamp is the original odometry sample time, not publication time.
std_msgs/Header header
string robot_id
float64 x
float64 y
float64 vx
float64 vy
# Consumers must check valid before using the four state values.
bool valid
string status
