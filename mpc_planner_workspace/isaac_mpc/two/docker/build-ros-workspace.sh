#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$task_root${PYTHONPATH:+:$PYTHONPATH}"
bash "$task_root/docker/build-ros-messages.sh"
source "$task_root/message_build/install/setup.bash"
set -u
# Keep the installed executables on the ROS environment's Python interpreter.
python -c 'import rclpy; from antbot_interfaces.msg import PeerState'
mkdir -p "$task_root/control_build"
cd "$task_root/control_build"
python /usr/bin/colcon build --base-paths "$task_root/controller_bringup" --symlink-install \
    --packages-select antbot_mpc_ros
