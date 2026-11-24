#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
task_root="$repo_root/isaac"
export PYTHONPATH="$repo_root:$task_root${PYTHONPATH:+:$PYTHONPATH}"
bash "$task_root/docker/build-ros-messages.sh"
source "$task_root/message_build/install/setup.bash"
mkdir -p "$task_root/control_build"
cd "$task_root/control_build"
python /usr/bin/colcon build \
    --base-paths "$task_root/swerve_controller" "$task_root/isaac_control_bridge" \
                 "$task_root/controller_bringup" "$task_root/path_following_controller" \
    --packages-select antbot_swerve_controller antbot_isaac_control antbot_dmpc_ros antbot_navigation \
    --symlink-install --cmake-args -DBUILD_TESTING=OFF
