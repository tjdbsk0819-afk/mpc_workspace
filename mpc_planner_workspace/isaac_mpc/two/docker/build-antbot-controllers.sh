#!/usr/bin/env bash
# Build Swerve, the Isaac bridge, controller bringup, and RPP launch/config.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$task_root${PYTHONPATH:+:$PYTHONPATH}"
bash "$task_root/docker/build-ros-messages.sh"
source "$task_root/message_build/install/setup.bash"
set -u
for package in swerve_controller isaac_control_bridge controller_bringup path_following_controller; do
    if [[ ! -f "$task_root/$package/package.xml" ]]; then
        echo "Missing package source: $task_root/$package" >&2
        exit 1
    fi
done
mkdir -p "$task_root/control_build"
cd "$task_root/control_build"
python /usr/bin/colcon build \
    --base-paths "$task_root/swerve_controller" "$task_root/isaac_control_bridge" "$task_root/controller_bringup" "$task_root/path_following_controller" \
    --packages-select antbot_swerve_controller antbot_isaac_control antbot_mpc_ros antbot_navigation \
    --symlink-install --cmake-args -DBUILD_TESTING=OFF
