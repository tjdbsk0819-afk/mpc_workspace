#!/usr/bin/env bash
# Build the camera interfaces and GT peer-state interface in one underlay.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
# rosidl's incremental Python type-support library can retain symbols for a
# deleted message. Remove only this package's generated artifacts on migration.
if [[ ! -f "$task_root/ros_messages/antbot_interfaces/msg/RecoveryState.msg" &&
      -f "$task_root/message_build/install/antbot_interfaces/lib/python3.12/site-packages/antbot_interfaces/msg/_recovery_state.py" ]]; then
    rm -rf -- "$task_root/message_build/build/antbot_interfaces" \
               "$task_root/message_build/install/antbot_interfaces"
fi
mkdir -p "$task_root/message_build"
cd "$task_root/message_build"
python /usr/bin/colcon build \
    --base-paths "$task_root/ros_messages" \
    --packages-select gaemi_interfaces antbot_interfaces \
    --symlink-install --cmake-args -DBUILD_TESTING=OFF
