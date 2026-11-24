#!/usr/bin/env bash
set -eo pipefail
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f /mpc_workspace/install/setup.bash ]]; then
    source /mpc_workspace/install/setup.bash
fi
exec "$@"
