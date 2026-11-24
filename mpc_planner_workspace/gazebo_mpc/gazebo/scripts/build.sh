#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
if [[ "$ANTBOT_PLANNER" == mpc ]]; then
    bash "$ANTBOT_REFERENCE/docker/build-driving-stack.sh"
else
    bash /workspace/isaac/docker/build-dmpc.sh
fi
source "$ANTBOT_REFERENCE/message_build/install/setup.bash"
cmake -S /workspace/gazebo/plugin -B /workspace/gazebo/build/plugin -DCMAKE_BUILD_TYPE=Release
cmake --build /workspace/gazebo/build/plugin --parallel 2
