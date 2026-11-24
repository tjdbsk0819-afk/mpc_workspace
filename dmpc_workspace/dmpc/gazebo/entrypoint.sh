#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
for prefix in message_build control_build planner_build; do
    if [[ -f "$ANTBOT_REFERENCE/$prefix/install/setup.bash" ]]; then
        source "$ANTBOT_REFERENCE/$prefix/install/setup.bash"
    fi
done
export PYTHONPATH="/workspace:$ANTBOT_REFERENCE:/workspace/gazebo${PYTHONPATH:+:$PYTHONPATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/workspace/gazebo/build/plugin${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
exec "$@"
