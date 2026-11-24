#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
if [[ -f /workspace/message_build/install/setup.bash ]]; then
    source /workspace/message_build/install/setup.bash
fi
if [[ -n "${ZENOH_SESSION_CONFIG_URI:-}" && ! -r "$ZENOH_SESSION_CONFIG_URI" ]]; then
    echo "Cannot read Zenoh session config: $ZENOH_SESSION_CONFIG_URI" >&2
    exit 1
fi
if [[ -f /workspace/control_build/install/setup.bash ]]; then
    source /workspace/control_build/install/setup.bash
fi
if [[ -f /workspace/planner_build/install/setup.bash ]]; then
    source /workspace/planner_build/install/setup.bash
fi
exec "$@"
