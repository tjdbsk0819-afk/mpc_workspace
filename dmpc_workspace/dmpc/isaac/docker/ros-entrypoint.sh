#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
for prefix in message_build control_build; do
    if [[ -f /workspace/isaac/$prefix/install/setup.bash ]]; then
        source "/workspace/isaac/$prefix/install/setup.bash"
    fi
done
export PYTHONPATH="/workspace:/workspace/isaac${PYTHONPATH:+:$PYTHONPATH}"
exec "$@"
