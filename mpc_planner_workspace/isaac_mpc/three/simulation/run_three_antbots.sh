#!/usr/bin/env bash
# Run inside the Isaac Lab container; the MPC container has no Isaac runtime.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
runner_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
reference_root=${ANTBOT_ISAAC_WORKSPACE:-$runner_root}
set +u
source "$reference_root/message_build/install/setup.bash"
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONUNBUFFERED=1
export PYTHONFAULTHANDLER=1
export ROS_LOG_DIR=/tmp/antbot_isaac_ros_logs
export LD_LIBRARY_PATH="/opt/ros/jazzy/opt/zenoh_cpp_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib:$reference_root/message_build/install/gaemi_interfaces/lib:/isaac-sim/exts/isaacsim.ros2.core/jazzy/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$runner_root:$reference_root${PYTHONPATH:+:$PYTHONPATH}"
for cache_dir in /isaac-sim/.cache/ov/texturecache /isaac-sim/.nv/ComputeCache; do
    if ! cache_probe=$(mktemp "$cache_dir/.antbot-write-check.XXXXXX"); then
        echo "Isaac cache is not writable: $cache_dir (user $(id -u):$(id -g))." >&2
        echo "Start from the host with: sudo bash docker/run-three-antbots.sh" >&2
        exit 1
    fi
    rm -- "$cache_probe"
done
exec /isaac-sim/python.sh "$runner_root/simulation/run_three_antbots.py" --project-root "$reference_root" "$@"
