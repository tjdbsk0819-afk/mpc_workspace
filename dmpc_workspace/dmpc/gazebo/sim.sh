#!/usr/bin/env bash
set -euo pipefail
scenario=${1:?scenario};robots=${2:?robot count};shift 2
output=/workspace/gazebo/generated
python3 /workspace/gazebo/generate.py --reference "$ANTBOT_REFERENCE" --output "$output" --scenario "$scenario" --robots "$robots"

sim_pid=''
bridge_pid=''
signal_groups() {
    local signal=$1 pid
    for pid in "$sim_pid" "$bridge_pid"; do
        [[ -n "$pid" ]] || continue
        kill -"$signal" -- "-$pid" 2>/dev/null || true
    done
}
groups_alive() {
    local pid
    for pid in "$sim_pid" "$bridge_pid"; do
        if [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null; then return 0; fi
    done
    return 1
}
wait_groups() {
    local attempts=$1 i
    for ((i=0;i<attempts;i++)); do
        groups_alive || return 0
        sleep .1
    done
    ! groups_alive
}
cleanup() {
    local result=$? pid
    # Repeated Ctrl+C must not re-enter cleanup or interrupt child reaping.
    trap - EXIT
    trap '' INT TERM
    echo 'Stopping Gazebo server and clock bridge...' >&2
    signal_groups INT
    if ! wait_groups 30; then
        echo 'Gazebo shutdown: escalating to SIGTERM' >&2
        signal_groups TERM
        if ! wait_groups 20; then
            echo 'Gazebo shutdown: escalating to SIGKILL' >&2
            signal_groups KILL
            wait_groups 10 || true
        fi
    fi
    # Reap finished direct children without waiting forever for a stuck child.
    for pid in "$sim_pid" "$bridge_pid"; do
        if [[ -n "$pid" ]] && ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null || true
        fi
    done
    exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Own each entire process tree (ros2 can spawn parameter_bridge). Async shell
# jobs may inherit SIGINT=ignored; restore default signals before exec.
setsid env --default-signal=INT,TERM ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' &
bridge_pid=$!
# Server-only rendering and all scenario/controller settings are unchanged.
setsid env --default-signal=INT,TERM gz sim -r -s "$output/world.sdf" "$@" &
sim_pid=$!
wait "$sim_pid"
