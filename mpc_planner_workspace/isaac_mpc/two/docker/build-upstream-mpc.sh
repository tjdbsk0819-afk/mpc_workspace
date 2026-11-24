#!/usr/bin/env bash
set -eo pipefail
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
solver_python=${ANTBOT_SOLVER_PYTHON:-/opt/antbot-solver-venv/bin/python}
if [[ ! -x "$solver_python" ]]; then
    echo 'Use the antbot-mpc:jazzy image (docker/compose.mpc.yaml).' >&2
    exit 1
fi
python3 "$task_root/docker/prepare-upstream-mpc.py"
bash "$task_root/docker/build-ros-messages.sh"
source "$task_root/message_build/install/setup.bash"
"$solver_python" "$task_root/mpc_planning/mpc_planner_antbot/scripts/generate_antbot_solver.py"
mkdir -p "$task_root/planner_build"
cd "$task_root/planner_build"
python /usr/bin/colcon build --base-paths \
    "$task_root/planner_dependencies/ros_tools" \
    "$task_root/mpc_planning/mpc_planner_types" \
    "$task_root/mpc_planning/mpc_planner_util" \
    "$task_root/mpc_planning/mpc_planner_solver" \
    "$task_root/mpc_planning/mpc_planner_modules" \
    "$task_root/mpc_planning/mpc_planner" \
    "$task_root/mpc_planning/mpc_planner_antbot" \
    --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
