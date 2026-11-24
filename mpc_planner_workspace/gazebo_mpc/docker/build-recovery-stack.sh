#!/usr/bin/env bash
# Rebuild the MPC initial-guess API and wrapper/controllers; reuse the generated solver.
set -eo pipefail
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -f "$task_root/planner_build/install/setup.bash" ]]; then
    exec bash "$task_root/docker/build-driving-stack.sh"
fi
bash "$task_root/docker/build-antbot-controllers.sh"
source "$task_root/planner_build/install/setup.bash"
cd "$task_root/planner_build"
python /usr/bin/colcon build \
    --base-paths "$task_root/mpc_planning/mpc_planner" "$task_root/mpc_planning/mpc_planner_antbot" \
    --packages-select mpc_planner mpc_planner_antbot --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
