#!/usr/bin/env bash
set -eo pipefail
source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd /mpc_workspace

python /opt/mpc-docker/prepare-workspace.py
python src/mpc_planner/mpc_planner_jackal/scripts/generate_jackal_solver.py
cp /opt/mpc-docker/solver.CMakeLists.txt src/mpc_planner/mpc_planner_solver/CMakeLists.txt

# Optional simulators are absent from this checkout; build the Jackal planner and core.
colcon build --symlink-install --parallel-workers "${BUILD_JOBS:-4}" \
    --packages-ignore antbot_mpc_sim mpc_planner_jackalsimulator mpc_planner_rosnavigation mpc_planner_dingo \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DPython3_EXECUTABLE=/usr/bin/python3
