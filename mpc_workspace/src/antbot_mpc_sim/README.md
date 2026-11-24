# Antbot MPC simulation experiments

This package owns experiment launch files. Robot models, worlds and controllers
remain in the existing `antbot_gazebo`, `antbot_description` and
`antbot_swerve_controller` packages. `two_robots.launch.py` creates two isolated
robots in parallel lanes; it does not implement mutual obstacle avoidance.

Build after sourcing the Antbot workspace:

```bash
source /opt/ros/humble/setup.bash
source /antbot_ws/install/setup.bash
cd /mpc_workspace
colcon build --symlink-install --packages-select antbot_mpc_sim \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch antbot_mpc_sim two_robots.launch.py
```

The default `mpc-build` excludes this optional package because the standalone
MPC image does not include the Antbot dependencies. Build it explicitly in the
Antbot-enabled container. The path publisher remains in `mpc_workspace/sim`.

## Encounter path length

Launch opposing initial poses with `encounter:=true`. The test-path publisher
accepts `path_length` in metres (default 5.0); the complete encounter uses 8 m:

```bash
python3 /mpc_workspace/sim/publish_test_path.py --ros-args \
  -r __ns:=/robot_a -p use_sim_time:=true -p path_length:=8.0
```

Run the same publisher for `robot_b` when starting nodes individually. Both
MPCs use `velocity_weight=1.0`, `reference_velocity=0.15` and the encounter
peer transforms (`peer_offset_x=6.0`, `peer_offset_y=0.0`, `peer_yaw=pi`).

## Exchange of MPC plans

After building `mpc_planner_jackal` and `antbot_mpc_sim`, source both workspaces
and run a complete encounter with one launch command:

```bash
source /opt/ros/humble/setup.bash
source /antbot_ws/install/setup.bash
source /mpc_workspace/install/setup.bash
ros2 launch antbot_mpc_sim cooperative_encounter.launch.py
```

This starts the simulator, both 8 m reference paths and both MPC nodes. Do not
also start the old separate path/MPC commands. The default is 0.15 m/s reference
velocity and velocity weight 1.0. Both robots run MPC with identical settings;
there is no robot-specific stop or yielding rule. Set
`share_trajectories:=false` to select the previous constant-velocity peer mode.

Each node publishes `/robot_a/predicted_trajectory` or
`/robot_b/predicted_trajectory` as `nav_msgs/Path`. Positions and orientations
are in the sender's local `odom` frame. The header and first pose carry the
planning epoch; every pose has its own absolute ROS time, spaced by the MPC
integration interval. The first point is measured state and later points are
solver predictions. Both peers must use the same ROS time source.

The receiver interpolates at its own current time and horizon steps, transforms
into its local frame using the configured spawn transform, and supplies those
positions to the existing Guidance/Gaussian obstacle constraints. Existing
prediction uncertainty is retained. `/robot_a/peer_prediction` and
`/robot_b/peer_prediction` expose the actual resampled inputs; their header
stamp identifies the source plan, while pose stamps identify sampled times.

Arrived and solver-failure states publish stationary plans while own
odometry remains fresh. This prevents startup deadlock and keeps an arrived
robot visible to the other planner. In exchange mode, a missing/stale plan or
insufficient horizon causes a stop instead of a silent constant-velocity
fallback. `peer_trajectory_timeout` defaults to 0.5 s and is checked against
both ROS timestamps and steady receipt time. The final segment may be extended
by at most timeout + one integration step to accommodate transmission delay.
Malformed frames, quaternions, nonfinite positions and unordered times are
rejected. This is distributed MPC with shared plans and no fixed priority, not a
joint solver or a negotiation protocol; transforms are specific to these two
known spawn poses.

The publisher is installed as `ros2 run antbot_mpc_sim publish_test_path`.
The old `/mpc_workspace/sim/publish_test_path.py` remains a compatibility entry
point. `two_robots.launch.py` still starts simulation only; the new
`cooperative_encounter.launch.py` starts simulation and MPC together.
