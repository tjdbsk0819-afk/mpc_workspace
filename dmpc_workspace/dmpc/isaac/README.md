# Two AntBots with local object prediction

For the prepared A → B → C → D workflow, use the
[scenario test guide](../docs/SCENARIO_TEST_GUIDE.md). `bash run_isaac.sh test A`
runs the driving launch in the foreground and records Markdown results;
all test results are appended only to `docs/SCENARIO_TEST_RESULTS.md`, without
per-run folders, JSON, telemetry, or log files.
`bash run_isaac.sh viewer --scenario A` starts the local path viewer on port 8092.
See [planner differences](../docs/MPC_PLANNER_COMPARISON.md) and
[test results](../docs/SCENARIO_TEST_RESULTS.md).

This workspace receives **observations**, not another robot's MPC solution.
Each robot has its own IPOPT instance. The two-robot execution structure follows
`mpc_planner_workspace/isaac_mpc/two`:

```text
Isaac /robot_b/odom (world pose, body-frame twist)
  -> robot_a/peer_state_relay (rotate velocity into world axes)
  -> /robot_a/peer_state {robot_id, original stamp, x, y, vx, vy, valid}
  -> robot_a/dmpc_planner
       local CV / CV+Gaussian moving-object forecast
       + own odometry + own goal
       -> CasADi / IPOPT optimization of only robot_a's motion
  -> /robot_a/mpc_plan (timestamped poses, speed, acceleration, yaw rate)
  -> native Nav2 RPP -> /robot_a/cmd_vel
  -> Swerve / ros2_control -> /robot_a/joint_command -> Isaac
```

Robot B runs the same pipeline with the identities reversed. Neither planner
subscribes to the other robot's goal, controls, `mpc_plan`, or path. Forecast paths
are published for visualization only. The peer's `vx,vy` are in **world axes**;
raw `Odometry.twist` is in `child_frame_id` and must be rotated first.

## Host commands

Run from `/home/ubuntu/tjdbsk/dmpc_workspace/dmpc`. Docker must be usable by the
current shell. The host needs an NVIDIA runtime/GPU and the Isaac Lab image
configured in [`.env`](.env). The copied image setting reuses the already
configured Isaac Lab 3.0.0b2.post1 / Isaac Sim 6.0.1 image; it does not use an
mpc_planner ROS image or acados.

The parent `dmpc_workspace/run_isaac.sh` forwards the same commands to this
repository, so they also work directly from `dmpc_workspace`.

```bash
# Validate Compose and the two-robot scene configuration without starting Isaac.
bash run_isaac.sh check

# Build the IPOPT ROS image, custom messages, Swerve, bridge, and RPP.
bash run_isaac.sh build

# One shared Zenoh router is needed on localhost:7447.
# Skip this command when a compatible router is already running there.
bash run_isaac.sh router
```

Use two terminals, with the same scenario in each:

```bash
# Terminal 1: start two physical articulations and their ROS I/O.
bash run_isaac.sh sim --scenario B

# Terminal 2: state relays, two IPOPT planners, RPP, and Swerve controllers.
bash run_isaac.sh drive scenario:=B
```

For physics without a viewer:

```bash
bash run_isaac.sh sim --scenario B --headless --duration 30
```

The default ROS domain is **31**, separate from mpc_planner's default 30.
Both the ROS build container and the Isaac container mount the repository at
`/workspace`, with ROS sources/builds under `/workspace/isaac`. This identical
layout is required because `colcon --symlink-install` generates absolute links.
Before starting Kit, the runner checks the generated message files, Python
imports, and native type support (including `JointCommandFeedback`).
Viewer: `http://118.216.110.114:8212`; signal TCP 49102, stream UDP 48000.
Port 8210 belongs to the separate original Isaac viewer. `ISAACSIM_HOST` in
`.env` must be the server address reachable from the browser PC; loopback only
works when the browser runs on the Isaac host. After changing this address,
rebuild the viewer and restart the simulation, since the viewer embeds its
connection settings at build time:

```bash
docker compose --env-file isaac/.env --project-directory isaac \
  -f isaac/docker/compose.isaac.yaml up -d --no-deps --build web-viewer
# Stop the running sim with Ctrl-C, then start it again.
bash run_isaac.sh sim --scenario B
```

Separate Docker cache volumes are used. The viewer image is built by Compose
when first needed. If the Isaac Lab image is absent, build it explicitly:

```bash
docker compose --env-file isaac/.env --project-directory isaac \
  -f isaac/docker/compose.isaac.yaml --profile lab build isaac-lab
```

Stop the foreground `drive` and `sim` processes with Ctrl-C. If this workspace
started the router, stop it with:

```bash
docker compose --env-file isaac/.env --project-directory . \
  -f isaac/docker/compose.dmpc.yaml stop router
```

## Configuration and observation contract

Planner settings are in
[`controller_bringup/config/dmpc.yaml`](controller_bringup/config/dmpc.yaml).
The default horizon is 30 intervals at 0.2 s (6 s), with planning at 10 Hz.
Both robots use a maximum forward speed of 1.0 m/s and acceleration of
0.5 m/s². The driving launch passes these planner limits to the native RPP
executor; turning, avoidance, and goal approach can reduce the actual speed.
Restart the driving launch after changing these limits; an Isaac restart and
ROS rebuild are not required for this configuration change.
The default model is `cv_gaussian`: the mean is `p0 + v*(t - measurement_stamp)`,
and the uncertainty padding matches mpc_planner/two:

```text
sigma(t) = noise * sqrt(prediction_dt * (t - measurement_stamp))
padding(t) = sqrt(-2*log(risk)) * sigma(t)
required_distance = 1.15 m + tracking_margin + padding(t)
```

Set `peer_prediction_model: cv` for deterministic constant velocity. Keep
`peer_prediction_dt` equal to `dt`; the planner, RPP, and prediction must agree.
All knot timestamps are `plan_stamp + k*dt`, including k=0. Measurements are
propagated to the planning epoch before predicting. A new solve is based on a
single snapshot, so callback timing cannot mix states inside one optimization.

The solver uses a second-order planar unicycle with state `[x,y,yaw,v]` and
input `[acceleration,yaw_rate]`. The native RPP receives N+1 poses and matching
N+1 speed/acceleration/yaw-rate entries, with zero terminal controls. Its
inter-knot integration matches the planner's Simpson/RK4 step. Object avoidance
is constrained at four substeps per interval, then checked independently at 20
substeps before publication. Workspace walls and static circles are included
for scenarios C and D. A right-passing seed breaks symmetric head-on solutions;
it uses the observed object forecast, not the object's goal.

Only fresh, finite, correctly framed states are accepted. Replayed timestamps
do not refresh the receive watchdog. Simulation-clock resets clear observations
and invalidate in-flight results. IPOPT runs on a worker thread, with its own
time/iteration limits; stale states, rejected output, and expired plans cause
RPP to stop. Neither IPOPT nor the CV forecast provides a hard real-time or
arbitrary-motion collision guarantee; RPP retains the reference command and
braking checks.

Scenarios: A is head-on, B is crossing (default), C is a narrow corridor, D adds
a static cylinder. C has insufficient width for the two configured footprints
to pass side by side, so stopping/deadlock is expected without a separate
right-of-way or retreat policy. Goal overrides are available through
`robot_a_goal_x:=...`, `robot_a_goal_y:=...`, `robot_a_goal_yaw:=...` and the
corresponding robot B arguments; spawn overrides are Isaac runner arguments.

## Original simulation functions

`src/decentralized.py`, `decentralized_leader.py`, and
`decentralized_rendezvous.py` also use local forecasts from the current model
state. `pred_X` and `pred_U` are now only each agent's own warm start. Their
default prediction is deterministic CV, including for the 3D drone and double
integrator examples. The Gaussian radius above is planar and is supported only
for the bicycle examples and the Isaac AntBot adapter.

The old `mode` argument is accepted for compatibility. Since both modes use the
same observation snapshot, they no longer implement different inter-agent
solution exchange schedules. `distributed*.py` remains the centralized joint
optimization baseline, where every agent's trajectory is a decision variable.
Custom dynamics need a position/velocity adapter in
`dmpc_runtime.prediction.observation_from_model_state`.

## Verification

The ROS-free suite uses real CasADi/IPOPT. The C++ contract test compiles the
vendored native RPP sampler and compares its output against optimized motion
profiles:

```bash
python -m pip install -e .
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m unittest discover -s tests -v
python -m dmpc_runtime.simulate --scenario B --duration 30 \
  --solve-seconds 0.08
```

Numerical runs apply the optimized input to a mathematical unicycle. They test
prediction and local optimization, and do **not** replace Isaac/RPP/Swerve
physics validation. Check ROS state and outputs after starting the stack:

```bash
bash run_isaac.sh shell
ros2 topic echo /robot_a/peer_state --once
ros2 topic echo /robot_a/mpc_status
ros2 topic echo /robot_b/mpc_status
ros2 topic echo /robot_a/control_status
```

The 2026-10-28 preparation passed 22 automated tests and actual Isaac headless
startup smoke checks for A/B/C/D. These short scene checks do not establish
closed-loop driving success. Full sequential driving runs and their recorded
metrics belong in [the test results](../docs/SCENARIO_TEST_RESULTS.md).

## Reference snapshot

`simulation`, `simulation_runtime`, AntBot USD assets, message definitions,
Swerve, Isaac bridge, and native RPP are copied from
`mpc_planner_workspace/isaac_mpc/two`; they no longer need that checkout at
runtime. [`vendor/manifest.json`](vendor/manifest.json) records original hashes;
local namespace, launch, Docker, and dependency adaptations differ from those
original hashes. The reference cache/frame-conversion helper was copied to
`dmpc_runtime/observations.py`. Preserve [`vendor/ANTBOT_LICENSE`](vendor/ANTBOT_LICENSE)
and the repository's original [MIT license](../LICENSE).
