# Single-robot MPC timing — run until user stop

All four robots start together. The selected ego runs the existing MPC; three
followers track recorded trajectories after removing their initial departure waits.
Followers prohibit body-frame reverse motion, including backward position
corrections, with the existing speed and acceleration limits.
Both workspaces use byte-identical `gazebo/references/{A,B,D}.json`.

```bash
# MPC planner
cd /home/tjdbsk/workspaces/mpc_planner_workspace
bash run_gazebo.sh benchmark --scenario A --ego robot_a

# DMPC — run separately to avoid another MPC competing for CPU
cd /home/tjdbsk/workspaces/dmpc_workspace
bash run_gazebo.sh benchmark --scenario A --ego robot_a
```

`benchmark` owns its simulator. Stop any normal sim/drive first. No rebuild is
required. Change `--scenario` to B or D, or `--ego` to robot_b/c/d. MPC planner uses 10 steps × 0.2 s = 2 seconds; DMPC remains at 30 steps × 0.2 s = 6 seconds. The B source recording
was incomplete; this is retained in the reference file's `source_result`.

There is no automatic experiment stop based on distance, tracking error,
odometry staleness, startup elapsed time, graph counts, goals, or recording end.
The former `--timeout` argument is accepted for compatibility but has no effect.
On reference completion, followers keep their final reference position; the
simulator and timing collector remain running. Existing MPC algorithms,
constraints, execution checks and solver settings are unchanged. If the MPC
stops optimizing at its goal, the collector waits rather than creating artificial
zero-duration timing samples. Computation failure samples remain included.

The terminal prints solver/planning current, mean and max times every two seconds.
Only optimization results completed after common release and no later than robot_a arrival are counted. `solver_ms`
is the sum of solver calls in a replan. `planning_ms` keeps the implementation's
existing wall-clock scope. OS, simulator, GUI and follower load remain relevant.
The three reference followers solve no optimization and never publish fake odometry;
MPC continues to observe actual peer odometry, not their known future reference.

All runs are accumulated in one result file per planner, prediction horizon and
scenario, for example `gazebo/results/mpc_planner_예측2초_시나리오A.json`
(or `dmpc_예측6초_시나리오A.json`). Each entry in `runs` contains only its solver/planning timing statistics,
sample count, and individual stamp/solver_ms/planning_ms samples. Existing runs are preserved.
Only successful-arrival runs are saved. Arrival means robot_a is within 0.2 m
of its scenario goal with speed at most 0.05 m/s for 0.5 seconds of fresh odometry;
final yaw alignment is not required. Timing samples are collected in memory
until arrival, then the entire pre-arrival timing record and mean/max/median/p95
statistics are appended once. Further timing samples are ignored; the simulation
continues until user stop. If stopped before robot_a arrives, that run is not
saved. This policy applies to A, B and D scenarios in both workspaces. No drive.log, separate timing.jsonl, summary.json, or
per-run output directory is created. Planner child-process logs are discarded.

Stop with Ctrl+C in the benchmark terminal. The arrival snapshot is preserved;
an incomplete run is discarded. Alternatively, in the same workspace:

```bash
bash run_gazebo.sh stop
```

GUI, view, timing and shell automatically join the running benchmark ROS domain.
Launch the GUI in another terminal; close any existing GUI first:

```bash
bash run_gazebo.sh gui --scenario A --robots 4
```

The ego displays its computed MPC path. Followers display their remaining fixed
reference route (at most 200 poses) through visualization-only `mpc_path` topics.
Stop/restart a pre-existing web viewer when changing benchmark domain.

Pure reference/command checks:
`python3 gazebo/single_mpc/test_reference.py`.
The original bags are never modified. To refresh a reference inside the ROS container:

```bash
python3 /workspace/gazebo/single_mpc/export_reference.py \
  --bag /workspace/gazebo/results/mpc_robot4_예측6초_시나리오A \
  --scenario A --output /workspace/gazebo/references/A.json
```

Copy the JSON to the other workspace's `gazebo/references/A.json` to keep the
reference identical. Simultaneous departure changes encounters from the original
staggered recording.

## B robot_a peer clearance

MPC planner scenario B robot_a uses peer safety margin 0.00 m (nominal center
separation 1.00 m), consistently in planner constraints/current-clearance checks
and MPC-RPP minimum separation. All other scenarios/robots keep 0.15 m planning margin
and 1.15 m nominal separation in the MPC constraints. Prediction uncertainty, tracking margin and
other checks still apply, so 1.00 m is not a promise that every command is allowed
at that distance. DMPC settings are unchanged.

## B robot_a proximity-stop experiment

Every MPC planner robot in all Gazebo scenarios sets `peer_proximity_stop_enabled=false`
in both its planner and executor. The planner does not short-circuit solely
because current peer distance is below the nominal minimum; the executor omits
peer-neighbor distance filtering. Static-obstacle/workspace checks and the MPC's
predicted collision constraints/plan validation remain. A failed/invalid or
expired plan can still command a stop. This applies to A/B/C/D and both single-MPC and normal drive modes.
DMPC and Isaac launch files are unchanged.

## B simple natural routes

All four B references now use smooth start-to-goal progress over 20 seconds,
with a small right-side bend of at most 1 m at the central crossing. Endpoints
are approached with zero terminal speed and correct heading, without reversing,
passing the goal or looping back. Timing starts simultaneously. A selected as
ego still chooses its own MPC path; the fixed followers execute these routes.
`B_recorded.json` and the original bags remain preserved. Regenerate with
`python3 gazebo/single_mpc/generate_b_routes.py`.

## ROS initialization reliability

The MPC runtime uses the UDP-only Fast DDS profile in `fastdds_udp.xml` to avoid
shared-memory initialization stalls after repeated container restarts. This
applies to the simulator, planners, reference follower and GUI containers.
Launch errors are printed on the measurement terminal without writing log files.
Prediction horizon is 10 × 0.2 = 2 seconds.

## A purple/green route separation

Green robot_c is unchanged. Only purple robot_d uses a lower-side smooth route
(peak Y=-1.6 m), opposite the upper green route. Its recorded timing and endpoints
are retained, with monotonically forward progress. The simultaneous reference
center distance between C/D is at least about 2.65 m. Original A trajectories
remain in A_recorded.json. Both workspaces use the same adjusted reference.

## Actual planning Hz

Both planners publish monotonic start/completion timestamps for optimization
cycles. Benchmark and timing monitor display configured Hz, actual start Hz,
completion Hz, and valid-plan publication Hz. Average Hz is (N-1)/elapsed event
time, not 1000 divided by mean planning time. The monitor uses its latest 1000
samples; the benchmark uses samples from release to arrival. Initial/legacy
samples without enough event timestamps show -- rather than an invented rate.
`summary.planning_hz` and raw start_wall_sec/completion_wall_sec/accepted are
saved alongside solver/planning times. Existing result files are not changed.
Arrival-only saving remains enabled.
