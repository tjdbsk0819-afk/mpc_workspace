# Verification performed on 2026-10-28

Environment: Python 3.12, CasADi 3.7.0 / IPOPT from the local wheel cache,
NumPy 2.5.4, OpenBLAS/OMP limited to one thread. The ROS image pins NumPy 1.26.4
for compatibility with ROS Jazzy's binary packages.

| Check | Result |
|---|---|
| Unit and real-IPOPT tests | 12 passed |
| Native RPP sampler contract | g++ C++17 build and numerical comparison passed |
| Python compilation | Passed |
| Shell syntax | Passed |
| ROS Compose configuration | Passed |
| Isaac + two-robot Compose configuration | Passed |
| Isaac scene preflight | Passed, two robots, domain 31 |
| Wheel packaging and execution outside the checkout | Passed |
| ROS image / colcon build | Not run: Docker socket access denied |
| Live Isaac / RPP / Swerve physics run | Not run: ROS/Isaac unavailable in this session |

Numerical closed-loop runs used two independent planners. Each received only
the other robot's current position, world velocity, identity, and timestamp.
Both robot inputs were computed from the same observation snapshot before
either mathematical plant was advanced. Actual pair separation was sampled at
0.01 s during each 0.1 s control interval.

| Scenario | Prediction | Both goals reached | Minimum separation | Solver failures |
|---|---|---|---|---|
| A: head-on | CV + Gaussian | Yes, 14.4 s | 1.817 m | 0 |
| B: crossing | CV + Gaussian | Yes, 15.2 s | 1.826 m | 0 |
| D: static cylinder | CV + Gaussian | Yes, 14.6 s | 2.200 m | 0 |
| B, 80 ms solver budget | CV + Gaussian | Yes, 15.2 s | 1.826 m | 0 |

Required nominal center separation was 1.15 m. Solver medians in these runs
were approximately 8–12 ms; initialization and validation add overhead. These
are mathematical unicycle runs, not measurements from Isaac or RPP tracking.
The narrow corridor C is not a passing benchmark: the configured robot sizes
cannot pass side by side there, so a separate traffic policy is needed for
progress.

Reproduction commands are in [README.md](README.md#verification). Full numerical
traces were written to `isaac/results/numerical_{A,B,D,B_budget}.json` (ignored
runtime artifacts). No changes were made to the mpc_planner reference workspace.

## ROS interface mount correction

The user's subsequent ROS build generated `JointCommandFeedback` successfully,
but Isaac could not import it. Inspection of the generated install confirmed
absolute symlinks into `/workspace/isaac/message_build/build`, while Isaac
previously mounted the `isaac` subdirectory at `/workspace`. The launcher now
mounts the entire repository at `/workspace` and runs from `/workspace/isaac`,
matching the ROS build container. Existing message artifacts can be reused.

Three focused regression tests passed: launcher/container path agreement,
resolution of the actual generated message links under the corrected mount,
and early detection of broken message links. Merged Compose configuration and
shell syntax checks also passed. The runner now checks message imports and
native type support before starting Kit. Live Isaac rerun remains unverified
from this session because its Docker socket is inaccessible.
