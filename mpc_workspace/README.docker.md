# MPC workspace container

ROS 2 Humble (Ubuntu 22.04), colcon, RViz, acados v0.4.0, and a Python 3.10
solver environment. Requires Docker Engine and the Docker Compose plugin on the host.

## Build and enter

```bash
cd /home/ubuntu/tjdbsk/mpc_workspace
docker compose build
docker compose run --rm mpc
```

The default image is a development environment containing a snapshot of `src`.
Rebuild after editing host sources. The Compose mount exposes the current host
sources read-only at `/host_src`; solver generation changes only the container copy.
The host `.venv`, build outputs, and repository metadata are excluded.

## Generate and compile

Inside the container:

```bash
mpc-build
source /mpc_workspace/install/setup.bash
ros2 pkg executables mpc_planner_jackal
```

To require successful solver generation and compilation during image creation:

```bash
BUILD_WORKSPACE=true docker compose build
docker compose run --rm mpc ros2 pkg executables mpc_planner_jackal
```

`mpc-build` selects acados for the Jackal configuration and uses the included
ROS 2 solver CMake adapter: the checkout's original CMake file still refers to
FORCES files and a nonexistent `src/solver_interface.cpp`. The Jackal CMake source
reference is also corrected to the existing `src/ros2_planner.cpp`. Original host sources
are preserved. Simulator and ROS navigation packages are excluded because their
external dependencies (`pedestrian_simulator`, `roadmap`, and Jackal Gazebo) are
not present. This is not a complete simulator image. ROS topics use host networking;
set `ROS_DOMAIN_ID` to match the robot. RViz requires separate display forwarding.

The upstream Python 3.8 / FORCES requirements are not used for this acados image.
acados and CasADi are pinned to versions compatible with the older generator API.
The bundled Tera renderer is the Linux x86-64 binary; this configuration targets amd64.

Docker image and MPC compilation have not been validated on this host because no
Docker Engine or alternate container builder is installed.

References: [Docker ROS 2 guide](https://docs.docker.com/guides/ros2/),
[acados Python interface](https://github.com/acados/acados/blob/v0.4.0/interfaces/acados_template/README.md).

## Gazebo desktop through noVNC

Build the base images, then start the browser desktop (no host DISPLAY required):

```bash
docker compose build
docker build -f Dockerfile.gazebo -t mpc-workspace:humble-gazebo .
docker compose -f compose.vnc.yaml up -d --build
```

Xvfb, Openbox, x11vnc, noVNC, and Gazebo run under Supervisor. Rendering uses
software OpenGL. Both VNC (5900) and noVNC (6080) listen only on server loopback;
the desktop has no VNC password and should be accessed through an SSH tunnel.
Run on your own computer:

```bash
ssh -N -L 6080:127.0.0.1:6080 ubuntu@SERVER_ADDRESS
```

Open `http://localhost:6080/vnc.html?autoconnect=true&resize=scale` in your browser.
On the server, inspect or enter the container:

```bash
docker exec mpc-gazebo-vnc tail -n 80 /tmp/gazebo-error.log
docker exec -it mpc-gazebo-vnc bash
```

In the container, `mpc-build` builds MPC. Gazebo initially opens an empty world;
robot spawning and MPC topic wiring are separate setup steps. The ROS domain
(default 42) must match any other ROS 2 processes you connect. Stop with
`docker compose -f compose.vnc.yaml stop`; rebuilds that recreate the container
lose container-local MPC build outputs. Host sources remain in `/host_src`.
