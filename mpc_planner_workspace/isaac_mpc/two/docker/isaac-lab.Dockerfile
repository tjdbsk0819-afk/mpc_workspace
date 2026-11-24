# syntax=docker/dockerfile:1

ARG ISAAC_SIM_BASE_IMAGE=nvcr.io/nvidia/isaac-sim:6.0.1
FROM ${ISAAC_SIM_BASE_IMAGE}

USER root

ARG ISAAC_LAB_VERSION=3.0.0b2.post1
ARG TORCH_VERSION=2.10.0
ARG TORCHVISION_VERSION=0.25.0

ENV PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

# Isaac Lab's base image contains the Isaac ROS 2 bridge, but not the Zenoh
# RMW plugin. Install the ROS Jazzy package so rmw_zenoh_cpp can be selected at
# runtime instead of silently falling back to Fast DDS.
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl gnupg \
    && curl -L https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
       -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
       > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        python3-colcon-common-extensions \
        ros-jazzy-rmw-zenoh-cpp \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-generate-parameter-library \
        ros-jazzy-backward-ros \
        ros-jazzy-realtime-tools \
    && rm -rf /var/lib/apt/lists/*

# Isaac Lab 3.0 Beta 2 Patch 1 is the release matched to Isaac Sim 6.0.1.
# Install torch from the CUDA 12.8 index first so pip cannot select a newer,
# incompatible CPU/PyPI build while resolving the Isaac Lab wheel.
RUN /isaac-sim/python.sh -m pip install \
        --index-url https://download.pytorch.org/whl/cu128 \
        "torch==${TORCH_VERSION}" \
        "torchvision==${TORCHVISION_VERSION}"

RUN /isaac-sim/python.sh -m pip install \
        --extra-index-url https://pypi.nvidia.com \
        --pre \
        "isaaclab==${ISAAC_LAB_VERSION}" \
    && /isaac-sim/python.sh -c \
        "import isaaclab, torch, warp; assert isaaclab.__version__ == '${ISAAC_LAB_VERSION}'; assert torch.__version__.startswith('${TORCH_VERSION}'); print('Isaac Lab', isaaclab.__version__, 'Torch', torch.__version__, 'Warp', warp.__version__)"

# Resolve missing dependencies using the installed parent packages' constraints.
RUN /isaac-sim/python.sh -m pip install \
        "osqp==1.0.5" "msal==1.35.1" "aioboto3==15.5.0" \
    && /isaac-sim/python.sh -m pip check
