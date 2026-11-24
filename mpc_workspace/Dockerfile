# syntax=docker/dockerfile:1
FROM ros:humble-ros-base-jammy

ARG ACADOS_VERSION=v0.4.0
ARG BUILD_JOBS=4
ARG BUILD_WORKSPACE=false
ENV BUILD_JOBS=${BUILD_JOBS}
ENV DEBIAN_FRONTEND=noninteractive \
    ACADOS_SOURCE_DIR=/opt/acados \
    LD_LIBRARY_PATH=/opt/acados/lib \
    VIRTUAL_ENV=/opt/mpc-venv \
    MPLBACKEND=Agg
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl ca-certificates pkg-config \
    libboost-dev libeigen3-dev libgsl-dev libomp-dev libyaml-cpp-dev \
    python3-pip python3-venv python3-colcon-common-extensions python3-rosdep \
    ros-humble-eigen3-cmake-module ros-humble-nav-msgs \
    ros-humble-tf2-geometry-msgs ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --branch "${ACADOS_VERSION}" --depth 1 --recurse-submodules \
        https://github.com/acados/acados.git /opt/acados \
    && cmake -S /opt/acados -B /opt/acados/build \
        -DCMAKE_BUILD_TYPE=Release -DACADOS_SILENT=ON \
        -DBLASFEO_TARGET=GENERIC -DHPIPM_TARGET=GENERIC \
    && cmake --build /opt/acados/build --parallel "${BUILD_JOBS}" \
    && cmake --install /opt/acados/build

# System site packages keep ROS Python modules available inside the venv.
RUN python3 -m venv --system-site-packages /opt/mpc-venv
ENV PATH=/opt/mpc-venv/bin:${PATH}
COPY docker/requirements.txt /tmp/mpc-requirements.txt
RUN pip install --no-cache-dir -r /tmp/mpc-requirements.txt \
    && pip install --no-cache-dir -e /opt/acados/interfaces/acados_template \
    && mkdir -p /opt/acados/bin \
    && curl -fsSL --retry 3 \
        https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux \
        -o /opt/acados/bin/t_renderer \
    && chmod +x /opt/acados/bin/t_renderer \
    && python -c 'import acados_template, casadi, numpy, yaml'

WORKDIR /mpc_workspace
COPY src/ src/
COPY docker/ /opt/mpc-docker/
RUN chmod +x /opt/mpc-docker/*.sh \
    && ln -s /opt/mpc-docker/build-workspace.sh /usr/local/bin/mpc-build \
    && if [ "${BUILD_WORKSPACE}" = true ]; then mpc-build; fi

ENTRYPOINT ["/opt/mpc-docker/entrypoint.sh"]
CMD ["bash"]
