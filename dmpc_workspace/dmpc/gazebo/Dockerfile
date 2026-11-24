ARG BASE_IMAGE=antbot-dmpc:jazzy
FROM ${BASE_IMAGE}
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
    ros-jazzy-gz-sim-vendor ros-jazzy-gz-plugin-vendor \
    && rm -rf /var/lib/apt/lists/*

# Korean glyphs for the native Qt HUD, and SVG icons used by Gazebo itself.
RUN apt-get update && apt-get install -y --no-install-recommends \
    fonts-noto-cjk libqt5svg5 fontconfig \
    && fc-cache -f && rm -rf /var/lib/apt/lists/*
