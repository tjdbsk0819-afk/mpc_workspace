#!/usr/bin/env bash
set -euo pipefail
export XDG_RUNTIME_DIR="/tmp/antbot-gazebo-runtime-$(id -u)"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
# Gazebo embeds an OpenGL scene in Qt Quick: keep Qt OpenGL enabled,
# while Mesa llvmpipe performs rendering on the CPU.
export LANG=C.UTF-8 LC_ALL=C.UTF-8
export QT_QPA_PLATFORM=xcb QT_OPENGL=desktop QT_XCB_GL_INTEGRATION=xcb_glx QSG_RENDER_LOOP=basic
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe LP_NUM_THREADS=2
unset QT_QUICK_BACKEND WAYLAND_DISPLAY
printf 'Gazebo GUI: Ogre / CPU rendering, DISPLAY=%s\n' "$DISPLAY"
export GZ_GUI_PLUGIN_PATH="/workspace/gazebo/build/plugin${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
if [[ ! -f /workspace/gazebo/build/plugin/libRobotSpeedPanel.so ]];then
    echo 'Build the Gazebo GUI panel first: run_gazebo.sh build-gui' >&2;exit 1
fi
config=$(python3 /workspace/gazebo/gui/gui_config.py)
exec gz sim -g -v 3 --render-engine-gui ogre --gui-config "$config"
