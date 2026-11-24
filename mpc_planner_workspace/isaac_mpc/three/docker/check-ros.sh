#!/usr/bin/env bash
set -euo pipefail
echo "ROS=$ROS_DISTRO RMW=$RMW_IMPLEMENTATION DOMAIN=${ROS_DOMAIN_ID:-0}"
python - <<'PY'
import socket
import rclpy
from rclpy.utilities import get_available_rmw_implementations
assert 'rmw_zenoh_cpp' in get_available_rmw_implementations()
with socket.create_connection(('127.0.0.1', 7447), timeout=3):
    print('TCP listener on port 7447 is reachable; checking the ROS graph next')
PY
timeout 15s ros2 topic list --no-daemon
