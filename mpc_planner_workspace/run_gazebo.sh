#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
exec bash "$root/gazebo_mpc/run_gazebo.sh" "$@"
