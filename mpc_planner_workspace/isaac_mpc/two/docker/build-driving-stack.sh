#!/usr/bin/env bash
# Run inside antbot-mpc:jazzy; original solver generation is unchanged.
set -eo pipefail
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
bash "$task_root/docker/build-antbot-controllers.sh"
bash "$task_root/docker/build-upstream-mpc.sh"
