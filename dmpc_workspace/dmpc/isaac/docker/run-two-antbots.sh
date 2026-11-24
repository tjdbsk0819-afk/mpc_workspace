#!/usr/bin/env bash
# Enter the Isaac container; run the Python runner manually as with run_antbot.
set -euo pipefail
launch_mode=shell
case "${1:-}" in
    --run) launch_mode=run; shift ;;
    --shell) shift ;;
    "") ;;
    *) echo "Usage: bash docker/run-two-antbots.sh [--shell | --run RUNNER_ARGS...]" >&2; exit 2 ;;
esac
if [[ "$launch_mode" == shell && $# -ne 0 ]]; then
    echo "Pass runner options inside the container, or use --run RUNNER_ARGS..." >&2
    exit 2
fi
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
export ISAAC_MPC_ROOT=$(cd "$task_root/.." && pwd)
if [[ ! -f "$task_root/docker/compose.isaac.yaml" || ! -f "$task_root/.env" ]]; then
    echo "Isaac Compose/environment not found at $task_root" >&2
    exit 1
fi
compose=(docker compose \
    --env-file "$task_root/.env" \
    --project-directory "$task_root" \
    -f "$task_root/docker/compose.isaac.yaml" \
    -f "$task_root/docker/compose.two_antbots.yaml" \
    --profile lab)
running_simulators=$(docker ps \
    --filter label=com.docker.compose.project=dmpc-two-antbots \
    --filter label=com.docker.compose.service=isaac-lab \
    --format '{{.Names}}')
if [[ -n "$running_simulators" ]]; then
    echo "A DMPC Isaac simulator is already running:" >&2
    echo "$running_simulators" >&2
    echo "Stop it before switching scenarios: bash run_isaac.sh stop-sim" >&2
    exit 1
fi
# Only the dedicated Docker cache volumes are writable in this root helper.
# The physics process continues to use the original non-root Isaac user.
"${compose[@]}" run --rm --no-deps two-antbot-cache-init
# The browser viewer is required for the stream; --no-deps avoids launching
# the original single-robot Isaac service through its dependency chain.
"${compose[@]}" up -d --no-deps web-viewer
if [[ "$launch_mode" == shell ]]; then
    echo "Inside the Isaac container, run:"
    echo "  cd /workspace/isaac"
    echo "  bash simulation/run_two_antbots.sh --scenario B"
    exec "${compose[@]}" run --rm --no-deps --entrypoint /bin/bash isaac-lab -i
fi
exec "${compose[@]}" run --rm --no-deps --entrypoint /bin/bash isaac-lab \
    /workspace/isaac/simulation/run_two_antbots.sh "$@"
