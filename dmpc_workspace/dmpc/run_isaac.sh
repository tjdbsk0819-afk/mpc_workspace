#!/usr/bin/env bash
# Host entrypoint. Commands use this workspace's assets/builds only.
set -euo pipefail
repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export LOCAL_UID=${LOCAL_UID:-$(id -u)} LOCAL_GID=${LOCAL_GID:-$(id -g)}
compose=(docker compose --env-file "$repo_root/isaac/.env" --project-directory "$repo_root"
         -f "$repo_root/isaac/docker/compose.dmpc.yaml")
command=${1:-help}
if [[ $# -gt 0 ]]; then shift; fi
case "$command" in
    build)
        mkdir -p "$repo_root/isaac/message_build" "$repo_root/isaac/control_build"
        "${compose[@]}" build ros
        "${compose[@]}" run --rm ros bash isaac/docker/build-dmpc.sh
        ;;
    router) exec "${compose[@]}" up -d router ;;
    sim) exec bash "$repo_root/isaac/docker/run-two-antbots.sh" --run "$@" ;;
    stop-sim)
        running_simulators=$(docker ps -q \
            --filter label=com.docker.compose.project=dmpc-two-antbots \
            --filter label=com.docker.compose.service=isaac-lab)
        if [[ -n "$running_simulators" ]]; then
            mapfile -t simulator_ids <<< "$running_simulators"
            exec docker stop --signal SIGINT --timeout 30 "${simulator_ids[@]}"
        fi
        echo "No DMPC Isaac simulator is running."
        ;;
    drive) exec "${compose[@]}" run --rm ros ros2 launch antbot_dmpc_ros two_antbot_dmpc.launch.py "$@" ;;
    viewer) exec "${compose[@]}" run --rm --name dmpc-two-antbots-path-viewer ros \
        python -m dmpc_runtime.path_viewer --host 0.0.0.0 --port 8092 "$@" ;;
    test) exec "${compose[@]}" run --rm --name dmpc-two-antbots-test ros \
        python -m dmpc_runtime.isaac_test "$@" ;;
    shell) exec "${compose[@]}" run --rm ros bash ;;
    check)
        "${compose[@]}" config --quiet
        exec python3 "$repo_root/isaac/simulation/run_two_antbots.py" --project-root "$repo_root/isaac" --check-config "$@"
        ;;
    *) echo "Usage: bash run_isaac.sh {build|router|sim [--scenario B --headless]|stop-sim|drive [scenario:=B]|viewer [--scenario B]|test A [--timeout 120]|shell|check [--scenario B]}"
       [[ "$command" == help ]] || exit 2 ;;
esac
