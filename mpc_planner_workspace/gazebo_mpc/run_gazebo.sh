#!/usr/bin/env bash
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
command=${1:-help};if (($#));then shift;fi
scenario=B;robots=3;ego=robot_a;benchmark_timeout=180.0
if [[ "$command" == benchmark ]];then scenario=A;robots=4;fi
while (($#)); do
    case "$1" in
        --ego) ego=${2:?ego robot};shift 2;;
        --scenario) scenario=${2:?scenario};shift 2;;
        --robots) robots=${2:?robot count};shift 2;;
        --timeout) benchmark_timeout=${2:?benchmark timeout seconds};shift 2;;
        *) echo "Unknown option: $1" >&2;exit 2;;
    esac
done
[[ "$scenario" =~ ^[ABCD]$ && "$robots" =~ ^[34]$ ]] || { echo 'Use --scenario A|B|C|D --robots 3|4' >&2;exit 2; }
source "$root/gazebo/workspace.env"
export LOCAL_UID=${LOCAL_UID:-${SUDO_UID:-$(id -u)}} LOCAL_GID=${LOCAL_GID:-${SUDO_GID:-$(id -g)}}
export ANTBOT_PROJECT ANTBOT_SCENARIO=$scenario ANTBOT_ROBOTS=$robots
compose=(docker compose --project-name "$ANTBOT_PROJECT" --project-directory "$root" -f "$root/gazebo/compose.yaml")
if [[ "$command" == gui || "$command" == view || "$command" == timing || "$command" == shell ]];then
    source "$root/gazebo/single_mpc/attach_domain.sh"
fi
case "$command" in
    benchmark)
        [[ "$robots" == 4 ]] || { echo "benchmark requires --robots 4" >&2;exit 2; }
        exec python3 "$root/gazebo/single_mpc/run_benchmark.py" --scenario "$scenario" --ego "$ego"
        ;;
    build)
        available=$(df -Pk "$root" | awk 'NR==2 {print $4}')
        if ((available<10485760));then echo 'Jazzy/Gazebo/acados build requires at least 10 GiB free. Free disk space before build.' >&2;exit 1;fi
        docker_root=$(docker info --format '{{.DockerRootDir}}')
        if [[ -d "$docker_root" ]];then
            docker_available=$(df -Pk "$docker_root" | awk 'NR==2 {print $4}')
            ((docker_available>=10485760)) || { echo 'Docker storage requires at least 10 GiB free.' >&2;exit 1; }
        fi
        if [[ "$ANTBOT_PLANNER" == dmpc ]];then
            docker build -t antbot-dmpc:jazzy -f "$root/isaac/docker/Dockerfile.dmpc" "$root"
        else
            base=$(realpath "$root/../../dmpc_workspace/dmpc")
            [[ -f "$base/isaac/docker/Dockerfile.dmpc" ]] || { echo 'DMPC base source not found' >&2;exit 1; }
            docker build -t antbot-dmpc:jazzy -f "$base/isaac/docker/Dockerfile.dmpc" "$base"
            docker build -t antbot-mpc:jazzy -f "$root/docker/Dockerfile.mpc" "$root"
        fi
        "${compose[@]}" build runtime
        "${compose[@]}" run --rm runtime bash /workspace/gazebo/scripts/build.sh
        ;;
    build-gui)
        "${compose[@]}" run --rm --user 0 runtime bash -c 'cmake -S /workspace/gazebo/plugin -B /workspace/gazebo/build/plugin -DCMAKE_BUILD_TYPE=Release && cmake --build /workspace/gazebo/build/plugin --target RobotSpeedPanel --parallel 2'
        ;;
    check) python3 "$root/gazebo/scene/generate.py" --reference "$root/$ANTBOT_LOCAL_REFERENCE" --output "$root/gazebo/generated/check" --scenario "$scenario" --robots "$robots" --check;;
    sim) exec "${compose[@]}" run --rm --name "${ANTBOT_PROJECT}-sim" runtime bash /workspace/gazebo/scripts/sim.sh "$scenario" "$robots";;
    gui) source "$root/gazebo/gui/gui.sh";;
    drive)
        "${compose[@]}" run --rm runtime python3 /workspace/gazebo/tools/ready.py --scenario "$scenario" --robots "$robots"
        exec "${compose[@]}" run --rm --name "${ANTBOT_PROJECT}-drive" runtime ros2 launch /workspace/gazebo/launch/drive.launch.py scenario:="$scenario" robots:="$robots"
        ;;
    test) exec "${compose[@]}" run --rm runtime python3 /workspace/gazebo/tools/test_run.py --scenario "$scenario" --robots "$robots";;
    view)
        name="${ANTBOT_PROJECT}-viewer"
        if [[ "$ANTBOT_PLANNER" == mpc ]];then port=8091;else port=8092;fi
        if docker container inspect "$name" >/dev/null 2>&1;then
            if [[ $(docker inspect -f '{{.State.Running}}' "$name") == true ]];then
                echo "Path viewer already running: http://127.0.0.1:$port (stop before changing scenario/count)"
                exit 0
            fi
            docker rm "$name" >/dev/null
        fi
        "${compose[@]}" run --rm -d --name "$name" runtime python3 /workspace/gazebo/gui/path_viewer.py \
            --host 127.0.0.1 --port "$port" --robot-count "$robots" --scenario "$scenario"
        echo "Path viewer: http://127.0.0.1:$port"
        ;;
    timing) exec "${compose[@]}" run --rm runtime python3 /workspace/gazebo/tools/timing_monitor.py --robots "$robots";;
    record)
        prediction_seconds=$(python3 - "$root/mpc_planning/mpc_planner_antbot/config/settings.yaml" <<'PYHORIZON'
import sys
import yaml
with open(sys.argv[1]) as stream:
    data = yaml.safe_load(stream)
seconds = data["N"] * data["integrator_step"]
print(f"{seconds:g}")
PYHORIZON
        )
        record_name="${ANTBOT_PLANNER}_robot${robots}_예측${prediction_seconds}초_시나리오${scenario}"
        if [[ -e "$root/gazebo/results/$record_name" ]];then
            echo "Recording already exists: $root/gazebo/results/$record_name (move it before recording again)" >&2
            exit 2
        fi
        topics=(/clock /robot_a/odom /robot_b/odom /robot_c/odom)
        if [[ "$robots" == 4 ]];then topics+=(/robot_d/odom);fi
        echo "Recording scenario=$scenario robots=$robots to $root/gazebo/results/$record_name"
        exec "${compose[@]}" run --rm runtime ros2 bag record \
            -o "/workspace/gazebo/results/$record_name" "${topics[@]}"
        ;;
    shell) exec "${compose[@]}" run --rm runtime bash;;
    stop)
        benchmark_container="${ANTBOT_PROJECT}-benchmark-sim"
        if [[ $(docker inspect -f '{{.State.Running}}' "$benchmark_container" 2>/dev/null || true) == true ]];then
            docker exec "$benchmark_container" bash -c '
                if [[ -f /tmp/single_mpc_measurement.pid ]];then
                    measurement_pid=$(cat /tmp/single_mpc_measurement.pid)
                    kill -INT "$measurement_pid" 2>/dev/null || true
                    for ((attempt=0;attempt<300;attempt++));do
                        kill -0 "$measurement_pid" 2>/dev/null || break
                        sleep .1
                    done
                fi' || true
        fi
        for role in drive gui sim viewer benchmark-sim;do
            name="${ANTBOT_PROJECT}-$role"
            if docker container inspect "$name" >/dev/null 2>&1;then docker stop --signal SIGINT --timeout 30 "$name";fi
        done;;
    *) echo 'Usage: bash run_gazebo.sh {build|build-gui|check|sim|gui|drive|test|view|timing|record|benchmark|shell|stop} [--scenario A|B|C|D] [--robots 3|4] [--ego robot_a|robot_b|robot_c|robot_d]';;
esac
