#!/usr/bin/env bash
# Host entry point for the independent three-robot workspace.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$root"
command=${1:-help};if (($#));then shift;fi
scenario=staggered
if [[ ${1:-} == --scenario ]];then scenario=${2:?Scenario required};shift 2;fi
if [[ $scenario != staggered && $scenario != simultaneous ]];then echo 'Use staggered or simultaneous' >&2;exit 2;fi
compose=(docker compose --project-directory "$root" -f "$root/docker/compose.mpc.yaml" --profile mpc)
case "$command" in
 help|--help|-h)
  echo 'Usage: sudo bash run.sh {build|check|router|sim|drive|start|view|monitor|status|stop} [--scenario staggered|simultaneous]'
  echo 'Default staggered: C starts at (0,-5). simultaneous: C starts at (0,-3).'
  echo 'Shared starts/goals: simulation/three_robots.yaml'
  echo 'start also starts the path viewer at http://127.0.0.1:8091 (SSH tunnel for remote access).'
  ;;
 build) "${compose[@]}" run --rm -T mpc bash docker/build/build-driving-stack.sh ;;
 check) "${compose[@]}" run --rm -T mpc python simulation/run_three_antbots.py --check-config --scenario "$scenario" ;;
 router)
  if [[ $(docker inspect -f '{{.State.Running}}' antbot-zenoh 2>/dev/null || true) == true ]];then
   echo 'antbot-zenoh already running'
  else
   "${compose[@]}" run --rm -d --name antbot-zenoh mpc ros2 run rmw_zenoh_cpp rmw_zenohd
  fi ;;
 sim) exec bash docker/run-three-antbots.sh --run --scenario "$scenario" "$@" ;;
 drive)
  "${compose[@]}" run --rm -T mpc python simulation/tools/wait_three_ready.py
  exec "${compose[@]}" run --rm --name antbot-three-driving mpc ros2 launch antbot_mpc_ros three_antbot_driving.launch.py scenario:="$scenario" "$@" ;;
 start)
  for name in antbot-driving antbot-swerve antbot-three-driving antbot-three-isaac;do
   if [[ $(docker inspect -f '{{.State.Running}}' "$name" 2>/dev/null || true) == true ]];then echo "Already running: $name. Stop that experiment first." >&2;exit 1;fi
  done
  bash run.sh router
  bash run.sh view
  bash docker/run-three-antbots.sh --detach --run --scenario "$scenario" "$@"
  if ! "${compose[@]}" run --rm -T mpc python simulation/tools/wait_three_ready.py;then
   docker stop --signal SIGINT --timeout 8 antbot-three-isaac || true;exit 1
  fi
  if ! "${compose[@]}" run --rm -d --name antbot-three-driving mpc ros2 launch antbot_mpc_ros three_antbot_driving.launch.py scenario:="$scenario";then
   bash run.sh stop;exit 1
  fi
  if ! "${compose[@]}" run --rm -T mpc python simulation/tools/wait_three_ready.py --driving;then
   bash run.sh stop;exit 1
  fi
  echo 'Three-robot experiment started. Viewer: http://127.0.0.1:8091 ; monitor: sudo bash run.sh monitor'
  ;;
 view)
  if [[ $(docker inspect -f '{{.State.Running}}' antbot-three-viewer 2>/dev/null || true) != true ]];then
   "${compose[@]}" run --rm -d --name antbot-three-viewer mpc python /workspace/controller_bringup/antbot_mpc_ros/peer_prediction_viewer.py --port 8091 "$@"
  fi
  if ! docker exec -i antbot-three-viewer python - <<'PY'
import json,time,urllib.request
for _ in range(50):
 try:
  with urllib.request.urlopen('http://127.0.0.1:8091/api/state',timeout=1) as response:
   state=json.load(response)
  if state.get('robots')==['robot_a','robot_b','robot_c']:break
 except (OSError,ValueError):pass
 time.sleep(.1)
else:raise SystemExit('Three-robot viewer did not become ready on port 8091')
print('Three-robot path viewer: http://127.0.0.1:8091')
PY
  then
   docker logs --tail 20 antbot-three-viewer >&2 || true;exit 1
  fi
  ;;
 monitor) exec "${compose[@]}" run --rm -T mpc python simulation/tools/monitor_three.py ;;
 status) docker ps --format '{{.Names}}\t{{.Status}}' ;;
 stop)
  for name in antbot-three-driving antbot-three-isaac antbot-three-viewer;do
   if docker inspect "$name" >/dev/null 2>&1;then docker stop --signal SIGINT --timeout 8 "$name";fi
  done ;;
 *) echo "Unknown command: $command. Use: bash run.sh help" >&2;exit 2 ;;
esac
