#!/usr/bin/env bash
# Source from run_gazebo.sh: visualization must join the running benchmark's ROS graph.
benchmark_container="${ANTBOT_PROJECT}-benchmark-sim"
if [[ $(docker inspect -f '{{.State.Running}}' "$benchmark_container" 2>/dev/null || true) == true ]];then
    benchmark_domain=$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$benchmark_container" | sed -n 's/^ROS_DOMAIN_ID=//p')
    if [[ "$benchmark_domain" =~ ^[0-9]+$ ]];then
        export ROS_DOMAIN_ID="$benchmark_domain"
        echo "Connecting to benchmark: ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    else
        echo "Cannot determine benchmark ROS_DOMAIN_ID" >&2
        exit 1
    fi
fi
