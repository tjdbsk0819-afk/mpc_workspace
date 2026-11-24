#!/usr/bin/env bash
# Run from run_gazebo.sh after sourcing workspace.env and defining compose.
set -euo pipefail
[[ -n ${DISPLAY:-} ]] || { echo 'DISPLAY is empty. Run gui from a local graphical terminal.' >&2;exit 1; }
command -v xauth >/dev/null || { echo 'xauth is required on the host (sudo apt install xauth).' >&2;exit 1; }
uid=${SUDO_UID:-$(id -u)}
gid=${SUDO_GID:-$(id -g)}
user_home=$(getent passwd "$uid" | cut -d: -f6)
authority=${XAUTHORITY:-}
if [[ -z $authority || ! -r $authority ]];then
    authority=''
    for candidate in "/run/user/$uid/gdm/Xauthority" "/run/user/$uid"/.mutter-Xwaylandauth.* "$user_home/.Xauthority";do
        if [[ -f $candidate && -r $candidate ]];then authority=$candidate;break;fi
    done
fi
[[ -n $authority && -r $authority ]] || {
    echo 'No readable X11 authority file. Run: sudo --preserve-env=DISPLAY,XAUTHORITY bash run_gazebo.sh gui' >&2;exit 1;
}
# Forward only this display's cookie; FamilyWild permits the container hostname.
# Keep authentication contents out of console output and process arguments.
entries=$(xauth -i -f "$authority" nlist "$DISPLAY" 2>/dev/null) || { echo "Could not read X11 authorization for DISPLAY." >&2;exit 1; }
[[ -n $entries ]] || { echo 'No X11 authorization entry matches DISPLAY.' >&2;exit 1; }
cookie=$(mktemp /tmp/antbot-gazebo-xauth.XXXXXX)
trap 'rm -f -- "$cookie"' EXIT
printf '%s\n' "$entries" | sed 's/^..../ffff/' | xauth -f "$cookie" nmerge -
chmod 600 "$cookie"
if [[ $(id -u) == 0 ]];then chown "$uid:$gid" "$cookie";fi
unset entries
options=(--user "$uid:$gid" --env "DISPLAY=$DISPLAY" --env XAUTHORITY=/tmp/antbot-gazebo.xauth
         --volume "$cookie:/tmp/antbot-gazebo.xauth:ro")
# Keep abnormal exits inspectable rather than losing --rm containers.
container_name="${ANTBOT_PROJECT}-gui"
if docker container inspect "$container_name" >/dev/null 2>&1;then
    running=$(docker inspect -f '{{.State.Running}}' "$container_name")
    if [[ $running == true ]];then
        echo "Gazebo GUI is already running: $container_name" >&2;exit 1
    fi
    docker rm "$container_name" >/dev/null 2>&1 || true
    for attempt in {1..30};do
        docker container inspect "$container_name" >/dev/null 2>&1 || break
        sleep .1
    done
fi
status=0
"${compose[@]}" run --name "$container_name" "${options[@]}" runtime bash /workspace/gazebo/gui-container.sh || status=$?
if docker container inspect "$container_name" >/dev/null 2>&1;then
    docker inspect -f 'GUI exit={{.State.ExitCode}} OOMKilled={{.State.OOMKilled}} error={{.State.Error}}' "$container_name"
    if [[ $status == 0 ]];then
        docker rm "$container_name" >/dev/null
    else
        echo "GUI diagnostics: sudo docker logs $container_name" >&2
    fi
fi
exit "$status"
