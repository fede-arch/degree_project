#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SEED="${1:-$RANDOM}"

GZ_PARTITION="tesi_live"
GZ_RELAY="192.168.0.140"
GZ_IP="192.168.0.49"

echo " Gazebo SERVER remoto  |  seed=$SEED"
echo "   partition = $GZ_PARTITION"
echo "   relay     = $GZ_RELAY"
echo "   ip        = $GZ_IP"
echo ""

python3 "$PROJECT_DIR/tools/setup/setup_run.py" remote "$PROJECT_DIR" "$SEED"

source "$PROJECT_DIR/tools/setup/build_plugin.sh"

xhost +local:docker 2>/dev/null || true

RENDER_GID=$(getent group render | cut -d: -f3 2>/dev/null || echo "")
VIDEO_GID=$(getent group video  | cut -d: -f3 2>/dev/null || echo "")
EXTRA_GROUPS=()
[[ -n "$RENDER_GID" ]] && EXTRA_GROUPS+=(--group-add "$RENDER_GID")
[[ -n "$VIDEO_GID"  ]] && EXTRA_GROUPS+=(--group-add "$VIDEO_GID")

docker run --rm \
  --name gz_server \
  --network=host \
  --device /dev/dri/card1 \
  --device /dev/dri/renderD128 \
  "${EXTRA_GROUPS[@]}" \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PROJECT_DIR:/lrauv_ws/src/degree_project" \
  lrauv:harmonic bash -c "
    source /setup.sh
    export GZ_PARTITION=$GZ_PARTITION
    export GZ_RELAY=$GZ_RELAY
    export GZ_IP=$GZ_IP
    export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build
    gz sim -r /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world.sdf
  "