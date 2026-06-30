#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SEED="${1:-$RANDOM}"

R_MAX="40.0"
RADIUS_ARRIVED="4.0"
FACE_GOAL="false"

echo " Gazebo GUI — singolo drone  |  seed=$SEED"
echo " r_max          = $R_MAX m"
echo " radius_arrived = $RADIUS_ARRIVED m"
echo " face_goal      = $FACE_GOAL"
echo " (parametri VFH da final_theta.json)"
echo ""

python3 "$PROJECT_DIR/tools/setup/setup_run.py" single "$PROJECT_DIR" "$SEED" "$FACE_GOAL" "$R_MAX" "$RADIUS_ARRIVED"

source "$PROJECT_DIR/tools/setup/build_plugin.sh"
xhost +local:docker 2>/dev/null || true

RENDER_GID=$(getent group render | cut -d: -f3 2>/dev/null || echo "")
VIDEO_GID=$(getent group video  | cut -d: -f3 2>/dev/null || echo "")
EXTRA_GROUPS=()
[[ -n "$RENDER_GID" ]] && EXTRA_GROUPS+=(--group-add "$RENDER_GID")
[[ -n "$VIDEO_GID"  ]] && EXTRA_GROUPS+=(--group-add "$VIDEO_GID")

docker run --rm \
  --name es_gui \
  --device /dev/dri/card1 \
  --device /dev/dri/renderD128 \
  "${EXTRA_GROUPS[@]}" \
  -e DISPLAY="$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$PROJECT_DIR:/lrauv_ws/src/degree_project" \
  lrauv:harmonic bash -c "
    source /setup.sh
    export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build
    gz sim -r /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world.sdf
  "