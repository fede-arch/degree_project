#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
N_DRONES="${1:-20}"
SEED="${2:-$RANDOM}"

R_MAX="40.0"
RADIUS_ARRIVED="4.0"
FACE_GOAL="true"

echo " Gazebo GUI — multi drone  |  seed=$SEED  |  droni=$N_DRONES"
echo " r_max          = $R_MAX m"
echo " radius_arrived = $RADIUS_ARRIVED m"
echo " face_goal      = $FACE_GOAL"
echo " (parametri VFH da final_theta.json)"
echo ""

python3 "$PROJECT_DIR/tools/setup/setup_run.py" multi "$PROJECT_DIR" "$N_DRONES" "$SEED" "$FACE_GOAL" "$R_MAX" "$RADIUS_ARRIVED"

source "$PROJECT_DIR/tools/setup/build_plugin.sh"
xhost +local:docker 2>/dev/null || true

RENDER_GID=$(getent group render | cut -d: -f3 2>/dev/null || echo "")
VIDEO_GID=$(getent group video  | cut -d: -f3 2>/dev/null || echo "")
EXTRA_GROUPS=()
[[ -n "$RENDER_GID" ]] && EXTRA_GROUPS+=(--group-add "$RENDER_GID")
[[ -n "$VIDEO_GID"  ]] && EXTRA_GROUPS+=(--group-add "$VIDEO_GID")

CONTAINER_BASE="/lrauv_ws/src/degree_project/lrauv_description/models"
CONTAINER_RESOURCE_PATH="$CONTAINER_BASE"
for (( i=0; i<N_DRONES; i++ )); do
    CONTAINER_RESOURCE_PATH="$CONTAINER_BASE/tethys_equipped_${i}:$CONTAINER_RESOURCE_PATH"
done

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
    export GZ_SIM_RESOURCE_PATH=$CONTAINER_RESOURCE_PATH
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build
    gz sim -r /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world_multi.sdf
  "