#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

N_DRONES="20"
SEED="${2:-$RANDOM}"

# Parametri plugin
GAIN_STEER="0.8"
GAIN_PITCH="0.8"
MAX_FIN_ANGLE="0.15"
RADIUS_ARRIVED="4.0"
MAX_ITERATIONS="200000"
R_MAX="100.0"
VALLEY_THRESHOLD="34.4"
GRID_DECAY="0.982"
MAGNITUDE_A="10.88"
SMOOTH_L="5"
SAFETY_WINDOW="2"
FACE_GOAL="False"

echo " Gazebo GUI run  |  seed=$SEED  |  droni=$N_DRONES"
echo "   r_max          = $R_MAX m"
echo "   r_active       = $R_ACTIVE m (75% automatico)"
echo "   valley_thr     = $VALLEY_THRESHOLD"
echo "   smooth_l       = $SMOOTH_L"
echo "   grid_decay     = $GRID_DECAY"
echo "   magnitude_a    = $MAGNITUDE_A"
echo "   safety_window  = $SAFETY_WINDOW bin"
echo "   gain_steer     = $GAIN_STEER"
echo "   gain_pitch     = $GAIN_PITCH"
echo "   max_fin_angle  = $MAX_FIN_ANGLE rad"
echo "   radius_arrived = $RADIUS_ARRIVED m"
echo "   max_iterations = $MAX_ITERATIONS"
echo ""

# 1. Genera il world e scrivi i model.sdf 
echo "[RUN] Generazione world con $N_DRONES droni..."
python3 - <<EOF
import sys
sys.path.insert(0, '$PROJECT_DIR/optimization')
sys.path.insert(0, '$PROJECT_DIR/lrauv_gazebo_plugins/scripts')
from es_utils import write_sdf
from generate_world_multi import generate_world_multi

n_drones = $N_DRONES
spawns, goals = generate_world_multi(
    '$PROJECT_DIR', seed=$SEED, face_goal=$FACE_GOAL, n_drones=n_drones)

theta = {
    "gain_steer": $GAIN_STEER, "gain_pitch": $GAIN_PITCH,
    "valley_threshold": $VALLEY_THRESHOLD, "grid_decay": $GRID_DECAY,
    "magnitude_a": $MAGNITUDE_A, "smooth_l": $SMOOTH_L,
    "safety_window": $SAFETY_WINDOW
}

for i, (gx, gy, gz) in enumerate(goals):
    model_dir = f'$PROJECT_DIR/lrauv_description/models/tethys_equipped_{i}'
    write_sdf(model_dir, theta, i, gx, gy, gz, $R_MAX, $RADIUS_ARRIVED)
    print(f'[RUN] Drone {i}: spawn=({spawns[i][0]:.1f},{spawns[i][1]:.1f},{spawns[i][2]:.1f}) goal=({gx:.1f},{gy:.1f},{gz:.1f})')
EOF

# 2. Ricompila plugin se sorgenti modificati
source "$SCRIPT_DIR/../tools/setup/build_plugin.sh"

# 3. X11 forwarding
xhost +local:docker 2>/dev/null || true

RENDER_GID=$(getent group render | cut -d: -f3 2>/dev/null || echo "")
VIDEO_GID=$(getent group video  | cut -d: -f3 2>/dev/null || echo "")
EXTRA_GROUPS=()
[[ -n "$RENDER_GID" ]] && EXTRA_GROUPS+=(--group-add "$RENDER_GID")
[[ -n "$VIDEO_GID"  ]] && EXTRA_GROUPS+=(--group-add "$VIDEO_GID")

# 4. Lancia Gazebo con GUI
echo "[RUN] Avvio Gazebo con GUI..."
echo "[RUN] Chiudi la finestra di Gazebo per terminare."
echo ""

# Costruisci resource path con tutte le cartelle dei droni
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
  lrauv:harmonic \
  bash -c "
    source /setup.sh
    export GZ_SIM_RESOURCE_PATH=$CONTAINER_RESOURCE_PATH:\$GZ_SIM_RESOURCE_PATH
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build
    gz sim -r /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world_multi.sdf
  "