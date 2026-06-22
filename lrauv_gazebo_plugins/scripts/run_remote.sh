#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
SEED="${1:-$RANDOM}"

GAIN_STEER="0.8"
GAIN_PITCH="0.8"
MAX_FIN_ANGLE="0.15"
RADIUS_ARRIVED="10.0"
MAX_ITERATIONS="200000"
R_MAX="100.0"
VALLEY_THRESHOLD="34.4"
GRID_DECAY="0.982"
MAGNITUDE_A="10.88"
SMOOTH_L="5"
SAFETY_WINDOW="2"

echo "============================================================"
echo " Gazebo SERVER (remoto)  |  seed=$SEED"
echo "============================================================"

# Genera world con template SENZA NavigationPlugin
python3 - <<PYEOF
import sys
sys.path.insert(0, '$PROJECT_DIR/lrauv_gazebo_plugins/scripts')
from generate_world import generate_world

gx, gy, gz = generate_world('$PROJECT_DIR', seed=$SEED, face_goal=False)

tmpl = '$PROJECT_DIR/lrauv_description/models/tethys_equipped/model_remote.sdf.template'
dest = '$PROJECT_DIR/lrauv_description/models/tethys_equipped/model.sdf'

r_max    = float('$R_MAX')
r_active = r_max * 0.75

with open(tmpl) as f:
    content = f.read()

content = content.replace('__DRONE_ID__', '0')
content = content.replace('__GOAL_X__',   f'{gx:.2f}')
content = content.replace('__GOAL_Y__',   f'{gy:.2f}')
content = content.replace('__GOAL_Z__',   f'{gz:.2f}')

with open(dest, 'w') as f:
    f.write(content)

print(f'[REMOTE] Goal: ({gx:.1f}, {gy:.1f}, {gz:.1f})')
print(f'[REMOTE] Avvia sul portatile:')
print(f'[REMOTE] ./standalone_controller tethys_0 {gx:.2f} {gy:.2f} {gz:.2f}')
PYEOF

# Permessi GPU
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
  -v "$PROJECT_DIR:/lrauv_ws/src/degree_project" \
  lrauv:harmonic \
  bash -c "
    source /setup.sh
    export GZ_PARTITION=tesi_live
    export GZ_RELAY=192.168.0.108
    export GZ_IP=192.168.0.49
    export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build
    gz sim -s -r /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world.sdf
  "