#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
SEED="${1:-$RANDOM}"

# ─── Parametri plugin (unica fonte di verità) ────────────────────
GAIN_STEER="0.8"
GAIN_PITCH="0.8"
MAX_FIN_ANGLE="0.15"
RADIUS_ARRIVED="10.0"
MAX_ITERATIONS="200000"
R_MAX="50.0"
VALLEY_THRESHOLD="34.4"
GRID_DECAY="0.982"
MAGNITUDE_A="10.88"
SMOOTH_L="5"
SAFETY_WINDOW="2"
FACE_GOAL="False"

echo " Gazebo GUI run  |  seed=$SEED"
echo "============================================================"
echo "   r_max          = $R_MAX m"
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

# ─── 1. Genera il world e scrivi model.sdf ───────────────────────
echo "[RUN] Generazione world..."
python3 - <<EOF
import sys
sys.path.insert(0, '$PROJECT_DIR/lrauv_gazebo_plugins/scripts')
from generate_world import generate_world

gx, gy, gz = generate_world('$PROJECT_DIR', seed=$SEED, face_goal=$FACE_GOAL)

tmpl = '$PROJECT_DIR/lrauv_description/models/tethys_equipped/model.sdf.template'
dest = '$PROJECT_DIR/lrauv_description/models/tethys_equipped/model.sdf'
lidar_tmpl = '$PROJECT_DIR/lrauv_description/models/tethys/model.sdf.template'
lidar_dest = '$PROJECT_DIR/lrauv_description/models/tethys/model.sdf'

r_max    = float('$R_MAX')
r_active = r_max * 0.75

with open(lidar_tmpl) as f:
    lidar_content = f.read()

lidar_content = lidar_content.replace('__R_MAX__', str(r_max))

with open(lidar_dest, 'w') as f:
    f.write(lidar_content)

with open(tmpl) as f:
    content = f.read()

content = content.replace('__DRONE_ID__',            '0')
content = content.replace('__GOAL_X__',              f'{gx:.2f}')
content = content.replace('__GOAL_Y__',              f'{gy:.2f}')
content = content.replace('__GOAL_Z__',              f'{gz:.2f}')
content = content.replace('__GAIN_STEER__',          '$GAIN_STEER')
content = content.replace('__GAIN_PITCH__',          '$GAIN_PITCH')
content = content.replace('__SMOOTH_L__',            '$SMOOTH_L')
content = content.replace('__MAX_FIN_ANGLE__',       '$MAX_FIN_ANGLE')
content = content.replace('__RADIUS_ARRIVED__',      '$RADIUS_ARRIVED')
content = content.replace('__MAX_ITERATIONS__',      '$MAX_ITERATIONS')
content = content.replace('__R_MAX__',                  str(r_max))
content = content.replace('__R_MIN__',                       '2.5')
content = content.replace('__R_ACTIVE__',               str(r_active))
content = content.replace('__VALLEY_THRESHOLD__',    '$VALLEY_THRESHOLD')
content = content.replace('__GRID_DECAY__',          '$GRID_DECAY')
content = content.replace('__MAGNITUDE_A__',         '$MAGNITUDE_A')
content = content.replace('__SAFETY_WINDOW__',       '$SAFETY_WINDOW')


with open(dest, 'w') as f:
    f.write(content)

print(f'[RUN] Goal: ({gx:.1f}, {gy:.1f}, {gz:.1f})')
EOF

# ─── 2. Ricompila plugin se sorgenti modificati ──────────────────
PLUGIN_SO="$PROJECT_DIR/docker_build/libNavigationPlugin.so"
SRC_DIR="$PROJECT_DIR/lrauv_gazebo_plugins/src"
HDR_DIR="$PROJECT_DIR/lrauv_gazebo_plugins/include"

NEEDS_BUILD=false
if [ ! -f "$PLUGIN_SO" ]; then
    echo "[RUN] Plugin non trovato → compilazione necessaria"
    NEEDS_BUILD=true
else
    NEWER=$(find "$SRC_DIR" "$HDR_DIR" \
        \( -name "*.cc" -o -name "*.hh" -o -name "*.cpp" -o -name "*.hpp" \) \
        -newer "$PLUGIN_SO" 2>/dev/null | head -1)
    if [ -n "$NEWER" ]; then
        echo "[RUN] Sorgente modificato: $(basename $NEWER) → ricompilazione..."
        NEEDS_BUILD=true
    fi
fi

if $NEEDS_BUILD; then
    docker run --rm --name es_build \
      --volume "$PROJECT_DIR:/lrauv_ws/src/degree_project" \
      lrauv:harmonic bash -c "
        source /setup.sh
        mkdir -p /lrauv_ws/src/degree_project/docker_build
        cd /lrauv_ws/src/degree_project/docker_build
        cmake ../lrauv_gazebo_plugins -DCMAKE_BUILD_TYPE=Release -Wno-dev \
              -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 2>&1 | tail -3
        make -j4 NavigationPlugin 2>&1 | tail -5
        echo BUILD_OK
      " | tee /tmp/run_build.log
    if ! grep -q "BUILD_OK" /tmp/run_build.log; then
        echo "[RUN] ✗ Build FALLITA — controlla /tmp/run_build.log"
        exit 1
    fi
    echo "[RUN] ✓ Build completata"
else
    echo "[RUN] Plugin aggiornato, nessuna ricompilazione"
fi

# ─── 3. X11 forwarding ───────────────────────────────────────────
xhost +local:docker 2>/dev/null || true

RENDER_GID=$(getent group render | cut -d: -f3 2>/dev/null || echo "")
VIDEO_GID=$(getent group video  | cut -d: -f3 2>/dev/null || echo "")
EXTRA_GROUPS=()
[[ -n "$RENDER_GID" ]] && EXTRA_GROUPS+=(--group-add "$RENDER_GID")
[[ -n "$VIDEO_GID"  ]] && EXTRA_GROUPS+=(--group-add "$VIDEO_GID")

# ─── 4. Lancia Gazebo con GUI ────────────────────────────────────
echo "[RUN] Avvio Gazebo con GUI..."
echo "[RUN] Chiudi la finestra di Gazebo per terminare."
echo ""

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
    export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build
    gz sim -r /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world.sdf
  "