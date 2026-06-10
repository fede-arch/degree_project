#!/bin/bash
# run.sh  –  lancia una singola run con la GUI di Gazebo.
#
# Uso:
#   ./run.sh           → seed casuale
#   ./run.sh 42        → seed fisso (riproducibile)
#
# Se esiste es/results/best_theta.json, applica automaticamente
# i parametri ottimizzati; altrimenti usa i default del template.
#
# Mettilo nella cartella es/ accanto a es_optimizer.py

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SEED="${1:-$RANDOM}"

echo "============================================================"
echo " Gazebo GUI run  |  seed=$SEED"
echo "============================================================"

# ─── 1. Genera il world ──────────────────────────────────────────
echo "[RUN] Generazione world..."
python3 - <<EOF
import sys, os, json
sys.path.insert(0, '$PROJECT_DIR/lrauv_gazebo_plugins/scripts')
from generate_world import generate_world

gx, gy, gz = generate_world('$PROJECT_DIR', seed=$SEED)

# Se esiste best_theta.json, riscrivi model.sdf con i parametri trained
best_path = '$PROJECT_DIR/es/results/best_theta.json'
if os.path.exists(best_path):
    with open(best_path) as f:
        best = json.load(f)
    theta  = best['theta']
    reward = best.get('reward', 0)

    tmpl = '$PROJECT_DIR/lrauv_description/models/tethys_equipped/model.sdf.template'
    dest = '$PROJECT_DIR/lrauv_description/models/tethys_equipped/model.sdf'
    with open(tmpl) as f:
        content = f.read()

    content = content.replace('__GOAL_X__',         f'{gx:.2f}')
    content = content.replace('__GOAL_Y__',         f'{gy:.2f}')
    content = content.replace('__GOAL_Z__',         f'{gz:.2f}')
    content = content.replace('__THRESHOLD__',      str(theta.get('threshold',      0.001)))
    content = content.replace('__S_MAX__',          str(int(round(theta.get('s_max',      20)))))
    content = content.replace('__SMOOTH_L__',       str(int(round(theta.get('smooth_l',    5)))))
    content = content.replace('__GAIN_STEER__',     str(theta.get('gain_steer',     0.5)))
    content = content.replace('__GAIN_PITCH__',     str(theta.get('gain_pitch',     0.5)))
    content = content.replace('__RADIUS_SLOWDOWN__',str(theta.get('radius_slowdown',15.0)))

    with open(dest, 'w') as f:
        f.write(content)

    print(f'[RUN] Parametri trained applicati  (best reward={reward:.3f})')
    for k, v in theta.items():
        print(f'        {k}: {v:.4f}')
else:
    print('[RUN] Nessun best_theta.json trovato → parametri di default')

print(f'[RUN] Goal: ({gx:.1f}, {gy:.1f}, {gz:.1f})')
EOF

# ─── 2. X11 forwarding ───────────────────────────────────────────
xhost +local:docker 2>/dev/null || true

# ─── 3. GPU groups ───────────────────────────────────────────────
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