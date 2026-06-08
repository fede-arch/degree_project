#!/bin/bash
# preview.sh - Genera e visualizza un world di esempio
# Uso: ./preview.sh [seed]

SEED=${1:-42}
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Generando world con seed=$SEED..."

# Genera model.sdf dal template con valori default
sed \
  -e 's/__THRESHOLD__/0.001/' \
  -e 's/__S_MAX__/20/' \
  -e 's/__SMOOTH_L__/5/' \
  -e 's/__GAIN_STEER__/0.5/' \
  -e 's/__GAIN_PITCH__/0.5/' \
  -e 's/__RADIUS_ARRIVED__/5.0/' \
  -e 's/__RADIUS_SLOWDOWN__/15.0/' \
  lrauv_description/models/tethys_equipped/model.sdf.template \
  > lrauv_description/models/tethys_equipped/model.sdf

# Genera il world con rocce casuali
python3 -c "
import sys
sys.path.append('lrauv_gazebo_plugins/scripts')
from generate_world import generate_world
generate_world('.', seed=$SEED)
"

echo "Lanciando Gazebo..."
xhost +local:docker

docker run -it --rm \
  --device /dev/dri/card1 \
  --device /dev/dri/renderD128 \
  --group-add $(getent group render | cut -d: -f3) \
  --group-add $(getent group video | cut -d: -f3) \
  --env DISPLAY=$DISPLAY \
  --env XDG_RUNTIME_DIR=/tmp/runtime-developer \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $PROJECT_DIR:/lrauv_ws/src/degree_project \
  lrauv:harmonic \
  bash -c "source /setup.sh && \
    NEEDS_CMAKE=false && \
    HYDRO_SO=/lrauv_ws/src/degree_project/docker_build/libHydrodynamicsPlugin.so && \
    NAV_SO=/lrauv_ws/src/degree_project/docker_build/libNavigationPlugin.so && \
    HYDRO_CC=/lrauv_ws/src/degree_project/lrauv_gazebo_plugins/src/HydrodynamicsPlugin.cc && \
    NAV_CC=/lrauv_ws/src/degree_project/lrauv_gazebo_plugins/src/NavigationPlugin.cc && \
    if [ ! -f \$HYDRO_SO ] || [ \$HYDRO_CC -nt \$HYDRO_SO ]; then \
      echo 'HydrodynamicsPlugin needs rebuild...' && NEEDS_CMAKE=true; \
    fi && \
    if [ ! -f \$NAV_SO ] || [ \$NAV_CC -nt \$NAV_SO ]; then \
      echo 'NavigationPlugin needs rebuild...' && NEEDS_CMAKE=true; \
    fi && \
    if [ \$NEEDS_CMAKE == 'true' ]; then \
      mkdir -p /lrauv_ws/src/degree_project/docker_build && \
      cd /lrauv_ws/src/degree_project/docker_build && \
      cmake ../lrauv_gazebo_plugins -DCMAKE_BUILD_TYPE=Release -Wno-dev && \
      make -j4 HydrodynamicsPlugin && \
      make -j4 NavigationPlugin && \
      echo 'Build done!'; \
    else \
      echo 'Plugins up to date, skipping build...'; \
    fi && \
    echo 'Launching Gazebo...' && \
    export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models && \
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build && \
    gz sim /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world.sdf"