#!/bin/bash
xhost +local:docker 2>/dev/null

WORLD=${1:-empty}

echo "Generating model.sdf..."
python3 lrauv_description/scripts/description_generator.py \
  lrauv_description/models/tethys/model.sdf.in \
  lrauv_description/models/tethys/model.sdf

if [ "$WORLD" == "ledge" ]; then
  WORLD_FILE="portuguese_ledge.sdf"
else
  WORLD_FILE="empty_environment.sdf"
fi

docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --env XDG_RUNTIME_DIR=/tmp/runtime-developer \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $PWD:/lrauv_ws/src/degree_project \
  lrauv:harmonic \
  bash -c "source /setup.sh 2>/dev/null && \
    if [ '$WORLD_FILE' == 'portuguese_ledge.sdf' ]; then \
      echo 'Generating portuguese_ledge.sdf...' && \
      python3 /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/empy_expander.py \
        /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/portuguese_ledge.sdf.em \
        /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/portuguese_ledge.sdf; \
    fi && \
    if [ ! -f /lrauv_ws/src/degree_project/docker_build/libHydrodynamicsPlugin.so ]; then \
      echo 'Building HydrodynamicsPlugin...' && \
      mkdir -p /lrauv_ws/src/degree_project/docker_build && \
      cd /lrauv_ws/src/degree_project/docker_build && \
      cmake ../lrauv_gazebo_plugins -DCMAKE_BUILD_TYPE=Release -Wno-dev 2>/dev/null && \
      make -j4 HydrodynamicsPlugin 2>/dev/null && \
      echo 'Plugin built'; \
    else \
      echo 'Plugin already built, skipping...'; \
    fi && \
    echo 'Launching Gazebo...' && \
    export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models && \
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build && \
    gz sim /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/$WORLD_FILE 2>/dev/null"
