#!/bin/bash
xhost +local:docker 2>/dev/null

echo "Generating model.sdf..."
python3 lrauv_description/scripts/description_generator.py \
  lrauv_description/models/tethys/model.sdf.in \
  lrauv_description/models/tethys/model.sdf

docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --env XDG_RUNTIME_DIR=/tmp/runtime-developer \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $PWD:/lrauv_ws/src/degree_project \
  lrauv:harmonic \
  bash -c "source /setup.sh 2>/dev/null && \
    if [ ! -f /lrauv_ws/src/degree_project/docker_build/libHydrodynamicsPlugin.so ]; then \
      echo 'Building HydrodynamicsPlugin...' && \
      mkdir -p /lrauv_ws/src/degree_project/docker_build && \
      cd /lrauv_ws/src/degree_project/docker_build && \
      cmake ../lrauv_gazebo_plugins -DCMAKE_BUILD_TYPE=Release -Wno-dev 2>/dev/null && \
      make -j4 HydrodynamicsPlugin 2>/dev/null && \
      echo 'Plugin built successfully!'; \
    else \
      echo 'Plugin already built'; \
    fi && \
    echo 'Launching Gazebo...' && \
    export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models && \
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build && \
    gz sim /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/tethys_at_empty_environment.sdf 2>/dev/null"