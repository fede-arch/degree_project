#!/bin/bash
xhost +local:docker

python3 lrauv_description/scripts/description_generator.py \
  lrauv_description/models/tethys/model.sdf.in \
  lrauv_description/models/tethys/model.sdf

docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $PWD:/lrauv_ws/src/degree_project \
  lrauv:harmonic \
  bash -c "source /setup.sh && \
    if [ ! -f /tmp/degree_build/libHydrodynamicsPlugin.so ]; then \
      mkdir -p /tmp/degree_build && cd /tmp/degree_build && \
      cmake /lrauv_ws/src/degree_project/lrauv_gazebo_plugins -DCMAKE_BUILD_TYPE=Release && \
      make -j4 HydrodynamicsPlugin; \
    fi && \
    export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models && \
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/tmp/degree_build && \
    gz sim /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/tethys_at_empty_environment.sdf"
