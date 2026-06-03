#!/bin/bash
xhost +local:docker


docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $PWD:/lrauv_ws/src/degree_project \
  lrauv:harmonic \
  bash -c "source /setup.sh && \
    mkdir -p /tmp/build && cd /tmp/build && \
    cmake /lrauv_ws/src/degree_project/lrauv_gazebo_plugins -DCMAKE_BUILD_TYPE=Release && \
    make -j4 HydrodynamicsPlugin && \
    export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models && \
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/tmp/build && \
    gz sim /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/tethys_at_empty_environment.sdf"
