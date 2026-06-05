#!/bin/bash
xhost +local:docker 

WORLD=${1:-empty}

echo "Generating model.sdf..."
python3 lrauv_description/scripts/description_generator.py \
  lrauv_description/models/tethys/model.sdf.in \
  lrauv_description/models/tethys/model.sdf

if [ "$WORLD" == "ledge" ]; then
  echo "Generating random target..."
  python3 lrauv_gazebo_plugins/scripts/target_generator.py
  WORLD_FILE="portuguese_ledge.sdf"
elif [ "$WORLD" == "nav" ]; then
  echo "Generating random target..."
  python3 lrauv_gazebo_plugins/scripts/target_generator.py
  WORLD_FILE="navigation_world.sdf"
else
  WORLD_FILE="empty_environment.sdf"
fi

docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --env XDG_RUNTIME_DIR=/tmp/runtime-developer \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $PWD:/lrauv_ws/src/degree_project \
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
    gz sim /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/$WORLD_FILE"