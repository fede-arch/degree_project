#!/bin/bash
xhost +local:docker 2>/dev/null

WORLD=${1:-empty}

echo "Generating model.sdf..."
python3 lrauv_description/scripts/description_generator.py \
  lrauv_description/models/tethys/model.sdf.in \
  lrauv_description/models/tethys/model.sdf

if [ "$WORLD" == "ledge" ]; then
  echo "Generating random target..."
  python3 lrauv_gazebo_plugins/scripts/target_generator.py
  WORLD_FILE="portuguese_ledge/portuguese_ledge.sdf"
  BUILD_NAV=true
  REGEN_LEDGE=true
elif [ "$WORLD" == "nav" ]; then
  echo "Generating random target..."
  python3 lrauv_gazebo_plugins/scripts/target_generator.py
  WORLD_FILE="navigation_world.sdf"
  BUILD_NAV=true
  REGEN_LEDGE=false
else
  WORLD_FILE="empty_environment.sdf"
  BUILD_NAV=false
  REGEN_LEDGE=false
fi

docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --env XDG_RUNTIME_DIR=/tmp/runtime-developer \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume $PWD:/lrauv_ws/src/degree_project \
  lrauv:harmonic \
  bash -c "source /setup.sh 2>/dev/null && \
    if [ '$REGEN_LEDGE' == 'true' ]; then \
      echo 'Regenerating portuguese_ledge.sdf...' && \
      python3 /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/scripts/empy_expander.py \
        /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/portuguese_ledge/portuguese_ledge.sdf.em \
        /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/portuguese_ledge/portuguese_ledge.sdf; \
    fi && \
    NEEDS_CMAKE=false && \
    HYDRO_SO=/lrauv_ws/src/degree_project/docker_build/libHydrodynamicsPlugin.so && \
    NAV_SO=/lrauv_ws/src/degree_project/docker_build/libNavigationPlugin.so && \
    HYDRO_CC=/lrauv_ws/src/degree_project/lrauv_gazebo_plugins/src/HydrodynamicsPlugin.cc && \
    NAV_CC=/lrauv_ws/src/degree_project/lrauv_gazebo_plugins/src/NavigationPlugin.cc && \
    if [ ! -f \$HYDRO_SO ] || [ \$HYDRO_CC -nt \$HYDRO_SO ]; then \
      echo 'HydrodynamicsPlugin needs rebuild...' && NEEDS_CMAKE=true; \
    fi && \
    if [ '$BUILD_NAV' == 'true' ] && ([ ! -f \$NAV_SO ] || [ \$NAV_CC -nt \$NAV_SO ]); then \
      echo 'NavigationPlugin needs rebuild...' && NEEDS_CMAKE=true; \
    fi && \
    if [ \$NEEDS_CMAKE == 'true' ]; then \
      mkdir -p /lrauv_ws/src/degree_project/docker_build && \
      cd /lrauv_ws/src/degree_project/docker_build && \
      cmake ../lrauv_gazebo_plugins -DCMAKE_BUILD_TYPE=Release -Wno-dev 2>/dev/null && \
      make -j4 HydrodynamicsPlugin 2>/dev/null && \
      if [ '$BUILD_NAV' == 'true' ]; then make -j4 NavigationPlugin 2>/dev/null; fi && \
      echo 'Build done!'; \
    else \
      echo 'Plugins up to date, skipping build...'; \
    fi && \
    echo 'Launching Gazebo...' && \
    export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models && \
    export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build && \
    gz sim /lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/$WORLD_FILE"
