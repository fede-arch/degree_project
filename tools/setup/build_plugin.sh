#!/bin/bash
# Uso: source build_plugin.sh
# Richiede che PROJECT_DIR sia già definito nel chiamante.

PLUGIN_SO="$PROJECT_DIR/docker_build/libNavigationPlugin.so"
SRC_DIR="$PROJECT_DIR/lrauv_gazebo_plugins/src"
HDR_DIR="$PROJECT_DIR/lrauv_gazebo_plugins/include"

NEEDS_BUILD=false
if [ ! -f "$PLUGIN_SO" ]; then
    echo "[BUILD] Plugin non trovato, compilazione necessaria"
    NEEDS_BUILD=true
else
    NEWER=$(find "$SRC_DIR" "$HDR_DIR" \
        \( -name "*.cc" -o -name "*.hh" -o -name "*.cpp" -o -name "*.hpp" \) \
        -newer "$PLUGIN_SO" 2>/dev/null | head -1)
    if [ -n "$NEWER" ]; then
        echo "[BUILD] Sorgente modificato: $(basename "$NEWER"), ricompilazione..."
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
        echo "[BUILD] Build FALLITA — /tmp/run_build.log"
        exit 1
    fi
    echo "[BUILD] Build completata"
else
    echo "[BUILD] Plugin aggiornato, nessuna ricompilazione"
fi