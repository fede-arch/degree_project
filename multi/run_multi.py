#!/usr/bin/env python3

import subprocess
import time
import os
import sys
import random

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "../lrauv_gazebo_plugins/scripts"))
from generate_multi_lrauv import generate_multi_lrauv

# CONFIGURAZIONE
N_DRONES     = 5
STARTUP_WAIT = 25.0
RUN_TIMEOUT  = 400

# PATH
PROJECT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
LOG_FILE    = os.path.join(PROJECT_DIR, "multi/gazebo.log")
MULTI_BIN   = "/lrauv_ws/src/degree_project/docker_build/MultiLrauv"

# CONTAINER
log_handle     = None
container_name = "multi_gazebo"

def play():
    subprocess.run(
        ["docker", "exec", container_name, "bash", "-c",
         "source /setup.sh && gz service -s /world/empty_environment/control "
         "--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
         "--req 'pause: false' --timeout 2000"],
        capture_output=True)

def build_plugins():
    print("[MULTI] Build...")
    subprocess.run(["docker", "rm", "-f", "multi_build"], capture_output=True)
    r = subprocess.run([
        "docker", "run", "--rm", "--name", "multi_build",
        "--volume", f"{PROJECT_DIR}:/lrauv_ws/src/degree_project",
        "lrauv:harmonic", "bash", "-c",
        (
            "source /setup.sh && "
            "mkdir -p /lrauv_ws/src/degree_project/docker_build && "
            "cd /lrauv_ws/src/degree_project/docker_build && "
            "cmake ../lrauv_gazebo_plugins -DCMAKE_BUILD_TYPE=Release -Wno-dev && "
            "make -j4 HydrodynamicsPlugin && "
            "make -j4 NavigationPlugin && "
            "make -j4 MultiLrauv && "
            "echo BUILD_OK"
        )
    ], capture_output=True, text=True)
    if "BUILD_OK" not in (r.stdout + r.stderr):
        print(r.stdout); print(r.stderr)
        raise RuntimeError("Build FALLITA")
    print("[MULTI] Build OK")

def start_docker():
    global docker_proc, log_handle
    subprocess.run(["docker", "rm", "-f", container_name], capture_output=True)
    render_gid = os.popen("getent group render | cut -d: -f3").read().strip()
    video_gid  = os.popen("getent group video | cut -d: -f3").read().strip()
    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    log_handle = open(LOG_FILE, "w")
    docker_proc = subprocess.Popen([
        "docker", "run", "--rm",
        "--name", container_name,
        "--device", "/dev/dri/card1",
        "--device", "/dev/dri/renderD128",
        "--group-add", render_gid,
        "--group-add", video_gid,
        "--volume", f"{PROJECT_DIR}:/lrauv_ws/src/degree_project",
        "lrauv:harmonic", "bash", "-c",
        (
            "source /setup.sh && "
            "export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models && "
            "export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build && "
            "gz sim --headless-rendering -s "
            "/lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world.sdf"
        )
    ], stdout=log_handle, stderr=subprocess.STDOUT, text=True)
    time.sleep(STARTUP_WAIT)

def stop_docker():
    global docker_proc, log_handle
    subprocess.run(["docker", "rm", "-f", container_name], capture_output=True)
    if docker_proc:
        docker_proc.terminate()
        docker_proc = None
    if log_handle:
        log_handle.close()
        log_handle = None

if __name__ == "__main__":
    try:
        seed = random.randint(0, 9999)
        print(f"[MULTI] Seed: {seed}, droni: {N_DRONES}")

        build_plugins()
        generate_multi_lrauv(PROJECT_DIR, seed=seed, n_drones=N_DRONES)
        start_docker()

        listener = subprocess.Popen(
            ["docker", "exec", container_name, "bash", "-c",
             f"source /setup.sh && {MULTI_BIN} {N_DRONES}"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(2.0)
        play()

        try:
            out, _ = listener.communicate(timeout=RUN_TIMEOUT)
            print(out)
        except subprocess.TimeoutExpired:
            listener.kill()
            print("[MULTI] Timeout!")

    finally:
        stop_docker()