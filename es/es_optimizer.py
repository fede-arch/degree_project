#!/usr/bin/env python3
"""
Evolution Strategies per ottimizzare i parametri VFH del NavigationPlugin.
Docker viene avviato UNA SOLA VOLTA, gli episodi si resettano via gz service.
"""

import random
import subprocess
import numpy as np
import time
import os
import sys
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "../lrauv_gazebo_plugins/scripts"))
from generate_world import generate_world

# ─── PARAMETRI ES ───────────────────────────────────────────────────────────
N_WORKERS = 6
N_GEN     = 10
SIGMA     = 0.1
ALPHA     = 0.05

# ─── PARAMETRI VFH ──────────────────────────────────────────────────────────
THETA_INIT = np.array([0.001, 20.0, 5.0, 0.5, 0.5, 5.0, 15.0])
THETA_MIN  = np.array([0.0001, 5.0, 1.0, 0.1, 0.1, 2.0,  8.0])
THETA_MAX  = np.array([0.01,  40.0, 15.0, 2.0, 2.0, 10.0, 30.0])

PARAM_NAMES = ["threshold", "s_max", "smooth_l", "gain_steer",
               "gain_pitch", "radius_arrived", "radius_slowdown"]

# ─── PATH ───────────────────────────────────────────────────────────────────
PROJECT_DIR   = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
TEMPLATE_FILE = os.path.join(PROJECT_DIR, "lrauv_description/models/tethys_equipped/model.sdf.template")
MODEL_FILE    = os.path.join(PROJECT_DIR, "lrauv_description/models/tethys_equipped/model.sdf")
RESULTS_FILE  = os.path.join(PROJECT_DIR, "es/results/es_results.csv")

# ─── REWARD ─────────────────────────────────────────────────────────────────
REWARD = {"ARRIVED": 1.0, "COLLISION": -1.0, "TIMEOUT": -0.5}

# ─── CONTAINER ──────────────────────────────────────────────────────────────
docker_proc    = None
container_name = "es_gazebo"

# ─── SPAWN POSE ──────────────────────────────────────────────────────────────
SPAWN_X, SPAWN_Y, SPAWN_Z = 0.0, 0.0, 0.0
SPAWN_QX, SPAWN_QY, SPAWN_QZ, SPAWN_QW = 0.0, 0.0, 1.0, 0.0

# ─── FUNZIONI ────────────────────────────────────────────────────────────────

def clip_theta(theta):
    return np.clip(theta, THETA_MIN, THETA_MAX)

def theta_to_json(theta):
    d = {name: float(val) for name, val in zip(PARAM_NAMES, theta)}
    d["s_max"]    = int(round(d["s_max"]))
    d["smooth_l"] = int(round(d["smooth_l"]))
    return json.dumps(d)

def yaw_to_quaternion(yaw):
    return 0.0, 0.0, np.sin(yaw/2), np.cos(yaw/2)

def read_spawn_pose():
    import xml.etree.ElementTree as ET
    world_path = os.path.join(PROJECT_DIR, "lrauv_gazebo_plugins/worlds/navigation_world.sdf")
    tree = ET.parse(world_path)
    root = tree.getroot()
    pose_text = root.find(".//include/pose").text
    parts = pose_text.split()
    x, y, z, yaw = float(parts[0]), float(parts[1]), float(parts[2]), float(parts[5])
    return x, y, z, yaw

def start_docker():
    global docker_proc
    subprocess.run(["docker", "rm", "-f", container_name], capture_output=True)

    render_gid = os.popen("getent group render | cut -d: -f3").read().strip()
    video_gid  = os.popen("getent group video | cut -d: -f3").read().strip()

    cmd = [
        "docker", "run", "--rm",
        "--name", container_name,
        "--device", "/dev/dri/card1",
        "--device", "/dev/dri/renderD128",
        "--group-add", render_gid,
        "--group-add", video_gid,
        "--env", f"DISPLAY={os.environ.get('DISPLAY', ':0')}",
        "--env", "XDG_RUNTIME_DIR=/tmp/runtime-developer",
        "--volume", "/tmp/.X11-unix:/tmp/.X11-unix",
        "--volume", f"{PROJECT_DIR}:/lrauv_ws/src/degree_project",
        "lrauv:harmonic",
        "bash", "-c",
        (
            "source /setup.sh && "
            "NEEDS_CMAKE=false && "
            "HYDRO_SO=/lrauv_ws/src/degree_project/docker_build/libHydrodynamicsPlugin.so && "
            "NAV_SO=/lrauv_ws/src/degree_project/docker_build/libNavigationPlugin.so && "
            "HYDRO_CC=/lrauv_ws/src/degree_project/lrauv_gazebo_plugins/src/HydrodynamicsPlugin.cc && "
            "NAV_CC=/lrauv_ws/src/degree_project/lrauv_gazebo_plugins/src/NavigationPlugin.cc && "
            "if [ ! -f $HYDRO_SO ] || [ $HYDRO_CC -nt $HYDRO_SO ]; then "
            "echo 'HydrodynamicsPlugin needs rebuild...' && NEEDS_CMAKE=true; fi && "
            "if [ ! -f $NAV_SO ] || [ $NAV_CC -nt $NAV_SO ]; then "
            "echo 'NavigationPlugin needs rebuild...' && NEEDS_CMAKE=true; fi && "
            "if [ $NEEDS_CMAKE = 'true' ]; then "
            "mkdir -p /lrauv_ws/src/degree_project/docker_build && "
            "cd /lrauv_ws/src/degree_project/docker_build && "
            "cmake ../lrauv_gazebo_plugins -DCMAKE_BUILD_TYPE=Release -Wno-dev && "
            "make -j4 HydrodynamicsPlugin && "
            "make -j4 NavigationPlugin && "
            "echo 'Build done!'; "
            "else echo 'Plugins up to date, skipping build...'; fi && "
            "echo 'Launching Gazebo...' && "
            "export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models && "
            "export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build && "
            "gz sim "
            "/lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world.sdf"
        )
    ]

    docker_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
    print("[ES] Docker avviato, attendo Gazebo...")
    time.sleep(30.0)
    print("[ES] Gazebo pronto!")

def stop_docker():
    global docker_proc
    subprocess.run(["docker", "rm", "-f", container_name], capture_output=True)
    if docker_proc:
        docker_proc.terminate()
        docker_proc = None
    print("[ES] Docker fermato.")

def publish_params(theta):
    params_json = theta_to_json(theta)
    cmd = [
        "docker", "exec", container_name, "bash", "-c",
        f"source /setup.sh && gz topic -t /es/vfh_params -m gz.msgs.StringMsg -p 'data: \"{params_json.replace(chr(34), chr(92)+chr(34))}\"'"
    ]
    subprocess.run(cmd, capture_output=True)

def pause_and_reset():
    cmd_pause = ["docker", "exec", container_name, "bash", "-c",
        "source /setup.sh && gz service -s /world/empty_environment/control "
        "--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
        "--req 'pause: true' --timeout 2000"]
    subprocess.run(cmd_pause, capture_output=True)
    time.sleep(0.5)

    cmd_set_pose = ["docker", "exec", container_name, "bash", "-c",
        f"source /setup.sh && gz service -s /world/empty_environment/set_pose "
        f"--reqtype gz.msgs.Pose --reptype gz.msgs.Boolean "
        f"--req 'name: \"tethys\" position: {{x: {SPAWN_X} y: {SPAWN_Y} z: {SPAWN_Z}}} orientation: {{x: 0 y: 0 z: 1 w: 0}}' "
        f"--timeout 2000"]
    subprocess.run(cmd_set_pose, capture_output=True)
    time.sleep(5.0)

def play():
    cmd_play = ["docker", "exec", container_name, "bash", "-c",
        "source /setup.sh && gz service -s /world/empty_environment/control "
        "--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
        "--req 'pause: false' --timeout 2000"]
    subprocess.run(cmd_play, capture_output=True)
    time.sleep(1.0)

def wait_for_result(episode_timeout=200):
    cmd = ["docker", "exec", container_name, "bash", "-c",
        f"source /setup.sh && gz topic -e -n 1 -t /es/episode_result"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=episode_timeout)
        output = result.stdout + result.stderr
        for line in output.splitlines():
            if "ARRIVED" in line:
                pause_and_reset()
                return REWARD["ARRIVED"]
            if "COLLISION" in line:
                pause_and_reset()
                return REWARD["COLLISION"]
            if "TIMEOUT" in line:
                pause_and_reset()
                return REWARD["TIMEOUT"]
        pause_and_reset()
        return REWARD["TIMEOUT"]
    except subprocess.TimeoutExpired:
        pause_and_reset()
        return REWARD["TIMEOUT"]

def run_episode(theta, episode_timeout=200):
    publish_params(theta)
    time.sleep(2.0)
    play()
    return wait_for_result(episode_timeout)

def save_results(gen, theta, mean_reward):
    os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
    header = not os.path.exists(RESULTS_FILE)
    with open(RESULTS_FILE, "a") as f:
        if header:
            f.write("gen,mean_reward," + ",".join(PARAM_NAMES) + "\n")
        vals = ",".join(f"{v:.6f}" for v in theta)
        f.write(f"{gen},{mean_reward:.4f},{vals}\n")

# ─── MAIN ES LOOP ────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("Evolution Strategies - VFH Parameter Optimization")
    print("=" * 60)
    print(f"Workers per gen: {N_WORKERS}, Generazioni: {N_GEN}, Sigma: {SIGMA}")
    print(f"Parametri iniziali: {dict(zip(PARAM_NAMES, THETA_INIT))}")
    print()

    theta = THETA_INIT.copy()
    best_theta = theta.copy()
    best_reward = -np.inf

    generate_world(PROJECT_DIR, seed=0)
    global SPAWN_X, SPAWN_Y, SPAWN_Z, SPAWN_QX, SPAWN_QY, SPAWN_QZ, SPAWN_QW
    SPAWN_X, SPAWN_Y, SPAWN_Z, syaw = read_spawn_pose()
    SPAWN_QX, SPAWN_QY, SPAWN_QZ, SPAWN_QW = yaw_to_quaternion(syaw)

    subprocess.run([
        "sed",
        "-e", "s/__THRESHOLD__/0.001/",
        "-e", "s/__S_MAX__/20/",
        "-e", "s/__SMOOTH_L__/5/",
        "-e", "s/__GAIN_STEER__/0.5/",
        "-e", "s/__GAIN_PITCH__/0.5/",
        "-e", "s/__RADIUS_ARRIVED__/5.0/",
        "-e", "s/__RADIUS_SLOWDOWN__/15.0/",
        TEMPLATE_FILE,
    ], stdout=open(MODEL_FILE, "w"))


    subprocess.run(["xhost", "+local:docker"])
    start_docker()

    try:
        for gen in range(N_GEN):
            # Genera world nuovo ad ogni generazione
            generate_world(PROJECT_DIR, seed=random.randint(0, 9999))
            SPAWN_X, SPAWN_Y, SPAWN_Z, SPAWN_QX, SPAWN_QY, SPAWN_QZ, SPAWN_QW
            SPAWN_X, SPAWN_Y, SPAWN_Z, syaw = read_spawn_pose()
            SPAWN_QX, SPAWN_QY, SPAWN_QZ, SPAWN_QW = yaw_to_quaternion(syaw)

            print(f"─── Generazione {gen+1}/{N_GEN} ───────────────────────────")

            epsilons = []
            for i in range(N_WORKERS // 2):
                eps = np.random.randn(len(theta))
                epsilons.append(eps)
                epsilons.append(-eps)

            rewards = []
            for i, eps in enumerate(epsilons):
                theta_i = clip_theta(theta + SIGMA * eps)
                print(f"  Worker {i+1}/{len(epsilons)}: ", end="", flush=True)
                r = run_episode(theta_i)
                rewards.append(r)
                label = {1.0: "ARRIVED ✓", -1.0: "COLLISION ✗", -0.5: "TIMEOUT ~"}
                print(label.get(r, f"{r:.2f}"))

            rewards = np.array(rewards)
            mean_reward = np.mean(rewards)

            ranked = np.argsort(np.argsort(rewards))
            shaped = (ranked / (len(ranked) - 1)) - 0.5

            grad = np.zeros_like(theta)
            for eps, s in zip(epsilons, shaped):
                grad += s * eps
            grad /= (len(epsilons) * SIGMA)
            theta = clip_theta(theta + ALPHA * grad)

            if mean_reward > best_reward:
                best_reward = mean_reward
                best_theta = theta.copy()

            print(f"  Mean reward: {mean_reward:.3f} | Best so far: {best_reward:.3f}")
            print(f"  Theta: {dict(zip(PARAM_NAMES, [round(v,4) for v in theta]))}")
            save_results(gen+1, theta, mean_reward)
            print()

    finally:
        stop_docker()

    print("=" * 60)
    print("OTTIMIZZAZIONE COMPLETATA")
    print(f"Best reward: {best_reward:.3f}")
    print("Best theta:")
    for name, val in zip(PARAM_NAMES, best_theta):
        print(f"  {name}: {val:.4f}")
    print(f"\nRisultati salvati in: {RESULTS_FILE}")

if __name__ == "__main__":
    main()