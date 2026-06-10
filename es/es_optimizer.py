#!/usr/bin/env python3
"""
Evolution Strategies per ottimizzare i parametri VFH del NavigationPlugin.
"""

import random
import subprocess
import numpy as np
import time
import os
import sys
import json

DRONE_NS = "tethys_0"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "../lrauv_gazebo_plugins/scripts"))
from generate_world import generate_world

# PARAMETRI ES
N_WORKERS    = 6          # perturbazioni per generazione (sequenziali); pari per antitetico
N_GEN        = 15
SIGMA        = 0.2        # ampiezza esplorazione, in FRAZIONE del range di ogni parametro
ALPHA        = 0.05       # learning rate (spazio normalizzato)
WEIGHT_DECAY = 0.01       # NB: in spazio normalizzato tira verso THETA_MIN; metti 0 per disattivarlo
STARTUP_WAIT = 20.0       # attesa avvio Gazebo (build gia' fatto, basta il boot)

# PARAMETRI VFH
PARAM_NAMES = ["threshold","s_max","smooth_l","gain_steer","gain_pitch","radius_slowdown"]

THETA_MIN   = np.array([0.0001, 5.0, 1.0, 0.1, 0.1,  8.0])
THETA_MAX   = np.array([0.01,  40.0, 15.0, 2.0, 2.0, 30.0])
THETA_INIT = np.array([
    random.uniform(THETA_MIN[i], THETA_MAX[i]) for i in range(len(THETA_MIN))
])
SCALE       = THETA_MAX - THETA_MIN

def denorm(theta_n):   # [0,1]^N -> valori reali
    return THETA_MIN + np.clip(theta_n, 0.0, 1.0) * SCALE

def norm(theta):       # valori reali -> [0,1]^N
    return (theta - THETA_MIN) / SCALE

# PATH
PROJECT_DIR  = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
RESULTS_FILE = os.path.join(PROJECT_DIR, "es/results/es_results.csv")
BEST_FILE    = os.path.join(PROJECT_DIR, "es/results/best_theta.json")
LOG_FILE     = os.path.join(PROJECT_DIR, "es/results/gazebo_last.log")

# CONTAINER 
docker_proc    = None
log_handle     = None
container_name = "es_gazebo"

# SPAWN POSE
SPAWN_X, SPAWN_Y, SPAWN_Z = 0.0, 0.0, 0.0
SPAWN_QX, SPAWN_QY, SPAWN_QZ, SPAWN_QW = 0.0, 0.0, 1.0, 0.0

# FUNZIONI

def theta_to_json(theta_real):
    d = {name: float(val) for name, val in zip(PARAM_NAMES, theta_real)}
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

def build_plugins():
    """Compila i plugin UNA SOLA VOLTA. Il .so finisce in docker_build (volume montato),
    quindi i container di lancio successivi lo riusano senza ricompilare."""
    print("[ES] Build plugin (una sola volta)...")
    subprocess.run(["docker", "rm", "-f", "es_build"], capture_output=True)
    cmd = [
        "docker", "run", "--rm", "--name", "es_build",
        "--volume", f"{PROJECT_DIR}:/lrauv_ws/src/degree_project",
        "lrauv:harmonic", "bash", "-c",
        (
            "source /setup.sh && "
            "mkdir -p /lrauv_ws/src/degree_project/docker_build && "
            "cd /lrauv_ws/src/degree_project/docker_build && "
            "cmake ../lrauv_gazebo_plugins -DCMAKE_BUILD_TYPE=Release -Wno-dev && "
            "make -j4 HydrodynamicsPlugin && "
            "make -j4 NavigationPlugin && "
            "echo BUILD_OK"
        )
    ]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if "BUILD_OK" not in (r.stdout + r.stderr):
        print(r.stdout)
        print(r.stderr)
        raise RuntimeError("[ES] Build dei plugin FALLITA")
    print("[ES] Build OK")

def start_docker():
    """Lancia Gazebo headless (paused). Niente build qui: gia' fatto da build_plugins()."""
    global docker_proc, log_handle
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
        "--volume", f"{PROJECT_DIR}:/lrauv_ws/src/degree_project",
        "lrauv:harmonic",
        "bash", "-c",
        (
            "source /setup.sh && "
            "export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models && "
            "export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build && "
            "gz sim --headless-rendering -s "
            "/lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world.sdf"
        )
    ]

    os.makedirs(os.path.dirname(LOG_FILE), exist_ok=True)
    log_handle = open(LOG_FILE, "w")
    docker_proc = subprocess.Popen(cmd, stdout=log_handle, stderr=subprocess.STDOUT, text=True)
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

def publish_params(theta_real):
    params_json = theta_to_json(theta_real)
    cmd = [
        "docker", "exec", container_name, "bash", "-c",
        f"source /setup.sh && gz topic -t /{DRONE_NS}/es/vfh_params -m gz.msgs.StringMsg "
        f"-p 'data: \"{params_json.replace(chr(34), chr(92)+chr(34))}\"'"
    ]
    subprocess.run(cmd, capture_output=True)

def play():
    cmd_play = ["docker", "exec", container_name, "bash", "-c",
        "source /setup.sh && gz service -s /world/empty_environment/control "
        "--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
        "--req 'pause: false' --timeout 2000"]
    subprocess.run(cmd_play, capture_output=True)

def parse_result(output):
    """Estrae i campi dal risultato del NavigationPlugin.
    Formato:  <TAG>;t=<iter>;dist=<m>;path=<m>;clr=<m>  (es. 'data: \"ARRIVED;t=...\"')."""
    for line in output.splitlines():
        for tag in ("ARRIVED", "COLLISION", "TIMEOUT"):
            i = line.find(tag)
            if i != -1:
                payload = line[i:].strip().strip('"')
                f = {"tag": tag}
                for kv in payload.split(";")[1:]:
                    if "=" in kv:
                        k, v = kv.split("=", 1)
                        try:
                            f[k] = float(v.strip().strip('"'))
                        except ValueError:
                            pass
                return f
    return None

def shaped_reward(f, dist_iniziale):
    if f is None:
        return -0.5
    dist = f.get("dist", dist_iniziale)
    path = f.get("path", 0.0)
    t    = f.get("t", 0.0)
    head = f.get("head", 180.0)  # gradi assoluti, 0 = muso verso goal

    progress     = max(0.0, min(1.0, (dist_iniziale - dist) / max(dist_iniziale, 1e-6)))
    heading_bonus = 1.0 - (head / 180.0)  # 1.0 = perfettamente allineato

    if f["tag"] == "ARRIVED":
        eff   = dist_iniziale / max(path, dist_iniziale)
        speed = 1.0 / (1.0 + (t / 1000.0) / 60.0)
        return 2.0 + 0.5 * eff + 0.5 * speed

    if f["tag"] == "COLLISION":
        return -1.0 + 0.5 * progress

    # TIMEOUT: mix progresso + allineamento
    return 0.7 * progress + 0.3 * heading_bonus

def run_episode(theta_real, dist_iniziale, episode_timeout=320):
    publish_params(theta_real)
    time.sleep(2.0)
    listener = subprocess.Popen(
        ["docker", "exec", container_name, "bash", "-c",
         f"source /setup.sh && gz topic -e -n 1 -t /{DRONE_NS}/es/episode_result"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    play()
    try:
        out, _ = listener.communicate(timeout=episode_timeout)
    except subprocess.TimeoutExpired:
        listener.kill()
        out = ""
    return shaped_reward(parse_result(out), dist_iniziale)

def save_results(gen, theta_real, mean_reward, seed):
    os.makedirs(os.path.dirname(RESULTS_FILE), exist_ok=True)
    header = not os.path.exists(RESULTS_FILE)
    with open(RESULTS_FILE, "a") as fh:
        if header:
            fh.write("gen,seed,mean_reward," + ",".join(PARAM_NAMES) + "\n")
        vals = ",".join(f"{v:.6f}" for v in theta_real)
        fh.write(f"{gen},{seed},{mean_reward:.4f},{vals}\n")

def save_best(theta_real, reward):
    os.makedirs(os.path.dirname(BEST_FILE), exist_ok=True)
    with open(BEST_FILE, "w") as fh:
        json.dump({
            "reward": float(reward),
            "theta": {name: float(val) for name, val in zip(PARAM_NAMES, theta_real)}
        }, fh, indent=2)

# MAIN LOOP
def main():
    print("=" * 60)
    print("Evolution Strategies - VFH Parameter Optimization")
    print("=" * 60)
    print(f"Workers/gen: {N_WORKERS}, Generazioni: {N_GEN}, Sigma: {SIGMA}, dims: {len(PARAM_NAMES)}")
    print(f"Parametri iniziali: {dict(zip(PARAM_NAMES, THETA_INIT))}")
    print()
    global SPAWN_X, SPAWN_Y, SPAWN_Z, SPAWN_QX, SPAWN_QY, SPAWN_QZ, SPAWN_QW

    build_plugins()                       
    theta = norm(THETA_INIT.copy())       
    best_theta = theta.copy()
    best_reward = -np.inf

    for gen in range(N_GEN):
        seed = random.randint(0, 9999)
        goal_x, goal_y, goal_z = generate_world(PROJECT_DIR, seed=seed)
        SPAWN_X, SPAWN_Y, SPAWN_Z, syaw = read_spawn_pose()
        SPAWN_QX, SPAWN_QY, SPAWN_QZ, SPAWN_QW = yaw_to_quaternion(syaw)
        dist_iniziale = np.sqrt((goal_x-SPAWN_X)**2 + (goal_y-SPAWN_Y)**2 + (goal_z-SPAWN_Z)**2)

        print(f"─── Generazione {gen+1}/{N_GEN} ───────────────────────────")

        epsilons = []
        for _ in range(N_WORKERS // 2):
            eps = np.random.randn(len(theta))
            epsilons.append(eps)
            epsilons.append(-eps)

        rewards = []
        for i, eps in enumerate(epsilons):
            theta_i = np.clip(theta + SIGMA * eps, 0.0, 1.0)   
            print(f"  worker {i+1}/{len(epsilons)}: ", end="", flush=True)

            start_docker()
            try:
                r = run_episode(denorm(theta_i), dist_iniziale) 
            finally:
                stop_docker()

            rewards.append(r)
            print(f"reward = {r:+.3f}")

        rewards = np.array(rewards)
        mean_reward = float(np.mean(rewards))

        if np.all(rewards == rewards[0]):
            print("  [WARN] tutti i reward identici, skip update "
                  "(controlla: OnParams ricevuto? rocce viste dal lidar?)")
        else:
            ranked = np.argsort(np.argsort(rewards))
            shaped = (ranked / (len(ranked) - 1)) - 0.5

            grad = np.zeros_like(theta)
            for eps, s in zip(epsilons, shaped):
                grad += s * eps
            grad /= (len(epsilons) * SIGMA)
            theta = np.clip((1.0 - WEIGHT_DECAY) * theta + ALPHA * grad, 0.0, 1.0)

        if mean_reward > best_reward:
            best_reward = mean_reward
            best_theta = theta.copy()
            save_best(denorm(best_theta), best_reward)

        print(f"  Mean reward: {mean_reward:.3f} | Best so far: {best_reward:.3f}")
        print(f"  Theta(real): {dict(zip(PARAM_NAMES, [round(v,4) for v in denorm(theta)]))}")
        save_results(gen+1, denorm(theta), mean_reward, seed)
        print()

    print("=" * 60)
    print("OTTIMIZZAZIONE COMPLETATA")
    print(f"Best reward: {best_reward:.3f}")
    print("Best theta (reali):")
    for name, val in zip(PARAM_NAMES, denorm(best_theta)):
        print(f"  {name}: {val:.4f}")
    print(f"\nRisultati: {RESULTS_FILE}\nMigliore: {BEST_FILE}")

if __name__ == "__main__":
    try:
        main()
    finally:
        stop_docker() 