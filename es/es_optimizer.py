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
import concurrent.futures

DRONE_NS = "tethys_0"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "../lrauv_gazebo_plugins/scripts"))
from generate_world import generate_world

# PARAMETRI ES
N_WORKERS    = 6          # perturbazioni per generazione (sequenziali); pari per antitetico
N_GEN        = 15
SIGMA        = 0.20        # ampiezza esplorazione, in FRAZIONE del range di ogni parametro
ALPHA        = 0.05       # learning rate (spazio normalizzato)
STARTUP_WAIT = 20.0       # attesa avvio Gazebo (build gia' fatto, basta il boot)

# PARAMETRI VFH
PARAM_NAMES = ["s_max", "smooth_l", "gain_steer", "gain_pitch"]


THETA_MIN   = np.array([5.0,  1.0, 0.3, 0.20])
THETA_MAX   = np.array([40.0, 6.0, 1.5, 0.71])
THETA_INIT = np.array([
    random.uniform(THETA_MIN[i], THETA_MAX[i]) for i in range(len(THETA_MIN))
])


SCALE = THETA_MAX - THETA_MIN

def denorm(theta_n):   # [0,1]^N -> valori reali
    return THETA_MIN + np.clip(theta_n, 0.0, 1.0) * SCALE

def norm(theta):       # valori reali -> [0,1]^N
    return (theta - THETA_MIN) / SCALE

# PATH
PROJECT_DIR  = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
RESULTS_FILE = os.path.join(PROJECT_DIR, "es/results/es_results.csv")
BEST_FILE    = os.path.join(PROJECT_DIR, "es/results/best_theta.json")
LOG_FILE     = os.path.join(PROJECT_DIR, "es/results/gazebo_last.log")

# FUNZIONI

def theta_to_json(theta_real):
    d = {name: float(val) for name, val in zip(PARAM_NAMES, theta_real)}
    d["threshold"]       = 0.001
    d["radius_slowdown"] = 15.0
    d["s_max"]           = int(round(d["s_max"]))
    d["smooth_l"]        = int(round(d["smooth_l"]))
    return json.dumps(d)

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

    progress = max(0.0, min(1.0, (dist_iniziale - dist) / max(dist_iniziale, 1e-6)))

    if f["tag"] == "ARRIVED":
        eff   = dist_iniziale / max(path, dist_iniziale)  # 1.0 = dritto
        speed = 1.0 / (1.0 + (t / 1000.0) / 60.0)       # 1.0 = veloce
        return 2.0 + 0.5 * eff + 0.5 * speed             # range [2, 3]

    if f["tag"] == "COLLISION":
        return -1.0 + 0.3 * progress   

    # TIMEOUT
    return -0.5 + 0.5 * progress      

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

def run_worker_parallel(args):
    """Esegue un singolo worker in un container isolato."""
    worker_id, theta_i, dist_iniziale = args
    cname     = f"es_gazebo_{worker_id}"
    partition = f"part{worker_id}"
    ns        = DRONE_NS  # stesso namespace, partizioni diverse

    # Avvia container con partizione isolata
    render_gid = os.popen("getent group render | cut -d: -f3").read().strip()
    video_gid  = os.popen("getent group video | cut -d: -f3").read().strip()
    subprocess.run(["docker", "rm", "-f", cname], capture_output=True)

    cmd = [
        "docker", "run", "--rm", "--name", cname,
        "--device", "/dev/dri/card1",
        "--device", "/dev/dri/renderD128",
        "--group-add", render_gid,
        "--group-add", video_gid,
        "--volume", f"{PROJECT_DIR}:/lrauv_ws/src/degree_project",
        "lrauv:harmonic", "bash", "-c",
        (
            f"source /setup.sh && "
            f"export GZ_PARTITION={partition} && "
            f"export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models && "
            f"export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build && "
            f"gz sim --headless-rendering -s "
            f"/lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world.sdf"
        )
    ]
    log = open(f"{LOG_FILE}.{worker_id}", "w")
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, text=True)
    time.sleep(STARTUP_WAIT)

    try:
        # Pubblica params
        params_json = theta_to_json(theta_i)
        subprocess.run([
            "docker", "exec", cname, "bash", "-c",
            f"source /setup.sh && export GZ_PARTITION={partition} && "
            f"gz topic -t /{ns}/es/vfh_params -m gz.msgs.StringMsg "
            f"-p 'data: \"{params_json.replace(chr(34), chr(92)+chr(34))}\"'"
        ], capture_output=True)
        time.sleep(2.0)

        # Listener
        listener = subprocess.Popen([
            "docker", "exec", cname, "bash", "-c",
            f"source /setup.sh && export GZ_PARTITION={partition} && "
            f"gz topic -e -n 1 -t /{ns}/es/episode_result"
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        # Play
        subprocess.run([
            "docker", "exec", cname, "bash", "-c",
            f"source /setup.sh && export GZ_PARTITION={partition} && "
            f"gz service -s /world/empty_environment/control "
            f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
            f"--req 'pause: false' --timeout 2000"
        ], capture_output=True)

        try:
            out, _ = listener.communicate(timeout=320)
        except subprocess.TimeoutExpired:
            listener.kill()
            out = ""
    finally:
        subprocess.run(["docker", "rm", "-f", cname], capture_output=True)
        proc.terminate()
        log.close()

    return shaped_reward(parse_result(out), dist_iniziale)

# MAIN LOOP
def main():
    print("Evolution Strategies - VFH Parameter Optimization")
    print("=" * 60)
    print(f"Workers/gen: {N_WORKERS}, Generazioni: {N_GEN}, Sigma: {SIGMA}, dims: {len(PARAM_NAMES)}")
    print(f"Parametri iniziali: {dict(zip(PARAM_NAMES, THETA_INIT))}")
    print()

    build_plugins()                       
    theta = norm(THETA_INIT.copy())       
    best_theta = theta.copy()
    best_reward = -np.inf

    for gen in range(N_GEN):
        seed = random.randint(0, 9999)
        goal_x, goal_y, goal_z = generate_world(PROJECT_DIR, seed=seed)
        SPAWN_X, SPAWN_Y, SPAWN_Z, _ = read_spawn_pose()
        dist_iniziale = np.sqrt((goal_x-SPAWN_X)**2 + (goal_y-SPAWN_Y)**2 + (goal_z-SPAWN_Z)**2)

        world_path = os.path.join(PROJECT_DIR, "lrauv_gazebo_plugins/worlds/navigation_world.sdf")
        with open(world_path) as wf:
            num_rocks = wf.read().count('<uri>falling rock')

        print(f"─── Generazione {gen+1}/{N_GEN} ")

        epsilons = []
        for _ in range(N_WORKERS // 2):
            eps = np.random.randn(len(theta))
            epsilons.append(eps)
            epsilons.append(-eps)

        args = [
            (i, denorm(np.clip(theta + SIGMA * eps, 0.0, 1.0)), dist_iniziale)
            for i, eps in enumerate(epsilons)
        ]
        print(f"  Lancio {len(args)} worker in parallelo...")
        with concurrent.futures.ThreadPoolExecutor(max_workers=N_WORKERS) as ex:
            rewards_list = list(ex.map(run_worker_parallel, args))
        for i, r in enumerate(rewards_list):
            print(f"  worker {i+1}/{len(rewards_list)}: reward = {r:+.6f}")

        rewards = np.array(rewards_list) 

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
            theta = np.clip(theta + ALPHA * grad, 0.0, 1.0)

        if mean_reward > best_reward and num_rocks > 20:
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

    final_file = os.path.join(PROJECT_DIR, "es/results/final_theta.json")
    with open(final_file, "w") as fh:
        json.dump({
            "theta": {name: float(val) for name, val in zip(PARAM_NAMES, denorm(theta))}
        }, fh, indent=2)
    print(f"\nTheta finale (più evoluto): {final_file}")    
    print(f"\nRisultati: {RESULTS_FILE}\nMigliore: {BEST_FILE}")

if __name__ == "__main__":
    try:
        main()
    finally:
        for i in range(N_WORKERS):
            subprocess.run(["docker", "rm", "-f", f"es_gazebo_{i}"],
                           capture_output=True)