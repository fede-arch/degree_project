#!/usr/bin/env python3
"""
Evolution Strategies per ottimizzare i parametri VFH del NavigationPlugin.
Worker paralleli: ogni worker ha la sua cartella tethys_equipped con model.sdf dedicato.
"""

import random
import subprocess
import numpy as np
import time
import os
import sys
import json
import shutil
import concurrent.futures
import xml.etree.ElementTree as ET

from es_utils import *
sys.path.append(os.path.join(PROJECT_DIR, "lrauv_gazebo_plugins/scripts"))
from generate_world import generate_world

# ES IPER-PARAMETRI 
N_WORKERS    = 6       # deve essere pari (campionamento antitetico)
N_GEN        = 20
SIGMA        = 0.15    # ampiezza perturbazione (spazio normalizzato [0,1])
ALPHA        = 0.05    # learning rate
STARTUP_WAIT = 25.0    # secondi attesa boot Gazebo headless

# PATH
WORKERS_DIR  = os.path.join(PROJECT_DIR, "es/workers")

# BUILD
def build_plugins():
    plugin_so = os.path.join(PROJECT_DIR, "docker_build/libNavigationPlugin.so")
    src_dirs  = [
        os.path.join(PROJECT_DIR, "lrauv_gazebo_plugins/src"),
        os.path.join(PROJECT_DIR, "lrauv_gazebo_plugins/include"),
    ]

    needs_build = not os.path.exists(plugin_so)
    if not needs_build:
        so_mtime = os.path.getmtime(plugin_so)
        for d in src_dirs:
            for root, _, files in os.walk(d):
                for f in files:
                    if f.endswith(('.cc', '.hh', '.cpp', '.hpp')):
                        if os.path.getmtime(os.path.join(root, f)) > so_mtime:
                            print(f"[ES] Sorgente modificato: {f} → ricompilazione...")
                            needs_build = True
                            break
                if needs_build: break
            if needs_build: break

    if not needs_build:
        print("[ES] Plugin aggiornato, nessuna ricompilazione")
        return

    print("[ES] Build NavigationPlugin...")
    subprocess.run(["docker", "rm", "-f", "es_build"], capture_output=True)
    r = subprocess.run([
        "docker", "run", "--rm", "--name", "es_build",
        "--volume", f"{PROJECT_DIR}:/lrauv_ws/src/degree_project",
        "lrauv:harmonic", "bash", "-c",
        (
            "source /setup.sh && "
            "mkdir -p /lrauv_ws/src/degree_project/docker_build && "
            "cd /lrauv_ws/src/degree_project/docker_build && "
            "cmake ../lrauv_gazebo_plugins -DCMAKE_BUILD_TYPE=Release "
            "-Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS=ON 2>&1 | tail -3 && "
            "make -j4 NavigationPlugin 2>&1 | tail -5 && "
            "echo BUILD_OK"
        )
    ], capture_output=True, text=True)
    if "BUILD_OK" not in (r.stdout + r.stderr):
        print(r.stdout[-2000:])
        print(r.stderr[-2000:])
        raise RuntimeError("[ES] Build FALLITA")
    print("[ES] Build OK")

# WORKER
def run_worker(args):
    """Lancia un container Gazebo headless isolato e restituisce (reward, tag)."""
    worker_id, theta_real, goal_x, goal_y, goal_z, dist_init = args

    cname     = f"es_gazebo_{worker_id}"
    partition = f"es_part_{worker_id}"

    model_dir = os.path.join(WORKERS_DIR, f"worker_{worker_id}", "tethys_equipped")
    write_sdf(model_dir, dict(zip(PARAM_NAMES, theta_real)), 0, goal_x, goal_y, goal_z, R_MAX, RADIUS_ARRIVED)

    # GZ_SIM_RESOURCE_PATH: cerca prima nella cartella worker, poi in quella default
    resource_path = (
        f"/lrauv_ws/src/degree_project/es/workers/worker_{worker_id}:"
        f"/lrauv_ws/src/degree_project/lrauv_description/models"
    )

    subprocess.run(["docker", "rm", "-f", cname], capture_output=True)

    os.makedirs(RESULTS_DIR, exist_ok=True)
    log = open(os.path.join(RESULTS_DIR, f"gazebo_{worker_id}.log"), "w")

    proc = subprocess.Popen([
        "docker", "run", "--rm", "--name", cname,
        "--volume", f"{PROJECT_DIR}:/lrauv_ws/src/degree_project",
        "lrauv:harmonic", "bash", "-c",
        (
            f"source /setup.sh && "
            f"export GZ_PARTITION={partition} && "
            f"export GZ_SIM_RESOURCE_PATH={resource_path} && "
            f"export GZ_SIM_SYSTEM_PLUGIN_PATH="
            f"/lrauv_ws/src/degree_project/docker_build && "
            f"gz sim -s -r "
            f"/lrauv_ws/src/degree_project/lrauv_gazebo_plugins/worlds/navigation_world.sdf"
        )
    ], stdout=log, stderr=subprocess.STDOUT)

    time.sleep(STARTUP_WAIT)

    try:
        listener = subprocess.Popen([
            "docker", "exec", cname, "bash", "-c",
            f"source /setup.sh && export GZ_PARTITION={partition} && "
            f"gz topic -e -n 1 -t /es/episode_result"
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        try:
            out, _ = listener.communicate(timeout=360)
        except subprocess.TimeoutExpired:
            listener.kill()
            out = ""
    finally:
        subprocess.run(["docker", "rm", "-f", cname], capture_output=True)
        proc.terminate()
        log.close()

    result = parse_result(out)
    reward = shaped_reward(result, dist_init)
    return reward, result["tag"]

# MAIN
def main():
    print("  Evolution Strategies — VFH Parameter Optimization")
    print(f"  Workers/gen : {N_WORKERS}  |  Generazioni: {N_GEN}")
    print(f"  Sigma: {SIGMA}  |  Alpha: {ALPHA}")
    print(f"  Parametri  : {PARAM_NAMES}")
    print(f"  θ iniziale : {dict(zip(PARAM_NAMES, THETA_INIT))}")
    print()

    build_plugins()

    theta       = norm(THETA_INIT.copy())
    best_theta  = theta.copy()
    best_reward = -np.inf

    for gen in range(N_GEN):
        seed = random.randint(0, 99999)

        # Genera world (uguale per tutti i worker di questa generazione)
        goal_x, goal_y, goal_z = generate_world(
            PROJECT_DIR, seed=seed, face_goal=False)

        # Leggi spawn dal world generato
        tree = ET.parse(WORLD_PATH)
        pose_parts = tree.getroot().find(".//include/pose").text.split()
        sx, sy, sz = float(pose_parts[0]), float(pose_parts[1]), float(pose_parts[2])
        dist_init  = np.sqrt((goal_x-sx)**2 + (goal_y-sy)**2 + (goal_z-sz)**2)

        print(f"── Gen {gen+1}/{N_GEN} | seed={seed} | dist={dist_init:.1f}m")

        # Perturbazioni antitetiche
        epsilons = []
        for _ in range(N_WORKERS // 2):
            eps = np.random.randn(len(theta))
            epsilons.append(eps)
            epsilons.append(-eps)

        args = [
            (i,
             denorm(np.clip(theta + SIGMA * eps, 0.0, 1.0)),
             goal_x, goal_y, goal_z, dist_init)
            for i, eps in enumerate(epsilons)
        ]

        # Lancia worker in parallelo
        with concurrent.futures.ThreadPoolExecutor(max_workers=N_WORKERS) as ex:
            results = list(ex.map(run_worker, args))

        rewards = np.array([r for r, _ in results])
        tags    = [t for _, t in results]

        for i, (r, t) in enumerate(zip(rewards, tags)):
            print(f"  worker {i}: {str(t):10s}  reward={r:+.2f}")

        mean_reward = float(np.mean(rewards))

        # ES update con rank shaping
        if not np.all(rewards == rewards[0]):
            ranked = np.argsort(np.argsort(rewards)).astype(float)
            shaped = ranked / (len(ranked) - 1) - 0.5
            grad   = sum(s * eps for s, eps in zip(shaped, epsilons))
            grad  /= len(epsilons) * SIGMA
            theta  = np.clip(theta + ALPHA * grad, 0.0, 1.0)
        else:
            print("  [WARN] tutti reward identici, skip update")

        if mean_reward > best_reward:
            best_reward = mean_reward
            best_theta  = theta.copy()
            save_best(denorm(best_theta), best_reward)

        print(f"  Mean={mean_reward:+.3f} | Best={best_reward:+.3f}")
        print(f"  θ = {dict(zip(PARAM_NAMES, [round(v, 3) for v in denorm(theta)]))}")
        save_results(gen+1, denorm(theta), mean_reward, seed)
        print()

    # Fine
    print("OTTIMIZZAZIONE COMPLETATA")
    best = denorm(best_theta)
    print(f"Best reward: {best_reward:+.3f}")
    for name, val in zip(PARAM_NAMES, best):
        print(f"  {name}: {val:.4f}")

    final_path = os.path.join(RESULTS_DIR, "final_theta.json")
    with open(final_path, "w") as fh:
        json.dump({"theta": dict(zip(PARAM_NAMES, [float(v) for v in best]))},
                  fh, indent=2)
    print(f"\nSalvato in: {final_path}")
    print(f"Risultati : {RESULTS_FILE}")
    print(f"Migliore  : {BEST_FILE}")

if __name__ == "__main__":
    try:
        main()
    finally:
        # Cleanup
        for i in range(N_WORKERS):
            subprocess.run(["docker", "rm", "-f", f"es_gazebo_{i}"],
                           capture_output=True)
        if os.path.exists(WORKERS_DIR):
            shutil.rmtree(WORKERS_DIR)