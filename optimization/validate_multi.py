#!/usr/bin/env python3
"""
Validazione multi-drone VFH su seed mai visti durante il training.
Testa con N_DRONES droni su N_SEEDS seed diversi e raccoglie metriche.
"""

import json, os, sys, shutil, subprocess, time
import numpy as np
import concurrent.futures

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, os.path.join(PROJECT_DIR, "lrauv_gazebo_plugins/scripts"))
from generate_world_multi import generate_world_multi

# ── PATHS ─────────────────────────────────────────────────────────
BEST_FILE    = os.path.join(PROJECT_DIR, "es/results/best_theta.json")
RESULTS_DIR  = os.path.join(PROJECT_DIR, "es/results")
WORKERS_DIR  = os.path.join(PROJECT_DIR, "es/val_multi_workers")
TMPL_PATH    = os.path.join(PROJECT_DIR,
    "lrauv_description/models/tethys_equipped/model.sdf.template")

# ── PARAMETRI FISSI ───────────────────────────────────────────────
R_MAX          = 100.0
R_MIN          = 2.5
MAX_FIN_ANGLE  = 0.15
RADIUS_ARRIVED = 4.0
MAX_ITERATIONS = 200000

# ── CONFIGURAZIONE VALIDAZIONE ────────────────────────────────────
STARTUP_WAIT = 40.0

SEEDS = [1111, 2222, 3333]

N_DRONES_LIST = [30, 40]

# ── SCRITTURA SDF WORKER ──────────────────────────────────────────
def write_drone_sdf(run_id, drone_id, theta, goal_x, goal_y, goal_z):
    model_name = f"tethys_equipped_{drone_id}"
    model_dir  = os.path.join(WORKERS_DIR, f"run_{run_id}", model_name)
    os.makedirs(model_dir, exist_ok=True)

    src_config = os.path.join(PROJECT_DIR,
        "lrauv_description/models/tethys_equipped/model.config")
    shutil.copy(src_config, os.path.join(model_dir, "model.config"))

    r_active = R_MAX * 0.75

    with open(TMPL_PATH) as f:
        content = f.read()

    content = content.replace("__DRONE_ID__",         str(drone_id))
    content = content.replace("__GOAL_X__",           f"{goal_x:.2f}")
    content = content.replace("__GOAL_Y__",           f"{goal_y:.2f}")
    content = content.replace("__GOAL_Z__",           f"{goal_z:.2f}")
    content = content.replace("__GAIN_STEER__",       f"{theta['gain_steer']:.4f}")
    content = content.replace("__GAIN_PITCH__",       f"{theta['gain_pitch']:.4f}")
    content = content.replace("__VALLEY_THRESHOLD__", f"{theta['valley_threshold']:.4f}")
    content = content.replace("__GRID_DECAY__",       f"{theta['grid_decay']:.4f}")
    content = content.replace("__MAGNITUDE_A__",      f"{theta['magnitude_a']:.4f}")
    content = content.replace("__SMOOTH_L__",         f"{int(round(theta['smooth_l']))}")
    content = content.replace("__SAFETY_WINDOW__",    f"{int(round(theta['safety_window']))}")
    content = content.replace("__MAX_FIN_ANGLE__",    f"{MAX_FIN_ANGLE}")
    content = content.replace("__RADIUS_ARRIVED__",   f"{RADIUS_ARRIVED}")
    content = content.replace("__MAX_ITERATIONS__",   f"{MAX_ITERATIONS}")
    content = content.replace("__R_MAX__",            f"{R_MAX}")
    content = content.replace("__R_MIN__",            f"{R_MIN}")
    content = content.replace("__R_ACTIVE__",         f"{r_active}")

    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(content)

# ── PARSE RESULT ──────────────────────────────────────────────────
def parse_result(output):
    for line in output.splitlines():
        low = line.lower()
        for tag in ("arrived", "collision", "timeout"):
            if tag in low:
                result = {"tag": tag, "path": None, "t": None, "dist": None}
                try:
                    if "path=" in low:
                        result["path"] = float(low.split("path=")[1].split(";")[0])
                    if ";t=" in low:
                        result["t"] = float(low.split(";t=")[1].split(";")[0].strip('"'))
                    if "dist=" in low:
                        result["dist"] = float(low.split("dist=")[1].split(";")[0].strip('"'))
                except:
                    pass
                return result
    return {"tag": None, "path": None, "t": None, "dist": None}

# ── RUN SINGOLA SIMULAZIONE MULTI-DRONE ──────────────────────────
def run_simulation(args):
    run_id, theta, seed, n_drones, world_path, goals = args

    cname     = f"val_multi_{run_id}"
    partition = f"val_multi_{run_id}"

    # Scrivi SDF per ogni drone
    for i, (gx, gy, gz) in enumerate(goals):
        write_drone_sdf(run_id, i, theta, gx, gy, gz)

    # Costruisci resource path
    run_dir = os.path.join(WORKERS_DIR, f"run_{run_id}")
    container_run  = f"/lrauv_ws/src/degree_project/es/val_multi_workers/run_{run_id}"
    resource_path  = f"{container_run}:/lrauv_ws/src/degree_project/lrauv_description/models"

    world_container = world_path.replace(PROJECT_DIR, "/lrauv_ws/src/degree_project")

    subprocess.run(["docker", "rm", "-f", cname], capture_output=True)
    os.makedirs(RESULTS_DIR, exist_ok=True)
    log = open(os.path.join(RESULTS_DIR, f"val_multi_{run_id}.log"), "w")

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
            f"gz sim -s -r {world_container}"
        )
    ], stdout=log, stderr=subprocess.STDOUT)

    time.sleep(STARTUP_WAIT)

    # Ascolta topic di ogni drone
    drone_results = []
    listeners = []
    for i in range(n_drones):
        listener = subprocess.Popen([
            "docker", "exec", cname, "bash", "-c",
            f"source /setup.sh && export GZ_PARTITION={partition} && "
            f"gz topic -e -n 1 -t /tethys_{i}/es/episode_result"
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        listeners.append(listener)

    # Poi raccoglie i risultati
    drone_results = []
    for i, listener in enumerate(listeners):
        try:
            out, _ = listener.communicate(timeout=360)
        except subprocess.TimeoutExpired:
            listener.kill()
            out = ""
        drone_results.append(parse_result(out))

    subprocess.run(["docker", "rm", "-f", cname], capture_output=True)
    proc.terminate()
    log.close()
    try: os.remove(world_path)
    except: pass

    # Stampa risultati
    arrived_count = sum(1 for r in drone_results if r["tag"] == "arrived")
    print(f"  seed={seed:5d} | n={n_drones} | "
          f"arrived={arrived_count}/{n_drones} | "
          f"rate={100*arrived_count/n_drones:.0f}%", flush=True)

    return drone_results


# ── MAIN ──────────────────────────────────────────────────────────
def main():
    with open(BEST_FILE) as f:
        best = json.load(f)
    theta = best["theta"]

    print("  VFH Multi-Drone Validation")
    print("=" * 60)
    print(f"  Best reward ES : {best['reward']:.4f}")
    print(f"  N droni testati: {N_DRONES_LIST}")
    print(f"  Seed per config: {len(SEEDS)}")
    print()

    all_results = {}

    for n_drones in N_DRONES_LIST:
        print(f"\n── N_DRONES = {n_drones} ──────────────────────────────")

        sim_args = []
        run_id = 0
        world_files = []

        for seed in SEEDS:
            world_path = os.path.join(PROJECT_DIR,
                f"lrauv_gazebo_plugins/worlds/val_multi_{run_id}.sdf")
            
            spawns, goals = generate_world_multi(
                PROJECT_DIR, seed=seed, face_goal=False, 
                n_drones=n_drones, output_path=world_path)
            
            world_files.append(world_path)

            sim_args.append((run_id, theta, seed, n_drones, world_path, goals))
            run_id += 1

        # Esegui simulazioni una alla volta (ogni sim ha già N droni)
        results_for_n = []
        for args in sim_args:
            drone_results = run_simulation(args)
            _, _, seed, _, _, goals = args
            results_for_n.append({
                "seed":          seed,
                "n_drones":      n_drones,
                "drone_results": drone_results,
                "arrived":       sum(1 for r in drone_results if r["tag"] == "arrived"),
                "collision":     sum(1 for r in drone_results if r["tag"] == "collision"),
                "timeout":       sum(1 for r in drone_results if r["tag"] == "timeout"),
                "none":          sum(1 for r in drone_results if r["tag"] is None),
            })

        # Statistiche per questo N
        total_drones  = n_drones * len(SEEDS)
        total_arrived = sum(r["arrived"] for r in results_for_n)
        total_collision = sum(r["collision"] for r in results_for_n)
        total_timeout = sum(r["timeout"] for r in results_for_n)

        print(f"\n  Risultati N={n_drones} ({len(SEEDS)} seed × {n_drones} droni = {total_drones} episodi):")
        print(f"    Arrived  : {total_arrived:3d}/{total_drones}  ({100*total_arrived/total_drones:.0f}%)")
        print(f"    Collision: {total_collision:3d}/{total_drones}  ({100*total_collision/total_drones:.0f}%)")
        print(f"    Timeout  : {total_timeout:3d}/{total_drones}  ({100*total_timeout/total_drones:.0f}%)")

        all_results[str(n_drones)] = {
            "arrived_rate": total_arrived / total_drones,
            "collision_rate": total_collision / total_drones,
            "timeout_rate": total_timeout / total_drones,
            "details": results_for_n
        }

    # Salva
    out_path = os.path.join(RESULTS_DIR, "validation_multi.json")
    with open(out_path, "w") as f:
        json.dump({
            "theta":   theta,
            "seeds":   SEEDS,
            "results": all_results
        }, f, indent=2)

    print(f"\n  Salvato: {out_path}")

    # Riepilogo finale
    print(f"\n{'='*60}")
    print(f"  RIEPILOGO")
    print(f"{'='*60}")
    print(f"  {'N droni':10s} | {'Arrived':10s} | {'Collision':10s} | {'Timeout':10s}")
    print(f"  {'-'*50}")
    for n_drones in N_DRONES_LIST:
        r = all_results[str(n_drones)]
        print(f"  {n_drones:<10d} | "
              f"{100*r['arrived_rate']:.0f}%{'':<7} | "
              f"{100*r['collision_rate']:.0f}%{'':<7} | "
              f"{100*r['timeout_rate']:.0f}%")


if __name__ == "__main__":
    try:
        main()
    finally:
        if os.path.exists(WORKERS_DIR):
            shutil.rmtree(WORKERS_DIR)
        for f in os.listdir(os.path.join(PROJECT_DIR,
                "lrauv_gazebo_plugins/worlds")):
            if f.startswith("val_multi_"):
                try:
                    os.remove(os.path.join(PROJECT_DIR,
                        "lrauv_gazebo_plugins/worlds", f))
                except:
                    pass