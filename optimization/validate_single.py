#!/usr/bin/env python3
"""
Validazione parametri VFH su seed mai visti durante il training.
Usa i parametri da best_theta.json e li testa su N_SEEDS seed diversi.
"""

import json, os, sys, shutil, subprocess, time
import numpy as np
import concurrent.futures
import xml.etree.ElementTree as ET
import argparse

DEFAULT_THETA = {
    "gain_steer":       0.5,
    "gain_pitch":       0.5,
    "valley_threshold": 50.0,
    "grid_decay":       0.97,
    "magnitude_a":      15.0,
    "smooth_l":         5,
    "safety_window":    3
}

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
sys.path.insert(0, os.path.join(PROJECT_DIR, "lrauv_gazebo_plugins/scripts"))
from generate_world import generate_world

# ── PATHS ─────────────────────────────────────────────────────────
BEST_FILE    = os.path.join(PROJECT_DIR, "es/results/best_theta.json")
RESULTS_DIR  = os.path.join(PROJECT_DIR, "es/results")
WORKERS_DIR  = os.path.join(PROJECT_DIR, "es/val_workers")
TMPL_PATH    = os.path.join(PROJECT_DIR,
    "lrauv_description/models/tethys_equipped/model.sdf.template")
WORLD_PATH   = os.path.join(PROJECT_DIR,
    "lrauv_gazebo_plugins/worlds/navigation_world.sdf")

# ── PARAMETRI FISSI ───────────────────────────────────────────────
R_MAX          = 100.0
R_MIN          = 2.5
MAX_FIN_ANGLE  = 0.15
RADIUS_ARRIVED = 10.0
MAX_ITERATIONS = 200000

# ── CONFIGURAZIONE VALIDAZIONE ────────────────────────────────────
N_PARALLEL   = 6
STARTUP_WAIT = 25.0

SEEDS = [
    1111, 2222, 3333, 4444, 5555,
    6666, 7777, 8888, 9999, 1234,
    5678, 9012, 3456, 7890, 2468,
    1357, 9753, 8642, 7531, 6420,
    1122, 3344, 5566, 7788, 9900,
    1010, 2020, 3030, 4040, 5050
]

# ── SCRITTURA SDF WORKER ──────────────────────────────────────────
def write_worker_sdf(worker_id, theta, goal_x, goal_y, goal_z):
    worker_dir = os.path.join(WORKERS_DIR, f"worker_{worker_id}", "tethys_equipped")
    os.makedirs(worker_dir, exist_ok=True)

    src_config = os.path.join(PROJECT_DIR,
        "lrauv_description/models/tethys_equipped/model.config")
    shutil.copy(src_config, os.path.join(worker_dir, "model.config"))

    r_active = R_MAX * 0.75

    with open(TMPL_PATH) as f:
        content = f.read()

    content = content.replace("__DRONE_ID__",         "0")
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

    with open(os.path.join(worker_dir, "model.sdf"), "w") as f:
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

# ── WORKER ────────────────────────────────────────────────────────
def run_worker(args):
    worker_id, theta, seed, goal_x, goal_y, goal_z, dist_init, world_dst = args

    cname     = f"val_gazebo_{worker_id}"
    partition = f"val_part_{worker_id}"

    write_worker_sdf(worker_id, theta, goal_x, goal_y, goal_z)

    resource_path = (
        f"/lrauv_ws/src/degree_project/es/val_workers/worker_{worker_id}:"
        f"/lrauv_ws/src/degree_project/lrauv_description/models"
    )

    world_container = world_dst.replace(PROJECT_DIR, "/lrauv_ws/src/degree_project")

    subprocess.run(["docker", "rm", "-f", cname], capture_output=True)
    os.makedirs(RESULTS_DIR, exist_ok=True)
    log = open(os.path.join(RESULTS_DIR, f"val_{worker_id}.log"), "w")

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

    try:
        listener = subprocess.Popen([
            "docker", "exec", cname, "bash", "-c",
            f"source /setup.sh && export GZ_PARTITION={partition} && "
            f"gz topic -e -n 1 -t /tethys_0/es/episode_result"
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
        try: os.remove(world_dst)
        except: pass

    result = parse_result(out)
    tag = result["tag"] or "none"
    t   = f"{result['t']:.1f}s" if result["t"] else "N/A"
    print(f"  seed={seed:5d} | {tag:9s} | t={t}", flush=True)
    return result

# ── MAIN ──────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--default", action="store_true",
                        help="Usa parametri default invece di best_theta.json")
    cli = parser.parse_args()

    if cli.default:
        theta = DEFAULT_THETA
        best_reward = 0.0
        mode = "DEFAULT"
    else:
        with open(BEST_FILE) as f:
            best = json.load(f)
        theta = best["theta"]
        best_reward = best["reward"]
        mode = "ES"

    print(f"  VFH Validation — modalità: {mode}")
    print("=" * 60)
    if not cli.default:
        print(f"  Best reward ES : {best_reward:.4f}")
    for k, v in theta.items():
        print(f"  {k:20s}: {v:.4f}")

    print(f"\n  Seed da testare: {len(SEEDS)}")
    print(f"  Paralleli      : {N_PARALLEL}")
    print()

    # Pre-genera tutti i world e li salva con nome univoco
    worker_args = []
    for i, seed in enumerate(SEEDS):
        goal_x, goal_y, goal_z = generate_world(PROJECT_DIR, seed=seed, face_goal=False)

        world_dst = os.path.join(PROJECT_DIR,
            f"lrauv_gazebo_plugins/worlds/val_{i}.sdf")
        shutil.copy(WORLD_PATH, world_dst)

        tree = ET.parse(world_dst)
        pose_parts = tree.getroot().find(".//include/pose").text.split()
        sx = float(pose_parts[0])
        sy = float(pose_parts[1])
        sz = float(pose_parts[2])
        dist_init = np.sqrt((goal_x-sx)**2 + (goal_y-sy)**2 + (goal_z-sz)**2)

        worker_args.append((i % N_PARALLEL, theta, seed,
                            goal_x, goal_y, goal_z, dist_init, world_dst))

    print(f"[VAL] World generati. Avvio validazione...\n")

    # Esegui in batch da N_PARALLEL
    results = []
    for batch_start in range(0, len(SEEDS), N_PARALLEL):
        batch = worker_args[batch_start:batch_start + N_PARALLEL]

        with concurrent.futures.ThreadPoolExecutor(max_workers=N_PARALLEL) as ex:
            batch_results = list(ex.map(run_worker, batch))

        for args, result in zip(batch, batch_results):
            _, _, seed, _, _, _, dist_init, _ = args
            results.append({
                "seed":      seed,
                "tag":       result["tag"] or "none",
                "t":         result["t"],
                "path":      result["path"],
                "dist":      result["dist"],
                "dist_init": dist_init
            })

    # Statistiche finali
    arrived   = [r for r in results if r["tag"] == "arrived"]
    collision = [r for r in results if r["tag"] == "collision"]
    timeout   = [r for r in results if r["tag"] == "timeout"]
    none_res  = [r for r in results if r["tag"] == "none"]

    n = len(results)
    print(f"\n{'='*60}")
    print(f"  RISULTATI VALIDAZIONE ({n} seed)")
    print(f"{'='*60}")
    print(f"  Arrived  : {len(arrived):2d}/{n}  ({100*len(arrived)/n:.0f}%)")
    print(f"  Collision: {len(collision):2d}/{n}  ({100*len(collision)/n:.0f}%)")
    print(f"  Timeout  : {len(timeout):2d}/{n}  ({100*len(timeout)/n:.0f}%)")
    print(f"  None     : {len(none_res):2d}/{n}  ({100*len(none_res)/n:.0f}%)")

    if arrived:
        times = [r["t"]    for r in arrived if r["t"]]
        paths = [r["path"] for r in arrived if r["path"]]
        print(f"\n  Arrived stats:")
        print(f"    Tempo medio : {np.mean(times):.1f}s  "
              f"(min={min(times):.1f} max={max(times):.1f})")
        print(f"    Path medio  : {np.mean(paths):.1f}m  "
              f"(min={min(paths):.1f} max={max(paths):.1f})")

    if collision or timeout or none_res:
        print(f"\n  Seed falliti:")
        for r in results:
            if r["tag"] != "arrived":
                print(f"    seed={r['seed']:5d} | {r['tag']}")

    filename = "validation_default.json" if cli.default else "validation_es.json"
    out_path = os.path.join(RESULTS_DIR, filename)
    with open(out_path, "w") as f:
        json.dump({
            "best_reward_es": best_reward,
            "theta":          theta,
            "n_seeds":        n,
            "arrived_rate":   len(arrived) / n,
            "results":        results
        }, f, indent=2)
    print(f"\n  Salvato: {out_path}")


if __name__ == "__main__":
    try:
        main()
    finally:
        for i in range(N_PARALLEL):
            subprocess.run(["docker", "rm", "-f", f"val_gazebo_{i}"],
                           capture_output=True)
        if os.path.exists(WORKERS_DIR):
            shutil.rmtree(WORKERS_DIR)