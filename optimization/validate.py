#!/usr/bin/env python3
import json, os, sys, shutil, subprocess, time
import numpy as np
import concurrent.futures

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(SCRIPT_DIR, "../lrauv_gazebo_plugins/scripts"))
sys.path.insert(0, SCRIPT_DIR)

from generate_world import generate_world
from es_optimizer import (read_spawn_pose, PROJECT_DIR, theta_to_json,
                          shaped_reward, parse_result, DRONE_NS, LOG_FILE)

BEST_FILE   = os.path.join(PROJECT_DIR, "es/results/best_theta.json")
PARAM_NAMES = ["s_max","smooth_l","gain_steer","gain_pitch","radius_slowdown","elev_cost"]
N_PARALLEL  = 6
STARTUP_WAIT = 20.0

SEEDS = [
    1117, 4306, 3411, 3223, 4297, 5489, 1375, 1108,
    7921, 6214, 5833, 1084, 4927, 5377, 4566, 5479,
    7559, 3797, 9333, 4200, 2976,  953,  608, 9491,
    4013,  757, 1830, 4323, 4940, 4248
]

def run_validation_worker(args):
    worker_id, theta_i, dist_i, world_container = args
    cname     = f"val_gazebo_{worker_id}"
    partition = f"val{worker_id}"
    ns        = DRONE_NS

    render_gid = os.popen("getent group render | cut -d: -f3").read().strip()
    video_gid  = os.popen("getent group video | cut -d: -f3").read().strip()
    subprocess.run(["docker", "rm", "-f", cname], capture_output=True)

    proc = subprocess.Popen([
        "docker", "run", "--rm", "--name", cname,
        "--device", "/dev/dri/card1", "--device", "/dev/dri/renderD128",
        "--group-add", render_gid, "--group-add", video_gid,
        "--volume", f"{PROJECT_DIR}:/lrauv_ws/src/degree_project",
        "lrauv:harmonic", "bash", "-c",
        (
            f"source /setup.sh && "
            f"export GZ_PARTITION={partition} && "
            f"export GZ_SIM_RESOURCE_PATH=/lrauv_ws/src/degree_project/lrauv_description/models && "
            f"export GZ_SIM_SYSTEM_PLUGIN_PATH=/lrauv_ws/src/degree_project/docker_build && "
            f"gz sim --headless-rendering -s {world_container}"
        )
    ], stdout=open(f"{LOG_FILE}.val{worker_id}", "w"),
       stderr=subprocess.STDOUT, text=True)
    time.sleep(STARTUP_WAIT)

    try:
        params_json = theta_to_json(theta_i)
        subprocess.run(["docker", "exec", cname, "bash", "-c",
            f"source /setup.sh && export GZ_PARTITION={partition} && "
            f"gz topic -t /{ns}/es/vfh_params -m gz.msgs.StringMsg "
            f"-p 'data: \"{params_json.replace(chr(34), chr(92)+chr(34))}\"'"
        ], capture_output=True)
        time.sleep(2.0)

        listener = subprocess.Popen(["docker", "exec", cname, "bash", "-c",
            f"source /setup.sh && export GZ_PARTITION={partition} && "
            f"gz topic -e -n 1 -t /{ns}/es/episode_result"
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

        subprocess.run(["docker", "exec", cname, "bash", "-c",
            f"source /setup.sh && export GZ_PARTITION={partition} && "
            f"gz service -s /world/empty_environment/control "
            f"--reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean "
            f"--req 'pause: false' --timeout 2000"
        ], capture_output=True)

        try:
            out, _ = listener.communicate(timeout=320)
        except subprocess.TimeoutExpired:
            listener.kill(); out = ""
    finally:
        subprocess.run(["docker", "rm", "-f", cname], capture_output=True)
        proc.terminate()

    return shaped_reward(parse_result(out), dist_i)


def main():
    with open(BEST_FILE) as f:
        best = json.load(f)
    theta = np.array([best["theta"][k] for k in PARAM_NAMES])

    print("=" * 50)
    print("PARAMETRI VALIDAZIONE:")
    print(f"  best reward ES: {best['reward']:.4f}")
    for k, v in best["theta"].items():
        print(f"  {k}: {v:.4f}")
    print(f"  threshold: 0.001 (fisso)")
    print("=" * 50)
    print(f"\nPreparazione {len(SEEDS)} world...")

    # 1. Pre-genera tutti i world e li copia in file separati
    base = os.path.join(PROJECT_DIR, "lrauv_gazebo_plugins/worlds/navigation_world.sdf")
    worker_args = []
    world_files = []

    for i, seed in enumerate(SEEDS):
        goal_x, goal_y, goal_z = generate_world(PROJECT_DIR, seed=seed)
        sx, sy, sz, _ = read_spawn_pose()
        dist_i = np.sqrt((goal_x-sx)**2 + (goal_y-sy)**2 + (goal_z-sz)**2)

        world_local = os.path.join(
            PROJECT_DIR, f"lrauv_gazebo_plugins/worlds/val_{i}.sdf")
        shutil.copy(base, world_local)
        world_files.append(world_local)

        world_container = (
            f"/lrauv_ws/src/degree_project/"
            f"lrauv_gazebo_plugins/worlds/val_{i}.sdf")
        worker_args.append((i, theta, dist_i, world_container))

    print(f"World pronti. Lancio {len(SEEDS)} seed ({N_PARALLEL} in parallelo)...\n")

    # 2. Esegui in parallelo, stampa man mano
    results = [None] * len(SEEDS)
    with concurrent.futures.ThreadPoolExecutor(max_workers=N_PARALLEL) as ex:
        futures = {ex.submit(run_validation_worker, a): i
                   for i, a in enumerate(worker_args)}
        for fut in concurrent.futures.as_completed(futures):
            i      = futures[fut]
            seed   = SEEDS[i]
            reward = fut.result()
            tag    = ("ARRIVED" if reward > 2.0
                      else "TIMEOUT" if reward > -0.6 else "COLLISION")
            results[i] = (seed, tag, reward)
            done = sum(1 for r in results if r is not None)
            print(f"  [{done:2d}/{len(SEEDS)}] seed={seed:5d} | {tag:9s} | r={reward:+.3f}")

    # 3. Cleanup world temporanei
    for wf in world_files:
        try: os.remove(wf)
        except: pass

    # 4. Risultati finali
    arrived = sum(1 for _, t, _ in results if t == "ARRIVED")
    print(f"\n{'='*40}")
    print(f"Success rate: {arrived}/{len(SEEDS)} ({100*arrived/len(SEEDS):.0f}%)")
    print(f"\nSeed falliti:")
    for seed, tag, r in results:
        if tag != "ARRIVED":
            print(f"  seed={seed} | {tag} | r={r:+.3f}")

    with open(os.path.join(PROJECT_DIR, "es/results/validation.json"), "w") as f:
        json.dump([{"seed":s,"tag":t,"reward":r} for s,t,r in results], f, indent=2)

if __name__ == "__main__":
    try:
        main()
    finally:
        for i in range(len(SEEDS)):
            subprocess.run(["docker", "rm", "-f", f"val_gazebo_{i}"],
                           capture_output=True)