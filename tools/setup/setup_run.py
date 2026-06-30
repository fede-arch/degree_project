#!/usr/bin/env python3
"""
Uso:
  setup_run.py single  <project_dir> <seed> <face_goal> <r_max> <radius_arrived>
  setup_run.py multi   <project_dir> <n_drones> <seed> <face_goal> <r_max> <radius_arrived>
  setup_run.py remote  <project_dir> <seed>
"""

import sys, json, os

def load_theta(project_dir):
    path = os.path.join(project_dir, 'optimization/results/training/final_theta.json')
    with open(path) as f:
        theta = json.load(f)["theta"]
    print("[RUN] Parametri da final_theta.json:")
    for k, v in theta.items():
        print(f"  {k:20s}: {v:.4f}")
    return theta

def run_single(project_dir, seed, face_goal, r_max, radius_arrived):
    from generate_world import generate_world
    from es_utils import write_sdf

    theta = load_theta(project_dir)
    gx, gy, gz = generate_world(project_dir, seed=seed, face_goal=face_goal)
    model_dir = os.path.join(project_dir, 'lrauv_description/models/tethys_equipped')
    write_sdf(model_dir, theta, 0, gx, gy, gz, r_max, radius_arrived)
    print(f"[RUN] Goal: ({gx:.1f}, {gy:.1f}, {gz:.1f})")

def run_multi(project_dir, n_drones, seed, face_goal, r_max, radius_arrived):
    from generate_world_multi import generate_world_multi
    from es_utils import write_sdf

    theta = load_theta(project_dir)
    spawns, goals = generate_world_multi(
        project_dir, seed=seed, face_goal=face_goal, n_drones=n_drones)

    for i, (gx, gy, gz) in enumerate(goals):
        model_dir = os.path.join(project_dir, f'lrauv_description/models/tethys_equipped_{i}')
        write_sdf(model_dir, theta, i, gx, gy, gz, r_max, radius_arrived)
        sx, sy, sz = spawns[i]
        print(f"[RUN] Drone {i:2d}: spawn=({sx:.1f},{sy:.1f},{sz:.1f}) goal=({gx:.1f},{gy:.1f},{gz:.1f})")

def run_remote(project_dir, seed):
    from generate_world import generate_world

    gx, gy, gz = generate_world(project_dir, seed=seed, face_goal=False)
    tmpl = os.path.join(project_dir, 'lrauv_description/models/tethys_equipped/model_remote.sdf.template')
    dest = os.path.join(project_dir, 'lrauv_description/models/tethys_equipped/model.sdf')

    with open(tmpl) as f:
        content = f.read()

    for k, v in [('__DRONE_ID__', '0'), ('__GOAL_X__', f'{gx:.2f}'),
                 ('__GOAL_Y__', f'{gy:.2f}'), ('__GOAL_Z__', f'{gz:.2f}')]:
        content = content.replace(k, v)

    with open(dest, 'w') as f:
        f.write(content)

    print(f"[REMOTE] Goal: ({gx:.1f}, {gy:.1f}, {gz:.1f})")
    print(f"[REMOTE] ./standalone_controller tethys_0 {gx:.2f} {gy:.2f} {gz:.2f}")

if __name__ == "__main__":
    mode        = sys.argv[1]
    project_dir = sys.argv[2]

    sys.path.insert(0, os.path.join(project_dir, 'optimization'))
    sys.path.insert(0, os.path.join(project_dir, 'scripts'))

    if mode == "single":
        run_single(project_dir, int(sys.argv[3]), sys.argv[4].lower() == 'true',
                   float(sys.argv[5]), float(sys.argv[6]))
    elif mode == "multi":
        run_multi(project_dir, int(sys.argv[3]), int(sys.argv[4]),
                  sys.argv[5].lower() == 'true', float(sys.argv[6]), float(sys.argv[7]))
    elif mode == "remote":
        run_remote(project_dir, int(sys.argv[3]))
    else:
        print(f"[ERR] Modalità sconosciuta: {mode}")
        sys.exit(1)