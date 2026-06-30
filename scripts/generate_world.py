import math
import random
import numpy as np
import os

ROCK_Z_BASE      = -98.0
EXCLUSION_RADIUS = 60.0
N_ROCKS          = 30
ROCK_MODEL       = "falling rock 1"
MIN_ROCK_DIST    = 40.0

TEMPLATE_PATH = "lrauv_gazebo_plugins/worlds/navigation_world.sdf.in"
WORLD_PATH    = "lrauv_gazebo_plugins/worlds/navigation_world.sdf"

def generate_world(project_dir, seed=None, face_goal=False):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    drone_z = -75.0

    quadrant = random.randint(0, 3)
    if quadrant == 0:
        spawn_x = random.uniform(-100, 50);  spawn_y = random.uniform(-150, 0)
        goal_x  = random.uniform(200, 350);  goal_y  = random.uniform(0, 150)
    elif quadrant == 1:
        spawn_x = random.uniform(-100, 50);  spawn_y = random.uniform(0, 150)
        goal_x  = random.uniform(200, 350);  goal_y  = random.uniform(-150, 0)
    elif quadrant == 2:
        spawn_x = random.uniform(200, 350);  spawn_y = random.uniform(0, 150)
        goal_x  = random.uniform(-100, 50);  goal_y  = random.uniform(-150, 0)
    else:
        spawn_x = random.uniform(200, 350);  spawn_y = random.uniform(-150, 0)
        goal_x  = random.uniform(-100, 50);  goal_y  = random.uniform(0, 150)

    goal_z = drone_z + random.uniform(-5, 5)
    dist   = np.sqrt((goal_x-spawn_x)**2 + (goal_y-spawn_y)**2)

    direction = math.atan2(goal_y - spawn_y, goal_x - spawn_x)
    if face_goal:
        spawn_yaw = direction - math.pi + random.uniform(-0.3, 0.3)
    else:
        spawn_yaw = direction + random.uniform(-0.3, 0.3)

    keypoints = [(spawn_x, spawn_y, EXCLUSION_RADIUS), (goal_x, goal_y, EXCLUSION_RADIUS)]
    placed_rocks = []

    def too_close(x, y):
        if any(np.sqrt((x-px)**2 + (y-py)**2) < r for px, py, r in keypoints):
            return True
        return any(np.sqrt((x-rx)**2 + (y-ry)**2) < MIN_ROCK_DIST for rx, ry in placed_rocks)

    rocks_sdf = ""
    placed = 0

    for _ in range(3000):
        if placed >= N_ROCKS:
            break
        x = random.uniform(-120, 370)
        y = random.uniform(-170, 170)
        if too_close(x, y):
            continue
        yaw      = random.uniform(0, 3.14)
        z_offset = random.uniform(0, 25)
        rocks_sdf += f"""  <include>
    <name>rock_{placed}</name>
    <uri>{ROCK_MODEL}</uri>
    <pose>{x:.2f} {y:.2f} {ROCK_Z_BASE + z_offset:.2f} 0 0 {yaw:.2f}</pose>
    <static>true</static>
  </include>\n"""
        placed_rocks.append((x, y))
        placed += 1

    if placed < 8:
        print(f"[GEN] WARNING: solo {placed} rocce, scenario non valido")

    tmpl = os.path.join(project_dir, TEMPLATE_PATH)
    with open(tmpl) as f:
        content = f.read()

    for k, v in [
        ("__SPAWN_X__",   f"{spawn_x:.2f}"),
        ("__SPAWN_Y__",   f"{spawn_y:.2f}"),
        ("__DRONE_Z__",   f"{drone_z:.2f}"),
        ("__SPAWN_YAW__", f"{spawn_yaw:.2f}"),
        ("__GOAL_X__",    f"{goal_x:.2f}"),
        ("__GOAL_Y__",    f"{goal_y:.2f}"),
        ("__GOAL_Z__",    f"{goal_z:.2f}"),
        ("__ROCKS_SDF__", rocks_sdf),
    ]:
        content = content.replace(k, v)

    world_path = os.path.join(project_dir, WORLD_PATH)
    with open(world_path, "w") as f:
        f.write(content)

    print(f"[GEN] Seed: {seed} | {placed} rocce | dist={dist:.0f}m")

    return goal_x, goal_y, goal_z