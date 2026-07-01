import math
import random
import numpy as np
import os

DRONE_Z         = -75.0
DRONE_EXCLUSION = 10.0
AREA_HALF       = 50.0

TEMPLATE_PATH = "lrauv_gazebo_plugins/worlds/navigation_world_multi.sdf.in"
WORLD_PATH    = "lrauv_gazebo_plugins/worlds/navigation_world_multi.sdf"

def generate_world_multi(project_dir, seed=None, face_goal=True, n_drones=1, output_path=None):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
        print(f"[GEN] Seed: {seed}")

    x_min, x_max = -AREA_HALF, AREA_HALF
    y_min, y_max = -AREA_HALF, AREA_HALF

    spawns = []
    goals  = []

    for _ in range(n_drones):
        for _ in range(200):
            sx = random.uniform(x_min, x_max)
            sy = random.uniform(y_min, y_max)
            if all(np.sqrt((sx-px)**2 + (sy-py)**2) >= DRONE_EXCLUSION for px, py, _ in spawns):
                break
        spawns.append((sx, sy, DRONE_Z))

        for _ in range(200):
            gx = random.uniform(x_min, x_max)
            gy = random.uniform(y_min, y_max)
            gz = DRONE_Z + random.uniform(-10, 7)
            if (np.sqrt((gx-sx)**2 + (gy-sy)**2) > 30.0 and
                    all(np.sqrt((gx-px)**2 + (gy-py)**2) >= DRONE_EXCLUSION for px, py, _ in goals)):
                break
        goals.append((gx, gy, gz))

    drones_sdf = ""
    for i, ((sx, sy, sz), (gx, gy, gz)) in enumerate(zip(spawns, goals)):
        direction = math.atan2(gy - sy, gx - sx)
        if face_goal:
            yaw = direction - math.pi + random.uniform(-0.3, 0.3)
        else:
            yaw = direction + random.uniform(-0.3, 0.3)

        drones_sdf += f"""    <include>
      <pose>{sx:.2f} {sy:.2f} {sz:.2f} 0 0 {yaw:.2f}</pose>
      <uri>tethys_equipped_{i}</uri>
    </include>
    <model name="target_marker_{i}">
      <static>true</static>
      <pose>{gx:.2f} {gy:.2f} {gz:.2f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><sphere><radius>1.0</radius></sphere></geometry>
          <material>
            <ambient>1.0 0.0 0.0 1.0</ambient>
            <diffuse>1.0 0.0 0.0 1.0</diffuse>
            <emissive>0.8 0.0 0.0 1.0</emissive>
          </material>
        </visual>
      </link>
    </model>\n"""

    tmpl = os.path.join(project_dir, TEMPLATE_PATH)
    with open(tmpl) as f:
        content = f.read()

    content = content.replace("__DRONES_SDF__", drones_sdf)

    out = output_path or os.path.join(project_dir, WORLD_PATH)
    with open(out, "w") as f:
        f.write(content)

    print(f"[GEN] World multi: {n_drones} droni")

    return spawns, goals