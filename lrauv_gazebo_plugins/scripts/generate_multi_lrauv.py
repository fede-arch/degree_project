import random
import numpy as np
import os
import shutil

ROCK_Z_BASE      = -113.0
EXCLUSION_RADIUS = 60.0
MIN_ROCK_DIST    = 15.0
MIN_DRONE_DIST   = 40.0
N_ROCKS          = 50

ROCK_MODELS = [
    "falling rock 1",
    "falling rock 2",
]

def on_path(x, y, spawn_x, spawn_y, goal_x, goal_y, margin=25.0):
    """True se (x,y) è entro margin metri dal segmento spawn->goal."""
    dx, dy = goal_x - spawn_x, goal_y - spawn_y
    length = np.sqrt(dx*dx + dy*dy)
    if length < 1e-6:
        return False
    t = np.clip(((x - spawn_x)*dx + (y - spawn_y)*dy) / (length*length), 0.0, 1.0)
    px = spawn_x + t*dx
    py = spawn_y + t*dy
    return np.sqrt((x-px)**2 + (y-py)**2) < margin

def too_close_to_rocks(x, y, positions):
    for px, py in positions:
        if np.sqrt((x-px)**2 + (y-py)**2) < MIN_ROCK_DIST:
            return True
    return False

def too_close_to_occupied(x, y, occupied, min_dist):
    for px, py in occupied:
        if np.sqrt((x-px)**2 + (y-py)**2) < min_dist:
            return True
    return False

def generate_multi_lrauv(project_dir, seed=None, n_drones=3):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
        print(f"[GEN] Seed: {seed}, droni: {n_drones}")

    base_z   = -75.0 + random.uniform(-5, 5)
    occupied = []
    spawns   = []
    goals    = []

    for i in range(n_drones):
        
        for _ in range(500):
            sx = random.uniform(-50, 220)
            sy = random.uniform(-100, 100)
            if not too_close_to_occupied(sx, sy, occupied, MIN_DRONE_DIST):
                break
        sz   = base_z + random.uniform(-5, 5)
        syaw = random.uniform(0, 6.28)
        spawns.append((sx, sy, sz, syaw))
        occupied.append((sx, sy))

        for _ in range(500):
            gx = random.uniform(-50, 220)
            gy = random.uniform(-100, 100)
            if np.sqrt((gx-sx)**2 + (gy-sy)**2) > 120.0 and \
               not too_close_to_occupied(gx, gy, occupied, MIN_DRONE_DIST):
                break
        gz = sz + random.uniform(-6, 6)
        goals.append((gx, gy, gz))
        occupied.append((gx, gy))

    keypoints = [(x, y, EXCLUSION_RADIUS) for x, y, *_ in spawns] + \
                [(x, y, EXCLUSION_RADIUS) for x, y, z   in goals]

    def too_close_to_keypoints(x, y):
        for px, py, radius in keypoints:
            if np.sqrt((x-px)**2 + (y-py)**2) < radius:
                return True
        return False

    rocks_sdf        = ""
    placed           = 0
    attempts         = 0
    placed_positions = []
    on_path_count = 0
    MIN_ON_PATH = 5 

    while placed < N_ROCKS and attempts < 1000:
        attempts += 1
        x = random.uniform(-80, 250)
        y = random.uniform(-130, 130)
        if too_close_to_keypoints(x, y):
            continue
        if too_close_to_rocks(x, y, placed_positions):
            continue
        
        is_on = on_path(x, y, spawn_x, spawn_y, goal_x, goal_y)

        if on_path_count < MIN_ON_PATH and not is_on:
          continue

        model = random.choice(ROCK_MODELS)
        yaw = random.uniform(0, 3.14)
        z_offset = random.uniform(0, 25)

        rocks_sdf += f"""
  <include>
    <name>rock_{placed}</name>
    <uri>{model}</uri>
    <pose>{x:.2f} {y:.2f} {ROCK_Z_BASE + z_offset:.2f} 0 0 {yaw:.2f}</pose>
    <static>true</static>
  </include>
"""
        placed_positions.append((x, y))
        if is_on:
            on_path_count += 1
        placed += 1

    # ── SDF droni e target ────────────────────────────────────────────────────
    drones_sdf  = ""
    targets_sdf = ""
    for i, ((sx, sy, sz, syaw), (gx, gy, gz)) in enumerate(zip(spawns, goals)):
        drones_sdf += f"""
    <include>
      <uri>tethys_equipped_{i}</uri>
      <pose>{sx:.2f} {sy:.2f} {sz:.2f} 0 0 {syaw:.2f}</pose>
    </include>"""
        targets_sdf += f"""
    <model name="target_{i}">
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
    </model>"""

    world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="empty_environment">

    <scene>
      <ambient>0.1 0.2 0.3 1.0</ambient>
      <background>0.0 0.1 0.3</background>
    </scene>

    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <plugin filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-buoyancy-system"
      name="gz::sim::systems::Buoyancy">
      <graded_buoyancy>
        <default_density>1025</default_density>
        <density_change>
          <above_depth>0</above_depth>
          <density>1.125</density>
        </density_change>
      </graded_buoyancy>
    </plugin>
    <plugin filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system"
      name="gz::sim::systems::Contact"/>

    <!-- Droni -->
    {drones_sdf}

    <!-- Target -->
    {targets_sdf}

    <!-- Scogli sul fondale -->
    {rocks_sdf}

    <!-- Fondale - Portuguese Ledge -->
    <include>
      <name>portuguese_ledge</name>
      <uri>portuguese ledge</uri>
      <pose>500 0 -70 0 0 0</pose>
      <static>true</static>
    </include>

  </world>
</sdf>
"""

    world_path = os.path.join(project_dir,
        "lrauv_gazebo_plugins/worlds/navigation_world.sdf")
    with open(world_path, "w") as f:
        f.write(world_content)

    template_path = os.path.join(project_dir,
        "lrauv_description/models/tethys_equipped/model.sdf.template")
    config_src = os.path.join(project_dir,
        "lrauv_description/models/tethys_equipped/model.config")

    if os.path.exists(template_path):
        with open(template_path, "r") as f:
            template = f.read()

        for i, ((sx, sy, sz, syaw), (gx, gy, gz)) in enumerate(zip(spawns, goals)):
            model_dir = os.path.join(project_dir,
                f"lrauv_description/models/tethys_equipped_{i}")
            os.makedirs(model_dir, exist_ok=True)

            if os.path.exists(config_src):
                shutil.copy(config_src, os.path.join(model_dir, "model.config"))

            content = template
            content = content.replace("__DRONE_ID__",        str(i))
            content = content.replace("__REMOVE_ON_COLLISION__", "true")
            content = content.replace("__GOAL_X__",          f"{gx:.2f}")
            content = content.replace("__GOAL_Y__",          f"{gy:.2f}")
            content = content.replace("__GOAL_Z__",          f"{gz:.2f}")
            content = content.replace("__THRESHOLD__",       "0.001")
            content = content.replace("__S_MAX__",           "20")
            content = content.replace("__SMOOTH_L__",        "5")
            content = content.replace("__GAIN_STEER__",      "0.5")
            content = content.replace("__GAIN_PITCH__",      "0.5")
            content = content.replace("__RADIUS_SLOWDOWN__", "15.0")

            with open(os.path.join(model_dir, "model.sdf"), "w") as f:
                f.write(content)

            dist = np.sqrt((gx-sx)**2 + (gy-sy)**2 + (gz-sz)**2)
            print(f"[GEN] drone_{i}: spawn=({sx:.1f},{sy:.1f},{sz:.1f}) "
                  f"goal=({gx:.1f},{gy:.1f},{gz:.1f}) dist={dist:.1f}m")

    print(f"[GEN] {placed} rocce generate")
    return spawns, goals