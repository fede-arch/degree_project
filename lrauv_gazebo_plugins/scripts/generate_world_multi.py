import math
import random
import numpy as np
import os

ROCK_Z_BASE      = -88.0
EXCLUSION_RADIUS = 60.0
N_ROCKS          = 0
ROCK_MODEL       = "falling rock 1"
MIN_ROCK_DIST    = 10.0
DRONE_EXCLUSION  = 10.0   
X_OFFSET = -160
Y_OFFSET = 0


def on_path(x, y, spawn_x, spawn_y, goal_x, goal_y, margin=35.0):
    """True se (x,y) è entro margin metri dal segmento spawn->goal."""
    dx, dy = goal_x - spawn_x, goal_y - spawn_y
    length = np.sqrt(dx*dx + dy*dy)
    if length < 1e-6:
        return False
    t = np.clip(((x - spawn_x)*dx + (y - spawn_y)*dy) / (length*length), 0.0, 1.0)
    px = spawn_x + t*dx
    py = spawn_y + t*dy
    return np.sqrt((x-px)**2 + (y-py)**2) < margin


def generate_world_multi(project_dir, seed=None, face_goal=True, n_drones=1, output_path=None):
    """
    Genera navigation_world.sdf con n_drones droni e restituisce:
      spawns → lista di (x, y, z) per ogni drone
      goals  → lista di (x, y, z) per ogni drone

    face_goal=True  → droni spawna guardando verso il goal
    face_goal=False → droni spawna con le spalle al goal
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
        print(f"[GEN] Seed: {seed}")

    drone_z = -60.0
    x_min = -50  + X_OFFSET
    x_max =   50 + X_OFFSET
    y_min =  -50 + Y_OFFSET
    y_max =   50 + Y_OFFSET

    # --- Genera spawn e goal per ogni drone ---
    spawns = []
    goals  = []

    for i in range(n_drones):
        # Spawn: evita altri spawn già piazzati
        for _ in range(200):
            sx = random.uniform(x_min, x_max)
            sy = random.uniform(y_min, y_max)
            too_close = any(
                np.sqrt((sx - px)**2 + (sy - py)**2) < DRONE_EXCLUSION
                for px, py, _ in spawns
            )
            if not too_close:
                break
        spawns.append((sx, sy, drone_z))

        for _ in range(200):
          gx = random.uniform(x_min, x_max)
          gy = random.uniform(y_min, y_max)
          gz = drone_z + random.uniform(-10, 7)
          dist_ok = np.sqrt((gx - sx)**2 + (gy - sy)**2) > 30.0
          goal_far = all(
              np.sqrt((gx - px)**2 + (gy - py)**2) > DRONE_EXCLUSION
              for px, py, _ in goals
          )
          if dist_ok and goal_far:
              break
        goals.append((gx, gy, gz))

    # --- Rocce ---
    # Keypoints: tutti gli spawn e goal con exclusion zone
    keypoints = []
    for sx, sy, _ in spawns:
        keypoints.append((sx, sy, EXCLUSION_RADIUS))
    for gx, gy, _ in goals:
        keypoints.append((gx, gy, EXCLUSION_RADIUS))

    placed_rocks = []

    def too_close_rock(x, y):
        if any(np.sqrt((x-px)**2 + (y-py)**2) < r for px, py, r in keypoints):
            return True
        if any(np.sqrt((x-rx)**2 + (y-ry)**2) < MIN_ROCK_DIST for rx, ry in placed_rocks):
            return True
        return False

    rocks_sdf      = ""
    placed         = 0
    attempts       = 0
    on_path_count  = 0
    MIN_ON_PATH    = 2

    while placed < N_ROCKS and attempts < 5000:
        attempts += 1
        x = random.uniform(x_min, x_max)
        y = random.uniform(y_min, y_max)
        if too_close_rock(x, y):
            continue

        # Controlla se è su almeno uno dei percorsi
        is_on = any(
            on_path(x, y, sx, sy, gx, gy)
            for (sx, sy, _), (gx, gy, _) in zip(spawns, goals)
        )
        if on_path_count < MIN_ON_PATH and not is_on:
            continue

        yaw      = random.uniform(0, 3.14)
        z_offset = random.uniform(0, 15)
        rocks_sdf += f"""
  <include>
    <name>rock_{placed}</name>
    <uri>{ROCK_MODEL}</uri>
    <pose>{x:.2f} {y:.2f} {ROCK_Z_BASE + z_offset:.2f} 0 0 {yaw:.2f}</pose>
    <static>true</static>
  </include>
"""
        if is_on:
            on_path_count += 1
        placed_rocks.append((x, y))
        placed += 1

    if placed < N_ROCKS:
        print(f"[GEN] WARN: solo {placed}/{N_ROCKS} rocce piazzate")

    # --- Include droni nel world SDF ---
    drones_sdf = ""
    for i, ((sx, sy, sz), (gx, gy, gz)) in enumerate(zip(spawns, goals)):
        dx = gx - sx
        dy = gy - sy
        direction = math.atan2(gy - sy, gx - sx)
        yaw = direction - math.pi + random.uniform(-0.3, 0.3)

        drones_sdf += f"""
    <!-- Drone {i} -->
    <include>
      <pose>{sx:.2f} {sy:.2f} {sz:.2f} 0 0 {yaw:.2f}</pose>
      <uri>tethys_equipped_{i}</uri>
    </include>

    <!-- Target {i} -->
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
    </model>
"""

    # --- World SDF ---
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

    {drones_sdf}

    <!-- Scogli -->
    {rocks_sdf}

    <!-- Fondale -->
    <include>
      <name>portuguese_ledge</name>
      <uri>portuguese ledge</uri>
      <pose>{(x_min+x_max)/2:.1f} {(y_min+y_max)/2:.1f} -70 0 0 0</pose>
      <static>true</static>
    </include>

    <!-- Piano fondale per collisioni -->
    <model name="sea_floor_collision">
      <static>true</static>
      <pose>{(x_min+x_max)/2:.1f} {(y_min+y_max)/2:.1f} -95 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>2000 2000 1</size></box>
          </geometry>
        </collision>
      </link>
    </model>

  </world>
</sdf>
"""

    if output_path is None:
      output_path = os.path.join(
          project_dir, "lrauv_gazebo_plugins/worlds/navigation_world_multi.sdf")
    with open(output_path, "w") as f:
        f.write(world_content)

    print(f"[GEN] World multi: {n_drones} droni | {placed} rocce")

    return spawns, goals