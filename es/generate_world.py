import math
import random
import numpy as np
import os

ROCK_Z_BASE    = -83.0
EXCLUSION_RADIUS = 60.0
N_ROCKS        = 30
ROCK_MODEL     = "falling rock 1"


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


def generate_world(project_dir, seed=None, face_goal=False):
    """
    Genera navigation_world.sdf e restituisce (goal_x, goal_y, goal_z).

    face_goal=True  → veicolo spawna guardando verso il goal (debug)
    face_goal=False → veicolo spawna con le spalle al goal (task difficile)
    """
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
        print(f"[GEN] Seed: {seed}")

    # --- Spawn e goal ---
    spawn_x = random.uniform(-50, 220)
    spawn_y = random.uniform(-100, 100)
    drone_z = -60.0

    for _ in range(100):
        goal_x = random.uniform(-50, 220)
        goal_y = random.uniform(-100, 100)
        if np.sqrt((goal_x - spawn_x)**2 + (goal_y - spawn_y)**2) > 120.0:
            break
    goal_z = drone_z + random.uniform(-10, 7)

    dx = goal_x - spawn_x
    dy = goal_y - spawn_y
    direction = math.atan2(dy, dx)

    if face_goal:
        spawn_yaw = direction + random.uniform(-0.3, 0.3)
    else:
        spawn_yaw = direction - math.pi + random.uniform(-0.3, 0.3)

    # --- Rocce ---
    keypoints = [
        (spawn_x, spawn_y, EXCLUSION_RADIUS),
        (goal_x,  goal_y,  EXCLUSION_RADIUS),
    ]

    def too_close(x, y):
        return any(
            np.sqrt((x-px)**2 + (y-py)**2) < r
            for px, py, r in keypoints
        )

    rocks_sdf = ""
    placed = 0
    attempts = 0
    on_path_count = 0
    MIN_ON_PATH = 4

    while placed < N_ROCKS and attempts < 1000:
        attempts += 1
        x = random.uniform(-80, 250)
        y = random.uniform(-130, 130)
        if too_close(x, y):
            continue
        is_on = on_path(x, y, spawn_x, spawn_y, goal_x, goal_y)
        if on_path_count < MIN_ON_PATH and not is_on:
            continue

        yaw      = random.uniform(0, 3.14)
        z_offset = random.uniform(0, 25)
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
        placed += 1

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

    <!-- Tethys -->
    <include>
      <pose>{spawn_x:.2f} {spawn_y:.2f} {drone_z:.2f} 0 0 {spawn_yaw:.2f}</pose>
      <uri>tethys_equipped</uri>
    </include>

    <!-- Target -->
    <model name="target_marker">
      <static>true</static>
      <pose>{goal_x:.2f} {goal_y:.2f} {goal_z:.2f} 0 0 0</pose>
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

    <!-- Scogli -->
    {rocks_sdf}

    <!-- Fondale -->
    <include>
      <name>portuguese_ledge</name>
      <uri>portuguese ledge</uri>
      <pose>500 0 -70 0 0 0</pose>
      <static>true</static>
    </include>

    <!-- Piano fondale per collisioni -->
    <model name="sea_floor_collision">
      <static>true</static>
      <pose>0 0 -95 0 0 0</pose>
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

    world_path = os.path.join(
        project_dir, "lrauv_gazebo_plugins/worlds/navigation_world.sdf")
    with open(world_path, "w") as f:
        f.write(world_content)

    print(f"[GEN] World: {placed} rocce | "
          f"spawn=({spawn_x:.1f},{spawn_y:.1f},{drone_z:.1f}) yaw={spawn_yaw:.2f} | "
          f"goal=({goal_x:.1f},{goal_y:.1f},{goal_z:.1f})")

    return goal_x, goal_y, goal_z