import math
import random
import numpy as np
import os

ROCK_Z_BASE      = -98.0
EXCLUSION_RADIUS = 60.0
N_ROCKS          = 30
ROCK_MODEL       = "falling rock 1"
MIN_ROCK_DIST    = 40.0

def generate_world(project_dir, seed=None, face_goal=False):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    # Spawn e goal agli estremi opposti
    drone_z = -75.0

    quadrant = random.randint(0, 3)
    if quadrant == 0:
        spawn_x = random.uniform(-100, 50)
        spawn_y = random.uniform(-150, 0)
        goal_x  = random.uniform(200, 350)
        goal_y  = random.uniform(0, 150)
    elif quadrant == 1:
        spawn_x = random.uniform(-100, 50)
        spawn_y = random.uniform(0, 150)
        goal_x  = random.uniform(200, 350)
        goal_y  = random.uniform(-150, 0)
    elif quadrant == 2:
        spawn_x = random.uniform(200, 350)
        spawn_y = random.uniform(0, 150)
        goal_x  = random.uniform(-100, 50)
        goal_y  = random.uniform(-150, 0)
    else:
        spawn_x = random.uniform(200, 350)
        spawn_y = random.uniform(-150, 0)
        goal_x  = random.uniform(-100, 50)
        goal_y  = random.uniform(0, 150)

    goal_z = drone_z + random.uniform(-5, 5)
    dist   = np.sqrt((goal_x-spawn_x)**2 + (goal_y-spawn_y)**2)

    dx = goal_x - spawn_x
    dy = goal_y - spawn_y
    direction = math.atan2(dy, dx)

    if face_goal:
        spawn_yaw = direction - math.pi + random.uniform(-0.3, 0.3)
    else:
        spawn_yaw = direction + random.uniform(-0.3, 0.3)

    # Rocce
    keypoints = [
        (spawn_x, spawn_y, EXCLUSION_RADIUS),
        (goal_x,  goal_y,  EXCLUSION_RADIUS),
    ]
    placed_rocks = []

    def too_close(x, y):
        if any(np.sqrt((x-px)**2 + (y-py)**2) < r for px, py, r in keypoints):
            return True
        if any(np.sqrt((x-rx)**2 + (y-ry)**2) < MIN_ROCK_DIST for rx, ry in placed_rocks):
            return True
        return False

    rocks_sdf = ""
    placed = 0
    attempts = 0

    while placed < N_ROCKS and attempts < 3000:
        attempts += 1
        x = random.uniform(-120, 370)
        y = random.uniform(-170, 170)
        if too_close(x, y):
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
        placed_rocks.append((x, y))
        placed += 1

    if placed < 8:
        print(f"[GEN] WARNING: solo {placed} rocce, scenario non valido")

    # World SDF
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
      <pose>500 0 -65 0 0 0</pose>
      <static>true</static>
    </include>
    
    <!-- Piano fondale per collisioni -->
    <model name="sea_floor_collision">
      <static>true</static>
      <pose>0 0 -98 0 0 0</pose>
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
    print(f"[GEN] Seed: {seed} | {placed} rocce | dist={dist:.0f}m")

    return goal_x, goal_y, goal_z