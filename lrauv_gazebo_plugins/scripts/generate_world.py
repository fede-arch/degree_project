import random
import numpy as np
import os
import re

ROCK_Z_BASE = -100.0
EXCLUSION_RADIUS = 27.0
MIN_ROCK_DIST    = 20.0

ROCK_MODELS = [
    "falling rock 1",
    "falling rock 2",
]

def generate_world(project_dir, seed=None):
    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)

    # Spawn tethys casuale
    spawn_x = random.uniform(-20, 20)
    spawn_y = random.uniform(-20, 20)
    spawn_yaw = random.uniform(0, 6.28)
    drone_z   = -75.0 + random.uniform(-10, 10)

    # Goal casuale, lontano dallo spawn
    for _ in range(100):
        goal_x = random.uniform(60, 200)
        goal_y = random.uniform(-80, 80)
        if np.sqrt((goal_x-spawn_x)**2 + (goal_y-spawn_y)**2) > 80.0:
            break
    goal_z = -75.0 + random.uniform(-10, 10)

    keypoints = [(spawn_x, spawn_y), (goal_x, goal_y)]

    def too_close_to_keypoints(x, y):
        for px, py in keypoints:
            if np.sqrt((x-px)**2 + (y-py)**2) < EXCLUSION_RADIUS:
                return True
        return False

    def too_close_to_rocks(x, y, positions):
        for px, py in positions:
            if np.sqrt((x-px)**2 + (y-py)**2) < MIN_ROCK_DIST:
                return True
        return False

    # Rocce lontane da spawn e goal 
    N_ROCKS = 20
    rocks_sdf = ""
    placed = 0
    attempts = 0
    placed_positions = []

    while placed < N_ROCKS and attempts < 1000:
        attempts += 1
        x = random.uniform(-30, 180)
        y = random.uniform(-70, 70)
        if too_close_to_keypoints(x, y):
            continue
        if too_close_to_rocks(x, y, placed_positions):
            continue

        model    = random.choice(ROCK_MODELS)
        yaw      = random.uniform(0, 3.14)
        z_offset = random.uniform(0, 3.0)

        rocks_sdf += f"""
  <include>
    <name>rock_{placed}</name>
    <uri>{model}</uri>
    <pose>{x:.2f} {y:.2f} {ROCK_Z_BASE + z_offset:.2f} 0 0 {yaw:.2f}</pose>
    <static>true</static>
  </include>
"""
        placed_positions.append((x, y))
        placed += 1

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

    # Aggiorna goal nel model.sdf template
    template_path = os.path.join(project_dir,
        "lrauv_description/models/tethys_equipped/model.sdf.template")
    if os.path.exists(template_path):
        with open(template_path, "r") as f:
            content = f.read()
        content = re.sub(r'<goal_x>.*?</goal_x>', f'<goal_x>{goal_x:.2f}</goal_x>', content)
        content = re.sub(r'<goal_y>.*?</goal_y>', f'<goal_y>{goal_y:.2f}</goal_y>', content)
        content = re.sub(r'<goal_z>.*?</goal_z>', f'<goal_z>{goal_z:.2f}</goal_z>', content)
        with open(template_path, "w") as f:
            f.write(content)

    print(f"[GEN] World: {placed} rocce | spawn=({spawn_x:.1f},{spawn_y:.1f},{drone_z:.1f}) yaw={spawn_yaw:.2f} | goal=({goal_x:.1f},{goal_y:.1f},{goal_z:.1f})")