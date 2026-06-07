#!/usr/bin/env python3
import random
import math
import re
import os

def generate_target(distance=80.0):
    yaw   = random.uniform(-math.pi, math.pi)
    pitch = random.uniform(-math.pi/4, math.pi/4)
    x = distance * math.cos(pitch) * math.cos(yaw)
    y = distance * math.cos(pitch) * math.sin(yaw)
    z = -abs(distance * math.sin(pitch)) - 2.0
    return x, y, z

def update_sdf_sphere(sdf_path, x, y, z):
    if not os.path.exists(sdf_path):
        return
    with open(sdf_path, "r") as f:
        content = f.read()
    content = re.sub(
        r'(<model name="target_marker">.*?<pose>)[^<]*(</pose>)',
        rf'\g<1>{x:.2f} {y:.2f} {z:.2f} 0 0 0\g<2>',
        content,
        flags=re.DOTALL
    )
    with open(sdf_path, "w") as f:
        f.write(content)

def update_navigation_plugin(x, y, z):
    model_path = "lrauv_description/models/tethys_equipped/model.sdf"
    with open(model_path, "r") as f:
        content = f.read()
    
    content = re.sub(r'<goal_x>[^<]*</goal_x>', f'<goal_x>{x:.2f}</goal_x>', content)
    content = re.sub(r'<goal_y>[^<]*</goal_y>', f'<goal_y>{y:.2f}</goal_y>', content)
    content = re.sub(r'<goal_z>[^<]*</goal_z>', f'<goal_z>{z:.2f}</goal_z>', content)
    
    with open(model_path, "w") as f:
        f.write(content)

if __name__ == "__main__":
    x, y, z = generate_target(80.0)
    print(f"Random target: ({x:.2f}, {y:.2f}, {z:.2f})")

    update_sdf_sphere("lrauv_gazebo_plugins/worlds/navigation_world.sdf", x, y, z)
    update_sdf_sphere("lrauv_gazebo_plugins/worlds/portuguese_ledge.sdf", x, y, z)
    update_navigation_plugin(x, y, z)

    print(f"SDF updated!")
