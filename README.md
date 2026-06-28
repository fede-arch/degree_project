# LRAUV Simulation

This repository contains the libraries, plugins and other files for the simulation of the Tethys-class Long-Range AUV (LRAUV) from the Monterey Bay Aquarium Research Institute (MBARI).

For documentation regarding this repository please refer to the [wiki](https://github.com/osrf/lrauv/wiki).

<p align="center">
  <img width="40%" src="https://raw.githubusercontent.com/wiki/osrf/lrauv/media/LRUAV_3D.gif" alt="LRAUV 3D">
</p>

Source files, models, and plugins relevant to a general audience are upstreamed on an irregular basis to [Gazebo libraries](https://gazebosim.org), the top-level library being [gz-sim](https://github.com/gazebosim/gz-sim). Upstreamed files may eventually be removed from the repository.

Standalone, the repository contains the environment and plugins necessary to simulate an underwater vehicle in Gazebo. Integrated with the real-world LRAUV controller code, the binaries of which are provided to the public on MBARI's DockerHub, the simulated robot can be controlled using the same code executed on the real robot. This enables the validation of scientific missions for oceanography research.

## Citations

> Timothy R. Player, Arjo Chakravarty, Mabel M. Zhang, Ben Yair Raanan, Brian Kieft, Yanwu Zhang, and Brett Hobson, "From Concept to Field Tests: Accelerated Development of Multi-AUV Missions Using a High-Fidelity Faster-than-Real-Time Simulator," in *IEEE International Conference on Robotics and Automation (ICRA)*, May 2023.

---

## VFH 3D Navigation — Degree Project

This fork extends the original MBARI simulation with a 3D Vector Field Histogram (VFH) navigation system for autonomous underwater obstacle avoidance, developed as a Bachelor's thesis at Sapienza Università di Roma.

**Author:** Federico D'Angelo  
**Thesis:** *Implementazione del metodo VFH in ambienti sottomarini* (2025/2026)  
**Supervisor:** Prof. Enrico Tronci

### Repository Structure

```
lrauv_gazebo_plugins/       # Gazebo plugins (C++)
│   ├── src/
│   │   ├── NavigationPlugin.cc   # VFH 3D navigation plugin
│   │   └── HydrodynamicsPlugin.cc
│   └── worlds/
│       ├── navigation_world.sdf         # single AUV world
│       └── navigation_world_multi.sdf   # multi-AUV world

optimization/               # Evolution Strategies parameter tuning
│   ├── es_optimizer.py     # ES main loop
│   ├── es_utils.py         # shared utilities and VFH parameters
│   ├── validate_single.py  # single AUV validation
│   └── validate_multi.py   # multi-AUV validation

scripts/                    # launch scripts
│   ├── generate_world.py   # procedural world generation
│   ├── generate_world_multi.py
│   ├── run.sh              # single AUV simulation
│   ├── run_multi.sh        # multi-AUV simulation
│   └── run_remote.sh       # distributed: Gazebo server side

distributed/                # distributed architecture (Chapter 9)
│   └── standalone_controller.cc  # VFH controller (remote machine)

tools/setup/                # Docker and build utilities
es/results/                 # ES optimization results (JSON)
tesi/                       # thesis plots and figures
```

### Requirements

- Docker with image `lrauv:harmonic` (Gazebo Harmonic)
- Python 3.10+ with `numpy`
- For distributed mode: `gz-harmonic`, `libgz-transport13-dev`, `libgz-msgs10-dev`

Build the Docker image:
```bash
docker build -t lrauv:harmonic tools/setup/
```

### Single AUV Simulation

Run a single navigation episode with a random scenario:
```bash
./scripts/run.sh [seed]
```

### Parameter Optimization (Evolution Strategies)

Optimize VFH parameters over 20 generations with 6 parallel workers:
```bash
cd optimization
python3 es_optimizer.py
```

Results are saved in `es/results/`. To validate optimized parameters on 30 unseen scenarios:
```bash
python3 validate_single.py
```

### Multi-AUV Simulation

Run scenarios with multiple simultaneous vehicles (10, 20, 30, 40 drones):
```bash
./scripts/run_multi.sh [n_drones] [seed]
```

Validate scalability:
```bash
cd optimization
python3 validate_multi.py
```

### Distributed Mode

Run Gazebo on a dedicated machine and the VFH controller on a remote computer over LAN.

**On the server machine:**
```bash
./scripts/run_remote.sh [seed]
```

**Compile the standalone controller on the remote machine:**
```bash
g++ -std=c++17 distributed/standalone_controller.cc -o standalone_controller \
  $(pkg-config --cflags --libs gz-transport13 gz-msgs10)
```

**Run the controller (use goal coordinates printed by run_remote.sh):**
```bash
export GZ_PARTITION=tesi_live
export GZ_IP=<local_ip>
export GZ_RELAY=<server_ip>
./standalone_controller tethys_0 <goal_x> <goal_y> <goal_z>
```