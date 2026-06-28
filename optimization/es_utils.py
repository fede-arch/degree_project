#!/usr/bin/env python3
"""
Utilities condivise tra es_optimizer.py, validate_single.py e validate_multi.py.
"""
import os
import json
import shutil
import numpy as np

# PATHS
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))

RESULTS_DIR         = os.path.join(PROJECT_DIR, "optimization/results")
TRAINING_DIR        = os.path.join(RESULTS_DIR, "training")
TRAINING_LOGS_DIR   = os.path.join(TRAINING_DIR, "logs")
VAL_SINGLE_DIR      = os.path.join(RESULTS_DIR, "validation_single")
VAL_SINGLE_LOGS_DIR = os.path.join(VAL_SINGLE_DIR, "logs")
VAL_MULTI_DIR       = os.path.join(RESULTS_DIR, "validation_multi")
VAL_MULTI_LOGS_DIR  = os.path.join(VAL_MULTI_DIR, "logs")

RESULTS_FILE = os.path.join(TRAINING_DIR, "es_results.csv")
BEST_FILE    = os.path.join(TRAINING_DIR, "best_theta.json")

TMPL_PATH = os.path.join(PROJECT_DIR, "lrauv_description/models/tethys_equipped/model.sdf.template")
WORLD_PATH = os.path.join(PROJECT_DIR, "lrauv_gazebo_plugins/worlds/navigation_world.sdf")

# PARAMETRI COSTANTI
R_MIN          = 2.5
R_MAX          = 40.0
MAX_FIN_ANGLE  = 0.15
MAX_ITERATIONS = 300000 # 300s
RADIUS_ARRIVED = 4.0

# PARAMETRI VFH
PARAM_NAMES = [
    "gain_steer", "gain_pitch",
    "valley_threshold", "grid_decay",
    "magnitude_a", "smooth_l", "safety_window"
]
THETA_MIN  = np.array([0.3,  0.3,  10.0, 0.90, 10.0, 2, 1])
THETA_MAX  = np.array([2.5,  2.0,  60.0, 0.99, 20.0, 8, 5])
THETA_INIT = np.array([0.8, 0.8, 30.0, 0.980, 15.0, 5, 2])
SCALE      = THETA_MAX - THETA_MIN
WEIGHT_DECAY = 0.01

# FUNZIONI NORMALIZZAZIONE
def norm(theta):
    return (theta - THETA_MIN) / SCALE
 
def denorm(theta_n):
    return THETA_MIN + np.clip(theta_n, 0.0, 1.0) * SCALE

def parse_result(output: str) -> dict:
    """
    Parsea l'output del topic /es/episode_result.
    Ritorna dict con chiavi: tag, path, t, dist.
    """
    for line in output.splitlines():
        low = line.lower()
        for tag in ("arrived", "collision", "timeout"):
            if tag in low:
                result = {"tag": tag, "path": None, "t": None, "dist": None}
                try:
                    if "path=" in low:
                        result["path"] = float(low.split("path=")[1].split(";")[0])
                except ValueError:
                    pass
                try:
                    if ";t=" in low:
                        result["t"] = float(low.split(";t=")[1].split(";")[0].strip('"'))
                except ValueError:
                    pass
                try:
                    if "dist=" in low:
                        result["dist"] = float(low.split("dist=")[1].split(";")[0].strip('"'))
                except ValueError:
                    pass
                return result
    return {"tag": None, "path": None, "t": None, "dist": None}

def write_sdf(model_dir: str, theta: dict, drone_id: int, goal_x: float, goal_y: float, goal_z: float,
              r_max: float, radius_arrived: float, r_active_factor: float = 0.75,):
    """
    Scrive model.sdf a partire dal template, con i parametri forniti.
    Crea la directory e copia model.config se non esiste già.
    """
    os.makedirs(model_dir, exist_ok=True)
 
    src_config = os.path.join(PROJECT_DIR, "lrauv_description/models/tethys_equipped/model.config")
    dst_config = os.path.join(model_dir, "model.config")
    if not os.path.exists(dst_config):
        shutil.copy(src_config, dst_config)
 
    r_active = r_max * r_active_factor
 
    with open(TMPL_PATH) as f:
        content = f.read()
 
    replacements = {
        "__DRONE_ID__":         str(drone_id),
        "__GOAL_X__":           f"{goal_x:.2f}",
        "__GOAL_Y__":           f"{goal_y:.2f}",
        "__GOAL_Z__":           f"{goal_z:.2f}",
        "__GAIN_STEER__":       f"{theta['gain_steer']:.4f}",
        "__GAIN_PITCH__":       f"{theta['gain_pitch']:.4f}",
        "__VALLEY_THRESHOLD__": f"{theta['valley_threshold']:.4f}",
        "__GRID_DECAY__":       f"{theta['grid_decay']:.4f}",
        "__MAGNITUDE_A__":      f"{theta['magnitude_a']:.4f}",
        "__SMOOTH_L__":         f"{int(round(theta['smooth_l']))}",
        "__SAFETY_WINDOW__":    f"{int(round(theta['safety_window']))}",
        "__MAX_FIN_ANGLE__":    str(MAX_FIN_ANGLE),
        "__RADIUS_ARRIVED__":   str(radius_arrived),
        "__MAX_ITERATIONS__":   str(MAX_ITERATIONS),
        "__R_MAX__":            str(r_max),
        "__R_ACTIVE__":         f"{r_active:.2f}",
        "__R_MIN__":            str(R_MIN),
    }
 
    for placeholder, value in replacements.items():
        content = content.replace(placeholder, value)
 
    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(content)

def shaped_reward(result: dict, dist_init: float) -> float:
    """
    Calcola reward da un risultato episodio.
 
    Nota: result["path"] è la lunghezza del percorso percorso, non la distanza
    residua al goal. Il calcolo del progress per collision/timeout è quindi
    un'approssimazione: sottostima il progresso reale se il robot ha aggirato ostacoli.
    """
    tag  = result["tag"]
    path = result["path"]
    t    = result["t"]
 
    MAX_TIME = 300.0
    MAX_PATH = dist_init * 3.0
 
    if tag == "arrived":
        base       = 1.0
        path_bonus = 0.5 * max(0.0, 1.0 - (path / MAX_PATH)) if path else 0.0
        time_bonus = 0.3 * max(0.0, 1.0 - (t   / MAX_TIME)) if t    else 0.0
        return base + path_bonus + time_bonus  # [1.0, 1.8]
 
    if tag == "collision":
        progress = max(0.0, (dist_init - (path or dist_init)) / dist_init)
        return -1.0 + 0.4 * progress  # [-1.0, -0.6]
 
    # timeout o None
    progress = max(0.0, (dist_init - (path or dist_init)) / dist_init)
    return -0.5 + 0.4 * progress  # [-0.5, -0.1]

def save_results(gen: int, theta_real: np.ndarray, mean_reward: float, seed: int):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    write_header = not os.path.exists(RESULTS_FILE)
    with open(RESULTS_FILE, "a") as fh:
        if write_header:
            fh.write("gen,seed,mean_reward," + ",".join(PARAM_NAMES) + "\n")
        vals = ",".join(f"{v:.6f}" for v in theta_real)
        fh.write(f"{gen},{seed},{mean_reward:.4f},{vals}\n")

def save_best(theta_real: np.ndarray, reward: float):
    os.makedirs(RESULTS_DIR, exist_ok=True)
    with open(BEST_FILE, "w") as fh:
        json.dump({
            "reward": float(reward),
            "theta":  dict(zip(PARAM_NAMES, [float(v) for v in theta_real]))
        }, fh, indent=2)        

