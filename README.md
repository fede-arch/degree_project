# Simulazione LRAUV — Tesi di Laurea Triennale

Questo repository estende la simulazione originale MBARI con un sistema di navigazione autonoma basato su **Vector Field Histogram 3D (VFH)** per l'elusione degli ostacoli in ambienti sottomarini.

**Autore:** Federico D'Angelo  
**Tesi:** *Navigazione autonoma di AUV con VFH 3D in ambiente simulato*  
**Relatore:** Prof. Enrico Tronci  
**Università:** Sapienza Università di Roma, 2025/2026

<p align="center">
  <img width="40%" src="https://raw.githubusercontent.com/wiki/osrf/lrauv/media/LRUAV_3D.gif" alt="LRAUV 3D">
</p>

---

## Struttura del Repository

```
lrauv_gazebo_plugins/
├── src/
│   ├── NavigationPlugin.cc        # plugin VFH 3D (Gazebo)
│   └── HydrodynamicsPlugin.cc
└── worlds/
    ├── navigation_world.sdf       # scenario singolo AUV
    └── navigation_world_multi.sdf # scenario multi-AUV

optimization/
├── es_optimizer.py                # loop principale Evolution Strategies
├── es_utils.py                    # parametri VFH e utilities condivise
├── validate_single.py             # validazione singolo AUV
├── validate_multi.py              # validazione multi-AUV
└── results/
    ├── training/                  # risultati training ES
    ├── validation_single/         # risultati validazione singolo
    └── validation_multi/          # risultati validazione multi

scripts/
├── generate_world.py              # generazione procedurale scenario
├── generate_world_multi.py        # generazione scenario multi-AUV
├── run.sh                         # avvio simulazione singolo AUV
├── run_multi.sh                   # avvio simulazione multi-AUV
└── run_remote.sh                  # avvio server Gazebo (modalità distribuita)

distributed/
└── standalone_controller.cc       # controller VFH standalone (macchina remota)

tools/setup/
├── setup_run.py                   # preparazione world e parametri
├── build_plugin.sh                # compilazione plugin
└── Dockerfile                     # immagine Docker lrauv:harmonic
```

---

## Requisiti

- Docker con immagine `lrauv:harmonic` (Gazebo Harmonic)
- Python 3.10+ con `numpy`
- Per modalità distribuita: `gz-harmonic`, `libgz-transport13-dev`, `libgz-msgs10-dev`

Costruisci l'immagine Docker:
```bash
docker build -t lrauv:harmonic tools/setup/
```

---

## Simulazione Singolo AUV

Avvia un episodio di navigazione con scenario casuale:
```bash
./scripts/run.sh [seed]
```

---

## Ottimizzazione Parametri (Evolution Strategies)

Ottimizza i parametri VFH su 20 generazioni con 10 worker paralleli:
```bash
cd optimization
python3 es_optimizer.py
```

Valida i parametri ottimizzati su 30 scenari mai visti:
```bash
python3 validate_single.py
```

---

## Simulazione Multi-AUV

Avvia scenari con più veicoli simultanei:
```bash
./scripts/run_multi.sh [n_drones] [seed]
```

Valida la scalabilità:
```bash
python3 validate_multi.py
```

---

## Modalità Distribuita

Esegui Gazebo su una macchina dedicata e il controller VFH su un computer remoto via LAN.

**Sul server:**
```bash
./scripts/run_remote.sh [seed]
```

**Compila il controller standalone sulla macchina remota:**
```bash
g++ -std=c++17 distributed/standalone_controller.cc -o standalone_controller \
  $(pkg-config --cflags --libs gz-transport13 gz-msgs10)
```

**Avvia il controller (usa le coordinate goal stampate da run_remote.sh):**
```bash
export GZ_PARTITION=tesi_live
export GZ_IP=<ip_locale>
export GZ_RELAY=<ip_server>
./standalone_controller tethys_0 <goal_x> <goal_y> <goal_z>
```

---

## Citazione Repository Originale

> Timothy R. Player et al., "From Concept to Field Tests: Accelerated Development of Multi-AUV Missions Using a High-Fidelity Faster-than-Real-Time Simulator," *ICRA 2023*.