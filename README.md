# Navigazione Autonoma di AUV con VFH 3D

Tesi di Laurea Triennale — Sapienza Università di Roma, 2025/2026  
**Autore:** Federico D'Angelo | **Relatore:** Prof. Enrico Tronci

Estensione della simulazione MBARI LRAUV con navigazione autonoma basata su Vector Field Histogram 3D e ottimizzazione dei parametri tramite Evolution Strategies.

---

## Requisiti

- Docker con immagine `lrauv:harmonic`
- Python 3.10+ con `numpy`

```bash
docker build -t lrauv:harmonic tools/setup/
```

---

## Utilizzo

Tutte le operazioni vanno eseguite dalla root del repository.

**Simulazione singolo AUV:**
```bash
./scripts/run.sh [seed]
```

**Simulazione multi-AUV:**
```bash
./scripts/run_multi.sh [n_drones] [seed]
```

**Ottimizzazione parametri VFH:**
```bash
cd optimization && python3 es_optimizer.py
```

**Validazione:**
```bash
cd optimization && python3 validate_single.py
cd optimization && python3 validate_multi.py
```

---

## Modalità Distribuita

Gazebo su server, controller VFH su macchina remota via LAN.

```bash
# Sul server
./scripts/run_remote.sh [seed]

# Compila sulla macchina remota
g++ -std=c++17 distributed/standalone_controller.cc -o standalone_controller \
  $(pkg-config --cflags --libs gz-transport13 gz-msgs10)

# Avvia (usa le coordinate stampate da run_remote.sh)
export GZ_PARTITION=tesi_live
export GZ_IP=<ip_locale>
export GZ_RELAY=<ip_server>
./standalone_controller tethys_0 <goal_x> <goal_y> <goal_z>
```

---

*Fork di [mbari-org/gz_lrauv](https://github.com/mbari-org/gz_lrauv)*