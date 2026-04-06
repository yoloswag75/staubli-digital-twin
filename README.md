# Stäubli Digital Twin — Impression 3D Robotisée

Jumeau numérique temps réel d'un robot Stäubli dédié à l'impression 3D par dépôt de matière.
Objectif : visualisation du cordon, détection et correction du phénomène de patte d'éléphant.

---

## Structure du projet

```
staubli_project/
├── val3/                          # Code VAL3 pour le contrôleur Stäubli
│   └── numericalTwins/            # Application principale
│       ├── numericalTwins.pjx     # Fichier projet
│       ├── numericalTwins.dtx     # Variables globales
│       ├── init.pgx               # Initialisation
│       ├── start.pgx              # Point d'entrée (lance les 3 tâches)
│       ├── stop.pgx               # Arrêt
│       ├── tPosition.pgx          # Tâche : envoi position → PC (4ms sync)
│       ├── tCommande.pgx          # Tâche : réception commandes ← PC
│       ├── tRobot.pgx             # Tâche : exécution des movej
│       └── getNum.pgx             # Utilitaire : lecture d'un nombre sur socket
│
├── ros2_ws/                       # Workspace ROS2 Humble
│   └── src/
│       ├── staubli_msgs/          # Messages custom (CMake)
│       ├── staubli_bridge/        # Bridge TCP ↔ ROS2 (Python)
│       ├── process_bridge/        # Collecte capteurs process (Python)
│       ├── staubli_dt/            # Jumeau numérique + détection (Python)
│       └── staubli_viz/           # Visualisation RViz markers (Python)
│
└── docs/
    ├── ROADMAP.md                 # Phases du projet
    └── protocol.md                # Documentation du protocole TCP
```

---

## Protocole de communication TCP

| Direction | Format | Fréquence |
|-----------|--------|-----------|
| Robot → PC | `{index;j1;j2;j3;j4;j5;j6;x;y;z;rx;ry;rz}\n` | 4 ms (250 Hz) |
| PC → Robot | `{index;j1;j2;j3;j4;j5;j6}\n` | À la demande |

- Unités joints : **degrés**
- Unités position : **mm** et **degrés**
- Séparateur : `;`  |  Début trame : `{`  |  Fin trame : `}`  |  Fin message : `\n` (LF)

---

## Topics ROS2

| Topic | Type | Direction |
|-------|------|-----------|
| `/joint_states` | `sensor_msgs/JointState` | bridge → RViz |
| `/staubli/tcp_pose` | `geometry_msgs/PoseStamped` | bridge → DT |
| `/staubli/joint_cmd` | `sensor_msgs/JointState` | DT → bridge → robot |
| `/staubli/process_state` | `staubli_msgs/ProcessState` | process_bridge → DT |
| `/staubli/bead_measure` | `staubli_msgs/BeadMeasure` | DT → viz |
| `/staubli/correction` | `staubli_msgs/ProcessCorrection` | DT → process_bridge |

---

## Installation et build

```bash
# Cloner / copier le projet
cd ~/staubli_project/ros2_ws

# Sourcer ROS2 Humble
source /opt/ros/humble/setup.bash

# Build dans l'ordre (messages d'abord)
colcon build --packages-select staubli_msgs
colcon build --packages-select staubli_bridge process_bridge staubli_dt staubli_viz

# Sourcer le workspace
source install/setup.bash
```

---

## Lancement

```bash
# Phase 1 & 2 : bridge seul (test SRS ou vrai robot)
ros2 run staubli_bridge bridge

# Phase 3+ : lancement complet
ros2 launch staubli_viz full_dt.launch.py
```

---

## Configuration réseau

| Contexte | IP cible dans `init.pgx` |
|----------|--------------------------|
| Test SRS (même machine) | `127.0.0.1` |
| Vrai robot | `192.168.0.3` |
| Port TCP | `2005` |
