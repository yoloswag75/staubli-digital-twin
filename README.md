# Stäubli Digital Twin — Impression 3D Robotisée

Jumeau numérique temps réel d'un robot Stäubli RX160L (CS8) dédié à l'impression 3D
par dépôt de matière. Objectif : visualisation du cordon, détection et correction
du phénomène de patte d'éléphant.

---

## Structure du projet

```
staubli_project/
├── val3/                          # Code VAL3 pour le contrôleur CS8
│   └── numericalTwins/
│       ├── numericalTwins.pjx     # Fichier projet
│       ├── numericalTwins.dtx     # Variables globales (file 5 points, ACK...)
│       ├── init.pgx               # Initialisation socket + file
│       ├── start.pgx              # Lance les 3 tâches
│       ├── stop.pgx               # Arrêt
│       ├── tPosition.pgx          # Envoi position → PC (4ms sync)
│       ├── tCommande.pgx          # Réception commandes ← PC (file circulaire)
│       ├── tRobot.pgx             # Exécution movej/movel + envoi ACK
│       └── getNum.pgx             # Utilitaire lecture nombre sur socket
│
├── ros2_ws/                       # Workspace ROS2 Humble
│   └── src/
│       ├── staubli_msgs/          # Messages custom (CMake) — Phase 4
│       ├── staubli_bridge/        # Bridge TCP ↔ ROS2
│       ├── staubli_description/   # URDF + meshes RX160L + launch RViz
│       ├── process_bridge/        # Capteurs process — Phase 4
│       ├── staubli_dt/            # Jumeau numérique — Phase 4
│       ├── staubli_viz/           # Visualisation RViz — Phase 4
│       └── staubli_tests/         # Tests de validation robot réel
│
├── tools/
│   ├── robot_simulator.py         # Simule le robot (remplace SRS)
│   ├── dtx_player.py              # Joue une trajectoire .dtx en CLI
│   └── dtx_gui.py                 # Interface graphique DTX player
│
├── config/
│   ├── config.yaml                # Config locale (ignorée par git)
│   └── config.yaml.example        # Template de configuration
│
├── docs/
│   ├── ROADMAP.md                 # Phases du projet
│   └── protocol.md                # Documentation protocole TCP
│
├── install.sh                     # Installation automatique
├── launch.sh                      # Lancement de tous les composants
└── requirements.txt               # Dépendances Python
```

---

## Protocole de communication TCP

| Direction | Format | Fréquence |
|-----------|--------|-----------|
| Robot → PC (position) | `{index;j1;j2;j3;j4;j5;j6;x;y;z;rx;ry;rz}\n` | 4ms (250Hz) |
| Robot → PC (ACK) | `{ACK;index_execute;n_restant}\n` | Après chaque point |
| PC → Robot (commande) | `{index;type;x;y;z;rx;ry;rz;vel}\n` | Sur demande |

**Champs commande :**
- `type` : 0 = movej, 1 = movel
- `x,y,z` : position en mm (repère world)
- `rx,ry,rz` : orientation en degrés (Euler)
- `vel` : vitesse TCP en mm/s

**ACK :**
- `index_execute` : index du point que le robot vient d'exécuter
- `n_restant` : points encore dans la file VAL3 (refill si ≤ 2)

---

## Architecture file circulaire

```
PC                              VAL3 (file 5 points)
──                              ────────────────────
Envoie 5 points         ──►    tCommande remplit la file
                                tRobot exécute point #1
                        ◄──    {ACK;1;4}
                                tRobot exécute point #2
                        ◄──    {ACK;2;3}
                                tRobot exécute point #3
                        ◄──    {ACK;3;2}  ← n_restant=2
Refill : 5 nouveaux     ──►    tCommande remplit la file
points envoyés                  ...
```

---

## Topics ROS2

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/joint_states` | `JointState` | bridge → RViz | Angles joints (rad) |
| `/staubli/tcp_pose` | `PoseStamped` | bridge → DT | Pose cartésienne TCP |
| `/staubli/cart_cmd` | `PoseStamped` | GUI → bridge | Commande cartésienne |
| `/staubli/joint_cmd` | `JointState` | → bridge | Commande articulaire |
| `/staubli/cmd_ack` | `JointState` | bridge → GUI | ACK point exécuté |

---

## Installation

```bash
# Cloner le repo
git clone https://github.com/yoloswag75/staubli-digital-twin.git
cd staubli-digital-twin

# Installation automatique
bash install.sh

# OU installation manuelle
sudo apt install python3-tk ros-humble-robot-state-publisher ros-humble-rviz2
pip install -r requirements.txt
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select staubli_msgs
colcon build --packages-select staubli_bridge staubli_description
source install/setup.bash
```

### Configuration réseau

Copier et éditer `config/config.yaml` :

```bash
cp config/config.yaml.example config/config.yaml
nano config/config.yaml
```

| Contexte | `robot_ip` dans `config.yaml` | `target` dans `init.pgx` |
|----------|-------------------------------|--------------------------|
| Simulateur | `127.0.0.1` | `127.0.0.1` |
| Vrai robot | `192.168.X.X` | IP du PC |
| Port TCP | `2005` | `2005` |

---

## Lancement rapide

```bash
# Tout lancer en une commande
bash launch.sh sim    # avec simulateur
bash launch.sh robot  # avec vrai robot
```

### Lancement manuel

```bash
# Terminal 1 — Bridge TCP ↔ ROS2
ros2 run staubli_bridge bridge

# Terminal 2 — Simulateur (remplace le vrai robot)
python3 tools/robot_simulator.py

# Terminal 3 — Visualisation RViz
ros2 launch staubli_description display.launch.py

# Terminal 4 — Interface graphique DTX Player
python3 tools/dtx_gui.py
```

### DTX Player en ligne de commande

```bash
# Dry-run (affiche les points sans envoyer)
python3 tools/dtx_player.py fichier.dtx --dry-run

# Lancer la trajectoire à 50mm/s
python3 tools/dtx_player.py fichier.dtx --vel 50
```

---

## Tests de validation (robot réel)

Les tests sont dans `ros2_ws/src/staubli_tests/`.
Voir `ros2_ws/src/staubli_tests/README.md` pour la procédure complète.

### Test 1 — Réception position + RViz

Vérifie que les positions du robot arrivent correctement dans ROS2.
**Aucun mouvement envoyé.**

```bash
python3 ros2_ws/src/staubli_tests/test_1_position_rviz.py --duration 20
```

Critères PASS : `/joint_states` > 200Hz, robot animé dans RViz.

### Test 2 — Petit déplacement X,Y

Déplace le robot de 20mm en X puis Y à 5mm/s.
**Libérer l'espace autour du robot avant de lancer.**

```bash
# Dry-run d'abord
python3 ros2_ws/src/staubli_tests/test_2_small_move.py --dry-run

# Exécution réelle
python3 ros2_ws/src/staubli_tests/test_2_small_move.py --amplitude 20 --velocity 5
```

### Test 3 — Trajectoire complète

Exécute une trajectoire DTX complète sur le robot réel.
**Vérifier l'espace de travail complet.**

```bash
# Dry-run
python3 ros2_ws/src/staubli_tests/test_3_full_trajectory.py fichier.dtx --dry-run

# Test partiel (50 premiers points)
python3 ros2_ws/src/staubli_tests/test_3_full_trajectory.py fichier.dtx --vel 5 --max-points 50

# Trajectoire complète
python3 ros2_ws/src/staubli_tests/test_3_full_trajectory.py fichier.dtx --vel 10
```

---

## Roadmap

| Phase | Statut | Description |
|-------|--------|-------------|
| 1 — Communication TCP | ✅ | Bridge ROS2, simulateur, file+ACK |
| 2 — Test robot réel | 🔄 | Tests unitaires, validation CS8 |
| 3 — Capteurs process | ⏳ | Temp, débit, pyromètre |
| 4 — Jumeau numérique | ⏳ | Modèle cordon, patte d'éléphant |
