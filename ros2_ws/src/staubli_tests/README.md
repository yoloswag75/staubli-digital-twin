# Tests de validation — Robot Stäubli RX160L

## Ordre d'execution recommande

```
Test 1 → Test 2 → Test 3
```

Ne pas passer au test suivant si le precedent echoue.

---

## Pre-requis communs

```bash
# Terminal 1 : Bridge TCP
ros2 run staubli_bridge bridge

# Terminal 2 : RViz
ros2 launch staubli_description display.launch.py
```

---

## Test 1 — Reception position + RViz

**Objectif** : Verifier que les positions du robot arrivent correctement dans ROS2.

```bash
python3 test_1_position_rviz.py --duration 20
```

**Criteres PASS** :
- /joint_states > 200Hz
- /tcp_pose > 200Hz
- Robot anime dans RViz
- Aucune erreur de limites articulaires

---

## Test 2 — Petit deplacement X,Y

**Objectif** : Valider l'envoi de commandes au robot reel.

**SECURITE** : Liberer l'espace autour du robot sur 20mm minimum.

```bash
# Dry-run d'abord (aucun mouvement)
python3 test_2_small_move.py --dry-run

# Execution reelle
python3 test_2_small_move.py --amplitude 20 --velocity 5
```

**Criteres PASS** :
- 4 ACK recus
- Position finale = position initiale (tolerance 2mm)

---

## Test 3 — Trajectoire complete

**Objectif** : Valider l'execution d'une trajectoire DTX complete.

**SECURITE** : Verifier l'espace de travail complet de la trajectoire.

```bash
# Dry-run
python3 test_3_full_trajectory.py fichier.dtx --dry-run

# Test partiel (50 premiers points)
python3 test_3_full_trajectory.py fichier.dtx --vel 5 --max-points 50

# Trajectoire complete
python3 test_3_full_trajectory.py fichier.dtx --vel 10
```

**Criteres PASS** :
- Tous les ACK recus
- File jamais vide (queue_empties = 0)
- Duree coherente avec vitesse et distance

---

## En cas d'echec

| Symptome | Cause probable | Action |
|----------|----------------|--------|
| /joint_states = 0Hz | Bridge non connecte | Verifier cable reseau |
| /joint_states < 100Hz | Surcharge reseau | Verifier tPosition.pgx |
| ACK non recus | tRobot.pgx non actif | Verifier programme VAL3 |
| Robot ne bouge pas | File pleine | Reduire queue_size |
| Position incorrecte | Probleme repere | Verifier init.pgx target IP |
