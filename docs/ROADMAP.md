# ROADMAP — Stäubli Digital Twin

---

## Phase 1 — Communication TCP bidirectionnelle ✅ TERMINÉE

**Objectif :** établir et valider la communication entre le robot et ROS2.

### VAL3 (robot)
- [x] `tPosition` — envoi position toutes les 4ms
- [x] `tCommande` — réception commandes joints
- [x] `tRobot` — exécution des movej reçus
- [x] Socket bidirectionnel sur port 2005

### ROS2 (PC)
- [x] Créer le workspace `ros2_ws`
- [x] Créer `staubli_msgs` avec les messages custom
- [x] Créer `staubli_bridge` avec le bridge TCP
- [x] Build et test de publication `/joint_states`

### Résultats validés
- [x] Fréquence stable ~240 Hz (min 4ms, max 5ms, std dev 0.2ms)
- [x] Topics `/joint_states`, `/staubli/tcp_pose`, `/staubli/joint_cmd` actifs
- [x] Envoi commande PC → robot fonctionnel
- [x] Simulateur `tools/robot_simulator.py` validé en remplacement de SRS

---

## Phase 2 — Test avec le vrai robot 🔄 EN COURS

**Objectif :** valider sur le matériel réel (Stäubli RX160L / CS8).

- [ ] Configurer `init.pgx` avec `target = "192.168.0.3"`
- [ ] Vérifier la connectivité réseau robot ↔ PC
- [ ] Déployer le projet VAL3 `numericalTwins` sur le CS8
- [ ] Valider la réception des trames position dans ROS2
- [ ] Tester l'envoi de commandes depuis ROS2 → robot réel
- [ ] Vérifier la détection d'overrun (`nOvrPos`)
- [ ] Visualiser le robot dans RViz avec l'URDF Stäubli RX160L

---

## Phase 3 — Test avec le vrai robot

**Objectif :** valider sur le matériel réel.

- [ ] Configurer `init.pgx` avec `target = "192.168.0.3"`
- [ ] Vérifier la connectivité réseau robot ↔ PC
- [ ] Valider la fréquence de 250 Hz en conditions réelles
- [ ] Tester la boucle commande complète PC → Robot → PC
- [ ] Vérifier la détection d'overrun (`nOvrPos`)

---

## Phase 4 — Simulation process impression 3D

**Objectif :** jumeau numérique complet avec détection et correction de la patte d'éléphant.

### 4a — Collecte données process (`process_bridge`)
- [ ] Identifier le protocole de communication des capteurs
      (Modbus, MQTT, serial, autre...)
- [ ] Implémenter `process_bridge` pour collecter :
      - Température extrudeur
      - Température plateau
      - Vitesse d'extrusion
      - Pyromètre couche n-1
- [ ] Publier `staubli_msgs/ProcessState` sur `/staubli/process_state`

### 4b — Modèle de cordon (`staubli_dt`)
- [ ] Implémenter le modèle de largeur de cordon :
      `width = f(débit, vitesse, temp_n1, temp_plateau)`
- [ ] Calibrer le modèle sur données réelles
- [ ] Publier `staubli_msgs/BeadMeasure` sur `/staubli/bead_measure`

### 4c — Détection patte d'éléphant (`staubli_dt`)
- [ ] Définir le seuil de détection (ex: ratio > 1.15)
- [ ] Implémenter la logique de correction :
      - Augmentation vitesse robot
      - Réduction débit extrusion
- [ ] Publier `staubli_msgs/ProcessCorrection`

### 4d — Visualisation (`staubli_viz`)
- [ ] Afficher le cordon couche par couche (MarkerArray)
- [ ] Colorer le cordon selon le ratio patte d'éléphant :
      - 🟢 Vert  : nominal (ratio < 1.10)
      - 🟡 Jaune : attention (1.10 < ratio < 1.15)
      - 🔴 Rouge : patte d'éléphant détectée (ratio > 1.15)
- [ ] Dashboard RViz avec les métriques process en temps réel

---

## Messages custom à créer (`staubli_msgs`)

| Fichier | Phase | Statut |
|---------|-------|--------|
| `ProcessState.msg` | 4a | ⏳ à faire |
| `BeadMeasure.msg` | 4b | ⏳ à faire |
| `ProcessCorrection.msg` | 4c | ⏳ à faire |

---

## Légende
- ✅ Terminé
- 🔄 En cours
- ⏳ À faire
- ❌ Bloqué
