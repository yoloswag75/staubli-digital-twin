# Protocole de communication TCP — Stäubli ↔ PC

---

## Paramètres réseau

| Paramètre | Valeur |
|-----------|--------|
| Port | `2005` |
| Protocole | TCP (orienté connexion) |
| Rôle robot | CLIENT |
| Rôle PC | SERVEUR |
| Encoding | ASCII |
| Fin de message | `\n` (LF, code 10) |
| Algorithme Nagle | Désactivé (envoi immédiat) |
| Timeout | Infini (-1) |

---

## Trame Robot → PC (position)

Envoyée toutes les **4 ms** (250 Hz) par `tPosition`.

### Format
```
{index;j1;j2;j3;j4;j5;j6;x;y;z;rx;ry;rz}\n
```

### Champs

| Index | Champ | Unité | Description |
|-------|-------|-------|-------------|
| 0 | `index` | entier | Numéro de trame (incrémenté par le robot) |
| 1 | `j1` | degrés | Angle axe 1 (format 10.3) |
| 2 | `j2` | degrés | Angle axe 2 |
| 3 | `j3` | degrés | Angle axe 3 |
| 4 | `j4` | degrés | Angle axe 4 |
| 5 | `j5` | degrés | Angle axe 5 |
| 6 | `j6` | degrés | Angle axe 6 |
| 7 | `x` | mm | Position TCP X (format .3) |
| 8 | `y` | mm | Position TCP Y |
| 9 | `z` | mm | Position TCP Z |
| 10 | `rx` | degrés | Rotation X (Euler) |
| 11 | `ry` | degrés | Rotation Y (Euler) |
| 12 | `rz` | degrés | Rotation Z (Euler) |

### Exemple
```
{1542;    10.000;   -45.500;    90.000;     0.000;    45.000;     0.000;350.123;120.456;800.789;0.000;90.000;0.000}
```

---

## Trame PC → Robot (commande)

Envoyée à la demande via le topic ROS2 `/staubli/joint_cmd`.

### Format
```
{index;j1;j2;j3;j4;j5;j6}\n
```

### Champs

| Index | Champ | Unité | Description |
|-------|-------|-------|-------------|
| 0 | `index` | entier | Numéro de trame (incrémenté par le PC) |
| 1..6 | `j1`..`j6` | degrés | Angles cibles des 6 axes |

### Exemple
```
{42;0.000;-30.000;90.000;0.000;45.000;0.000}
```

---

## Gestion des erreurs

### Côté robot (`tRobot`)
Si `nCmdIndex != nLastIndex + 1` → trame perdue ou désordonnée :
- Log d'erreur via `logMsg()`
- Resynchronisation sur l'index reçu (évite le blocage)

### Côté Python (`staubli_bridge`)
- Envoi protégé par `threading.Lock` (accès concurrent)
- Réception dans un thread séparé (non bloquant pour ROS2)
- Buffer de reconstruction des trames (gestion fragmentation TCP)

---

## Conversions appliquées côté Python

| Donnée | Robot (VAL3) | ROS2 |
|--------|-------------|------|
| Angles joints | degrés | radians (`math.radians`) |
| Position TCP | mm | mètres (÷ 1000) |
| Orientation | Euler degrés | Quaternion (`euler_to_quaternion`) |
