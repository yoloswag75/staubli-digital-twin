# Contribuer au projet

## Branches

| Branche | Rôle |
|---------|------|
| `main` | Code stable, testé |
| `dev` | Développement en cours |
| `phase/X-nom` | Branche dédiée à une phase (ex: `phase/2-srs-test`) |

## Workflow recommandé

```bash
# Cloner le repo
git clone https://github.com/TON_USERNAME/staubli-digital-twin.git
cd staubli-digital-twin

# Créer une branche pour ta phase
git checkout -b phase/2-srs-test

# Travailler...

# Commiter
git add .
git commit -m "phase2: validation com TCP avec SRS"

# Pousser
git push origin phase/2-srs-test
```

## Convention de commits

```
phase1: description   → travaux Phase 1 (VAL3 & Com)
phase2: description   → travaux Phase 2 (tests SRS/robot)
phase3: description   → travaux Phase 3 (capteurs process)
phase4: description   → travaux Phase 4 (DT & visualisation)
fix: description      → correction de bug
doc: description      → documentation uniquement
ros2: description     → modifications packages ROS2
val3: description     → modifications code VAL3
```

## Structure du projet

Voir [README.md](README.md) pour la structure complète et les instructions de build.
