# ProcessCorrection.msg — Phase 4c
# Correction à appliquer sur le process

std_msgs/Header header
float64 speed_correction       # Facteur multiplicatif vitesse robot (1.0 = pas de correction)
float64 extrusion_correction   # Facteur multiplicatif débit extrusion
bool    correction_active      # true si une correction est appliquée
string  reason                 # Description de la raison
