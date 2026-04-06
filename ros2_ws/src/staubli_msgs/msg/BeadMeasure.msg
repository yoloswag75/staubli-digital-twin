# BeadMeasure.msg — Phase 4b
# Mesure et estimation du cordon déposé

std_msgs/Header header
float64 width_theoretical    # Largeur théorique GCode (mm)
float64 width_estimated      # Largeur estimée par le modèle (mm)
float64 elephant_foot_ratio  # width_estimated / width_theoretical
bool    anomaly_detected     # true si ratio > seuil
int32   layer_index
