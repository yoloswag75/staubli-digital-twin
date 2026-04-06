# ProcessState.msg — Phase 4a
# État complet du process d'impression à un instant t
# À décommenter dans CMakeLists.txt quand prêt

std_msgs/Header header
float64 extruder_temp        # Température extrudeur (°C)
float64 bed_temp             # Température plateau (°C)
float64 extrusion_speed      # Vitesse d'extrusion (mm³/s)
float64 layer_n1_temp        # Pyromètre couche n-1 (°C)
float64 robot_speed          # Vitesse TCP robot (mm/s)
int32   layer_index          # Numéro de couche courante
