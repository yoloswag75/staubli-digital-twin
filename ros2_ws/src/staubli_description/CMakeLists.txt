cmake_minimum_required(VERSION 3.8)
project(staubli_description)

find_package(ament_cmake REQUIRED)

# Installer tous les fichiers de données
install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
