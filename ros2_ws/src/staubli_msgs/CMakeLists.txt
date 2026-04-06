cmake_minimum_required(VERSION 3.8)
project(staubli_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Les messages seront ajoutés en Phase 4
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/ProcessState.msg"
#   "msg/BeadMeasure.msg"
#   "msg/ProcessCorrection.msg"
#   DEPENDENCIES std_msgs
# )

ament_package()
