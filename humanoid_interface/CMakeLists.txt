cmake_minimum_required(VERSION 3.5)
project(humanoid_interface)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

file(GLOB ACTION_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "action/*.action")
file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "msg/*.msg")
file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "srv/*.srv")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${ACTION_FILES}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces
)

ament_package()
