cmake_minimum_required(VERSION 3.5)
project(humanoid_control)

# C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory("footlib")

# build component
aux_source_directory("src" component_srcs)
list(REMOVE_ITEM component_srcs "src/composition.cpp")
add_library(humanoid_control_component SHARED ${component_srcs})
target_include_directories(humanoid_control_component PUBLIC "include")
target_compile_definitions(humanoid_control_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(humanoid_control_component footlib)
ament_target_dependencies(humanoid_control_component ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
rclcpp_components_register_nodes(humanoid_control_component "humanoid::HumanoidControlNode")

# build a composition node
add_executable(humanoid_control_node "src/composition.cpp")
target_link_libraries(humanoid_control_node humanoid_control_component)
ament_target_dependencies(humanoid_control_node ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})

install(TARGETS
  humanoid_control_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  humanoid_control_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
