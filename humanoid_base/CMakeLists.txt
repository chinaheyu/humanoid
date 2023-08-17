cmake_minimum_required(VERSION 3.5)
project(humanoid_base)

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

# build component
aux_source_directory("src" component_srcs)
list(REMOVE_ITEM component_srcs "src/composition.cpp")
add_library(humanoid_base_component SHARED ${component_srcs})
target_include_directories(humanoid_base_component PUBLIC "include")
target_compile_definitions(humanoid_base_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(humanoid_base_component -lusb-1.0)
ament_target_dependencies(humanoid_base_component ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})

# build a composition node
add_executable(humanoid_base_node "src/composition.cpp")
target_link_libraries(humanoid_base_node humanoid_base_component)
ament_target_dependencies(humanoid_base_node ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})

install(TARGETS
  humanoid_base_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  humanoid_base_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
