cmake_minimum_required(VERSION 3.10)
project(foot_kinematics_lib)

set(CMAKE_CXX_STANDARD 17)

aux_source_directory(src foot_kinematics_lib_srcs)
add_library(foot_kinematics_lib ${foot_kinematics_lib_srcs})
target_include_directories(foot_kinematics_lib PRIVATE include)
target_include_directories(foot_kinematics_lib PUBLIC wrapper)
