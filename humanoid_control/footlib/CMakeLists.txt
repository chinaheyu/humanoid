cmake_minimum_required(VERSION 3.10)
project(footlib)

set(CMAKE_CXX_STANDARD 17)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

aux_source_directory(src footlib_srcs)
add_library(footlib ${footlib_srcs})
target_include_directories(footlib PRIVATE include)
target_include_directories(footlib PUBLIC wrapper ${Eigen3_INCLUDE_DIRS})
set_target_properties(footlib PROPERTIES POSITION_INDEPENDENT_CODE ON)
