cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(fractal_generator)

find_package(PCL 1.7.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp)

add_executable(fractal_generator ${project_SOURCES})
target_link_libraries (fractal_generator ${PCL_LIBRARIES})
