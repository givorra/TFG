cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(box_counting)

find_package(PCL 1.7.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(src)

set  (project_SOURCES main.cpp)

add_executable(box_counting ${project_SOURCES})
target_link_libraries (box_counting ${PCL_LIBRARIES})
