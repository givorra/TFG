cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(box_counting)

find_package(PCL 1.7.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/lib)
include_directories(${CMAKE_SOURCE_DIR}/src)

set  (project_HEADERS include/box_counting.h include/util.h include/linear_regression.h)
set  (project_SOURCES src/main.cpp lib/box_counting.cpp lib/util.cpp lib/linear_regression.cpp)

add_executable(box_counting ${project_SOURCES} ${project_HEADERS})
target_link_libraries(box_counting ${PCL_LIBRARIES})
