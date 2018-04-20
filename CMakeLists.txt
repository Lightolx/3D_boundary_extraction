cmake_minimum_required(VERSION 3.9)
project(3D_boundary)

set(CMAKE_CXX_STANDARD 11)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(3D_boundary main.cpp triangle.h triangle.cpp)

target_link_libraries(3D_boundary ${OpenCV_LIBS})