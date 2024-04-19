cmake_minimum_required(VERSION 3.26)
project(rEd_point)

set(CMAKE_CXX_STANDARD 11)

add_executable(rEd_point main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(rEd_point ${OpenCV_LIBS})
