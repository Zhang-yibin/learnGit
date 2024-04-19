cmake_minimum_required(VERSION 3.26)
project(untitled5)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(untitled5 main.cpp)
target_link_libraries(untitled5 ${OpenCV_LIBS})
