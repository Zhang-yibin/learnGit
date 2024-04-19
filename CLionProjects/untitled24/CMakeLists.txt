cmake_minimum_required(VERSION 3.26)
project(untitled24)

set(CMAKE_CXX_STANDARD 11)

add_executable(untitled24 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(untitled24 ${OpenCV_LIBS})
