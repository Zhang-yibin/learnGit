cmake_minimum_required(VERSION 3.26)
project(1112)

set(CMAKE_CXX_STANDARD 11)

add_executable(1112 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(1112  ${OpenCV_LIBS})