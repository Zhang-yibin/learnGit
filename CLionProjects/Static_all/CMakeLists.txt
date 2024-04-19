cmake_minimum_required(VERSION 3.26)
project(1116)

set(CMAKE_CXX_STANDARD 11)

add_executable(1116 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(1116 ${OpenCV_LIBS})
