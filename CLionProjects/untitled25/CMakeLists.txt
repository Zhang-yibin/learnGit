cmake_minimum_required(VERSION 3.26)
project(untitled25)

set(CMAKE_CXX_STANDARD 17)

add_executable(untitled25 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(untitled25 ${OpenCV_LIBS})
