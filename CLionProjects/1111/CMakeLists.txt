cmake_minimum_required(VERSION 3.26)
project(1111)

set(CMAKE_CXX_STANDARD 11)

add_executable(1111 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(1111  ${OpenCV_LIBS})