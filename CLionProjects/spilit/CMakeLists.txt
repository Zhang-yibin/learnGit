cmake_minimum_required(VERSION 3.26)
project(spilit)

set(CMAKE_CXX_STANDARD 11)

add_executable(spilit main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(spilit  ${OpenCV_LIBS})