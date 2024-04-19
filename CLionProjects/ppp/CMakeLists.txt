cmake_minimum_required(VERSION 3.26)
project(NUM2)

set(CMAKE_CXX_STANDARD 11)

add_executable(NUM2 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(NUM2 ${OpenCV_LIBS})
