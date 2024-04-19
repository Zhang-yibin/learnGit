cmake_minimum_required(VERSION 3.26)
project(0125)

set(CMAKE_CXX_STANDARD 11)

add_executable(0125 main.cpp
        dec.h)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(0125 ${OpenCV_LIBS})
















