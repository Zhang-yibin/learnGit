cmake_minimum_required(VERSION 3.27)
project(RAW)

set(CMAKE_CXX_STANDARD 17)

add_executable(RAW main.cpp
        dec.h)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(RAW  ${OpenCV_LIBS})
