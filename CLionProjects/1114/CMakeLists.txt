cmake_minimum_required(VERSION 3.26)
project(1114)

set(CMAKE_CXX_STANDARD 11)

add_executable(1114 main.cpp
        shitttt.h
        paramet.h
        supervis.h)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(1114 ${OpenCV_LIBS})
