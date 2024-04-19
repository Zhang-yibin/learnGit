cmake_minimum_required(VERSION 3.28)
project(untitled11)

set(CMAKE_CXX_STANDARD 17)

add_executable(untitled11 main.cpp
        matplotlibcpp.h)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
include_directories("/usr/include/python3.10")
target_link_libraries(untitled11 ${Python3_LIBRARIES})
