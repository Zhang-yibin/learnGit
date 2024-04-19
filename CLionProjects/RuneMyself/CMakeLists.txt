cmake_minimum_required(VERSION 3.26)
project(RuneMyself)

set(CMAKE_CXX_STANDARD 11)
add_executable(RuneMyself main.cpp
        dec.h)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(RuneMyself  ${OpenCV_LIBS})

