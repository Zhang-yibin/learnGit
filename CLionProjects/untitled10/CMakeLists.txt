cmake_minimum_required(VERSION 3.28)
project(untitled10)

set(CMAKE_CXX_STANDARD 17)

add_executable(untitled10 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(untitled10  ${OpenCV_LIBS})

