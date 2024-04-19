cmake_minimum_required(VERSION 3.26)
project(liantonyu)

set(CMAKE_CXX_STANDARD 11)

add_executable(liantonyu main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(liantonyu ${OpenCV_LIBS})
