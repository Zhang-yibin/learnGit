cmake_minimum_required(VERSION 3.26)
project(Change_color)

set(CMAKE_CXX_STANDARD 17)

add_executable(Change_color main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(Change_color ${OpenCV_LIBS})
