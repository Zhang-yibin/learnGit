cmake_minimum_required(VERSION 3.26)
project(1029)

set(CMAKE_CXX_STANDARD 11)

add_executable(1029 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(1029 ${OpenCV_LIBS})