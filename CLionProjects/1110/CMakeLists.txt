cmake_minimum_required(VERSION 3.26)
project(1110)

set(CMAKE_CXX_STANDARD 17)

add_executable(1110 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(1110 ${OpenCV_LIBS})
