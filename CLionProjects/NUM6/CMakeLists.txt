cmake_minimum_required(VERSION 3.26)
project(NUM6)

set(CMAKE_CXX_STANDARD 11)

add_executable(NUM6 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(NUM6 ${OpenCV_LIBS})