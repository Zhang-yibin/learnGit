cmake_minimum_required(VERSION 3.26)
project(untitled23)

set(CMAKE_CXX_STANDARD 17)

add_executable(untitled23 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(untitled23 ${OpenCV_LIBS})
