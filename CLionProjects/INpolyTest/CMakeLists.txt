cmake_minimum_required(VERSION 3.26)
project(INpolyTest)

set(CMAKE_CXX_STANDARD 11)

add_executable(INpolyTest main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(INpolyTest ${OpenCV_LIBS})
