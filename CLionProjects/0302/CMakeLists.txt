cmake_minimum_required(VERSION 3.27)
project(0302)

set(CMAKE_CXX_STANDARD 17)

add_executable(0302 main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(0302 ${OpenCV_LIBS})

