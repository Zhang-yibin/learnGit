cmake_minimum_required(VERSION 3.26)
project(CANNY_select)

set(CMAKE_CXX_STANDARD 11)

add_executable(CANNY_select main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(CANNY_select ${OpenCV_LIBS})
