cmake_minimum_required(VERSION 3.26)
project(STATIC_IMAGE_PROCESSING)

set(CMAKE_CXX_STANDARD 11)

add_executable(STATIC_IMAGE_PROCESSING main.cpp)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(STATIC_IMAGE_PROCESSING  ${OpenCV_LIBS})