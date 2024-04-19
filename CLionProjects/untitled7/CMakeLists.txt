cmake_minimum_required(VERSION 3.26)
project(11_1)

set(CMAKE_CXX_STANDARD 11)

add_executable(untitled7 main.cpp
        yeeyeye.h

)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(untitled7 ${OpenCV_LIBS})
















