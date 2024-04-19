make_minimum_required(VERSION 3.22.1)
project(opencv_test)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV)
include_directories(${OpenCV})

add_executable(opencv_test src/main.cpp
        opencv_test.cpp
        opencv_test.h
        opencv_test.h)
target_link_libraries(opencv_test ${OpenCV_LIBS})
