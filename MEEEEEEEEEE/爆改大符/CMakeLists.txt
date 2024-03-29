cmake_minimum_required(VERSION 3.17)
project(SR_SDUST)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -pthread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c17 -O3 -march=native -pthread")

find_package(OpenCV 4 REQUIRED)
find_package(fmt REQUIRED)
include_directories("/usr/include/eigen3")

# MVS MVC
include_directories(${PROJECT_SOURCE_DIR}/Hikvision/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/Hikvision/include HIK)

# MVS LIB
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/Hikvisio/MVS/lib)
LINK_LIBRARIES(X11 MvCameraControl)

# Serial
include_directories(${PROJECT_SOURCE_DIR}/Serial)
aux_source_directory(${PROJECT_SOURCE_DIR}/Serial SRL)

add_executable(SR_SDUST main.cpp
        ${HIK} ${DTC} ${CLS} ${PNP} ${SRL}
        ${PRE} ${TRA} ${KF}
        ${RDTC} ${RFLT} ${RPCT} ${PRE}  Hikvision/HaiKangCamera.cpp
        1111.h
        new.cpp
)

target_link_libraries(SR_SDUST ${OpenCV_LIBS} fmt::fmt jsoncpp)