cmake_minimum_required(VERSION 3.17)
project(SR_SDUST)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native -pthread")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c17 -O2 -march=native -pthread")
set(HIKVISION_CAMERA_DIR "${PROJECT_SOURCE_DIR}/hikvision") # 设置海康相机SDK工具包目录
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED COMPONENTS EigenSparse)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
include_directories("/usr/include/eigen3")

# MVS MVC
include_directories(${PROJECT_SOURCE_DIR}/Hikvision/include)
aux_source_directory(${PROJECT_SOURCE_DIR}/Hikvision/include HIK)

# MVS LIB
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/Hikvisio/MVS/lib)
LINK_LIBRARIES(X11 MvCameraControl)

# Classifier
include_directories(${PROJECT_SOURCE_DIR}/Autoaim/Classifier)
aux_source_directory(${PROJECT_SOURCE_DIR}/Autoaim/Classifier CLS)

# Detector
include_directories(${PROJECT_SOURCE_DIR}/Autoaim)
include_directories(${PROJECT_SOURCE_DIR}/Autoaim/Detector)
aux_source_directory(${PROJECT_SOURCE_DIR}/Autoaim/Detector DTC)

# Rune
include_directories(${PROJECT_SOURCE_DIR}/Rune/Detector)
aux_source_directory(${PROJECT_SOURCE_DIR}/Rune/Detector RDTC)

# Rune
include_directories(${PROJECT_SOURCE_DIR}/Rune/Predict)
aux_source_directory(${PROJECT_SOURCE_DIR}/Rune/Predict RPCT)

# Rune
include_directories(${PROJECT_SOURCE_DIR}/Rune/Filter)
aux_source_directory(${PROJECT_SOURCE_DIR}/Rune/Filter RFLT)


# Pnpsolver
include_directories(${PROJECT_SOURCE_DIR}/Autoaim/Pnpsolver)
aux_source_directory(${PROJECT_SOURCE_DIR}/Autoaim/Pnpsolver PNP)

# ArmorPredict
include_directories(${PROJECT_SOURCE_DIR}/Autoaim/ArmorPredict)
aux_source_directory(${PROJECT_SOURCE_DIR}/Autoaim/ArmorPredict PRE)

# Serial
include_directories(${PROJECT_SOURCE_DIR}/Serial)
aux_source_directory(${PROJECT_SOURCE_DIR}/Serial SRL)

#cj ArmorPredict
include_directories(${PROJECT_SOURCE_DIR}/StateEstimation)
aux_source_directory(${PROJECT_SOURCE_DIR}/StateEstimation STATE)

# Other
include_directories(${PROJECT_SOURCE_DIR}/Other)

add_executable(SR_SDUST main.cpp
        ${HIK} ${DTC} ${CLS} ${PNP} ${SRL}
        ${PRE} ${TRA} ${KF} ${STATE}
        ${RDTC} ${RFLT} ${RPCT} ${PRE} ${NEW_HIK} ${NEW_HIKHIK} Thread/thread.h Thread/thread.cpp Hikvision/HaiKangCamera.cpp Rune/rune.cpp
        Autoaim/aimbot.cpp
        Autoaim/StateEstimation/state_predict.cpp Autoaim/StateEstimation/state_predict.h Autoaim/StateEstimation/tracker.cpp Autoaim/StateEstimation/tracker.hpp
        Autoaim/StateEstimation/extended_kalman_filter.hpp Autoaim/StateEstimation/extended_kalman_filter.cpp)

target_link_libraries(SR_SDUST ${OpenCV_LIBS} Ceres::ceres glog fmt::fmt yaml-cpp jsoncpp)