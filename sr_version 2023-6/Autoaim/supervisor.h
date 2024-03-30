//
// Created by eski on 6/30/22.
//

#ifndef SR_SDUST_SUPERVISOR_H
#define SR_SDUST_SUPERVISOR_H

#include <iostream>
#include <array>
#include <vector>
#include <utility>
#include <algorithm>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <string.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


#include <opencv2/core/eigen.hpp>


using namespace std;
using namespace cv;

const int RED = 0;
const int BLUE = 1;


static bool is_balance_rob = false;
static int cnt_balance = 0;
static int cnt_search = 0;

const int ENEMY_COLOR = BLUE;

// TODO: commonly used
//#define CV_INIT
//#define OPEN_SERIAL


#define SHOW_SRC
//#define SHOW_CENTERS
//#define SHOW_RUNE_CENTERS

///////////////time////////////////////////
//#define SHOW_PRODUCER_LATENCY
//#define SHOW_CONSUMER_LATENCY



//#define PRINT_ORIGINAL_SEND_DATA
#define PRINT_ORIGINAL_RECEIVE_DATA

// TODO: main.cpp
// ---- #define SHOW_SRC

// ---- #define SHOW_TIME
// ---- #define OPEN_SERIAL

// TODO: aimbot.h aimbot.cpp
// TODO: armor part
//#define SHOW_COORDINATE
//#define SHOW_YAW_SCATTER_DIAGRAM
#define USE_PREDICTOR
#define SHOW_TARGET_DEBUG
#define PRINT_TARGET_INFO


// TODO: rune part
//#define SHOW_RUNE_CENTERS
#define SHOW_CERES_DEBUG

// TODO: send part
//#define PRINT_SEND_MESSAGE


// TODO: detector.h detector.cpp
#define LOOSE_PROCESS
//#define STRICT_PROCESS
//#define TRACKER_ON

#define SHOW_BINARY
#define SHOW_PREPROCESS_IMG
#define SHOW_LIGHTS
#define SHOW_ARMORS


// TODO: rune_detector.h rune_detector.cpp
//#define SHOW_RUNE_BINARY
//#define SHOW_RUNE_FLOOD
//why no flood????????????????????????????????????????
//
//#define SHOW_RUNE_ARMORS_BINARY
//#define SHOW_RUNE_ARMOR

//#define SHOW_RUNE_ARMOR_LINES
//#define SHOW_R_ROI
//#define SHOW_RUNE_CENTER


// TODO: state_predict.h state_predict.cpp
#define SHOW_PREDICT_DEBUG
//#define SHOW_TARGET_STATE
//#define SHOW_HORIZON
//#define SHOW_TRACKER_DEBUG


#define changeTo2f(x) to_string(int(x)) + "." + to_string(int(x * 100 + 0.5) % 100)

#endif //SR_SDUST_SUPERVISOR_H