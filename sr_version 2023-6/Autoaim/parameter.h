//
// Created by eski on 6/29/22.
//

#ifndef SR_SDUST_PARAMETER_H
#define SR_SDUST_PARAMETER_H

#include "supervisor.h"

#define predict_count 8

const static cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
const static cv::Mat kernel5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
const static cv::Mat kernel7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
const static cv::Mat kernel11 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));

enum ArmorType { SMALL = 0, LARGE = 1 ,INVALID=2};

enum AimMode{
    NORMAL_AIM,ANTI_TOP,SMALL_BUFF,BIG_BUFF
};

struct LightParams
{
    // height / width
    int binary_threshold = 70;
    double min_ratio = 1.2;
    double max_ratio = 25;
    // vertical angle
    double max_angle = 40;
    float light_deviation =45;
};

struct ArmorParams
{
    double min_light_ratio = 0.8;

    double min_small_center_distance = 0.8;
    double max_small_center_distance = 3.2;

    double min_large_center_distance = 3.2;
    double max_large_center_distance = 5.5;

    // horizontal angle
    double max_angle = 35;
};

struct RuneParam{
    int binary_threshold=120;

    //contour_fill
    int min_filledcontours_area=300;
    int max_contourArea=43000;
    int min_contourArea=2000;

    //select the fan
    float max_fan_area=55000;
    float min_fan_area=1000;
    double min_area_ratio=1.75;
    double min_fan_ratio=1.6;
    double max_fan_ratio=2.5;

    //find the flow
    double min_flow_area_ratio=0.7;
    float max_flow_area=30000;
    float min_flow_area=100;
    float max_ratio =5;
    float min_ratio=2.5;
    // 60 300 120 240
    // size(5,5) 40 200
    int min_r_area = 20, max_r_area = 1000;
    // 0.4 3.2
    float min_r_ratio = 0.4, max_r_ratio = 5;

};





























//static array<double, 9> cameraMatrix = {864.107238674125, 0.000000, 337.077127954314,
//                                        0.000000, 864.359261234400, 228.717457604452,
//                                        0.000000, 0.000000, 1.000000 };
static array<double, 9> cameraMatrix = {1788.10802903, 0.000000, 687.7655,
                                        0.000000, 1787.771293205565, 584.4036,
                                        0.000000, 0.000000, 1.000000 };
// k1 k2 p1 p2 k3
static vector<double> distCoeffs = {-0.0614, 0.0971, 0,0, 0};

// StateTracker state
static double max_Matchdistance = 0.55;
static int trackingThreshold = 5;
static int lostThreshold = 55;
static double maxMatchYawdiff = 0.4;

static double last_time = 0;

// Q
static double Q00 = 0.01, Q11 = 10, Q22 = 0.01, Q33 = 10, Q44 = 0.01;
// R
static double R00 = 1, R11 = 1, R22 = 800;

#endif //SR_SDUST_PARAMETER_H
