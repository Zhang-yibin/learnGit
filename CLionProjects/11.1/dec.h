//
// Created by yukki on 24-1-
#define ARMOR_DETECTOR__DETECTOR_HPP_// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
// STD
#include <cmath>
#include <string>
#include <vector>
#ifndef INC_11_1_DEC_H
#define INC_11_1_DEC_H

#endif //INC_11_1_DEC_H
namespace rm_auto_aim
{

class ShootFan
{
public:
    ShootFan() = default;
    std::vector<cv::Point> fan_contours;
    cv::RotatedRect rrect;
    std::vector<cv::Point2f> longside_centers;
    cv::Point2f towards;  //右上角为原点，向右为x轴，向下为y轴
    cv::Point2f long_side;
    cv::Point2f target_center;
    int fan_cols;
    std::vector<cv::Point2f> armor_points;
    cv::Point2f fan_center;
    double fan_angle;
    ShootFan(std::vector<cv::Point> a, cv::RotatedRect r) : fan_contours(std::move(a)), rrect(r) {}
    //这个C++函数是一个构造函数，用于初始化一个名为ShootFan的对象。它接受一个类型为std::vectorcv::Point的参数a和一个类型为cv::RotatedRect的参数r。
    // 函数将参数a移动构造为fan_contours成员变量，并将参数r移动构造为rrect成员变量。
    cv::Point2f flow_far_from_center;
private:
};

class RuneDetector
{
public:
    struct RuneParam
    {
        double binary_threshold = 80;

        // Contour fill
        double min_filledContours_area = 900;
        double max_contourArea = 27000;
        double min_contourArea = 5000;

        //Select the fan
        double max_fan_area = 29000;
        double min_fan_area = 3000;
        double min_area_ratio = 1.5;
        double min_fan_ratio = 1.4;
        double max_fan_ratio = 2.5;

        //Find the flow
        double min_flow_area_ratio = 0.7;
        double max_flow_area = 9000;
        double min_flow_area = 300;
        double max_ratio = 5;
        double min_ratio = 2;

        // 60 300 120 240
        // size(5,5) 40 200
        double min_r_area = 20, max_r_area = 2000;
        // 0.4 3.2
        double min_r_ratio = 0.4, max_r_ratio = 5;
    };

public:

    cv::Point2f r_center;
    std::vector<cv::RotatedRect> armors;
    ShootFan final_fan;
    float fan_angle = 0;
    enum Color {
        self_BLUE,
        self_RED,
    } rune_color;
    bool findCenter(cv::Mat & src);
    bool imageProcess(cv::Mat & src);
    std::vector<ShootFan> fillContour();
    bool fanSizer(std::vector<ShootFan> fans);
    static double PixelContour(cv::Mat & rot, double start, int height);

    RuneDetector(const RuneParam & r);
private:

public:
    RuneParam rune_param;
    cv::Mat binary_img;
    cv::Mat filled_contour_img;
    std::vector<std::vector<cv::Point>> filled_contours;
    std::array<cv::Point2f, 4> sorted_pts;

    // Debug image
    cv::Mat rune_debug;
    cv::Mat rune_armor_lines;
    cv::Mat rune_detector_debug;
};

struct Rune
{
    cv::Point2f r_center;
    cv::Point2f pre_center;
    cv::RotatedRect rune_armor;

    double real_angle;
    float radian = 0;

    int cols;
    int rows;

};


}  // namespace rm_auto_aim
