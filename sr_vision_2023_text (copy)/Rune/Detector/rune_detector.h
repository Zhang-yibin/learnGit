//
// Created by root on 2022/7/4.
//
#define before_orgin
#ifdef before_orgin

#ifndef SR_SDUST_RUNE_DETECTOR_H
#define SR_SDUST_RUNE_DETECTOR_H

#include "parameter.h"

class ShootFan{

public:
    cv::Point2f flow_far_from_center;
    ShootFan()= default;;
    std::vector<Point>  fancontours;
    RotatedRect rrect;
    std::vector<Point2f> longside_centers;
    Point2f towards;  //右上
    Point2f long_side;
    Point2f target_center;
    int fan_cols;
    std::vector<Point2f> armor_points;
    Point2f fan_center;
    double fan_angle;
    ShootFan(std::vector<Point>  a,RotatedRect r):fancontours(std::move(a)),rrect(r){}
private:

};

class RuneDetector
{
public:

    cv::Point2f r_center;
    int quadrant;
    float final_angle;
    RotatedRect final_armor;
    vector<RotatedRect> armors;
    ShootFan final_fan;
    float rot_angle = 0;
    bool findCenter(Mat src);

    enum Color {
        self_RED,
        self_BLUE,
    } detect_color;

//    void stateInspection(cv::Point2f &center,Point2f &armor_center);
private:
//    bool findArmor();
//    bool imageProcess(Mat src);
//    bool judgeFlowingLight(Point2f &short_side,double &flowing_sum);
    bool imageProcess(Mat src);
    std::vector<ShootFan> fillContour();
    bool fanSizer(std::vector<ShootFan> fans);
    static void colorSpilt(Mat& input,Mat& output,int div);
    static double PixelCounter(Mat rot, double start, int height);
private:
    RuneParam rune_param;
//    std::array<cv::Point2f, 4> sorted_pts;
    Mat rune_debug;
//    Mat binary;
    Mat contour_filled;
    Mat filled_contour_img;
    std::vector<std::vector<Point> > filledcontours;
    std::array<cv::Point2f, 4> sorted_pts;
private:


    Mat binary;
    Mat rune_armor_lines;
    Mat rune_detector_debug;

private:
    void clearArray(){
        armors.clear();

    }
};


#endif //SR_SDUST_RUNE_DETECTOR_H
// }
#endif





#ifdef new_orgin1

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
// STD
#include <cmath>
#include <string>
#include <vector>
#define para1
#define show_debuug

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
        //轮廓点集+矩形中心/角点
        cv::Point2f flow_far_from_center;
    private:
    };

    class RuneDetector
    {
    public:
#ifdef para1
        struct RuneParam
        {
            double binary_threshold = 80;

            // Contour fill
            double min_filledContours_area = 900;
            double max_contourArea = 39000;
            double min_contourArea = 2000;

            //Select the fan
            double max_fan_area = 29000;
            double min_fan_area = 3000;
            double min_area_ratio = 1.2;
            double min_fan_ratio = 1.2;
            double max_fan_ratio = 2.3;

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
#endif
#ifdef para2
        struct RuneParam
        {
            double binary_threshold = 80;//why not 120?

            // Contour fill
            double min_filledContours_area = 300;
            double max_contourArea = 23000;
            double min_contourArea = 7000;

            //Select the fan
            double max_fan_area = 25000;
            double min_fan_area = 3000;
            double min_area_ratio = 1.5;
            double min_fan_ratio = 1.4;
            double max_fan_ratio = 2.5;

            //Find the flow
            double min_flow_area_ratio = 0.7;
            double max_flow_area = 3000;
            double min_flow_area = 500;
            double max_ratio = 5;
            double min_ratio = 2;

            // 60 300 120 240
            // size(5,5) 40 200
            double min_r_area = 200, max_r_area = 1000;
            // 0.4 3.2
            double min_r_ratio = 0.4, max_r_ratio = 5;
        };
#endif
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

#endif