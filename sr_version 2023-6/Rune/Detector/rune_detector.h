//
// Created by root on 2022/7/4.
//

#ifndef SR_SDUST_RUNE_DETECTOR_H
#define SR_SDUST_RUNE_DETECTOR_H

#include "parameter.h"

class ShootFan{

public:
    ShootFan()= default;
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
    //这是一个C++函数，名为ShootFan。它接受一个名为a的类型为std::vector<Point>的参数和一个RotatedRect类型的参数r。
    // 函数将参数a移动构造赋值给成员变量fancontours，将参数r直接赋值给成员变量rrect。
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
//    Mat contour_filled;
    Mat contour_filled;
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
