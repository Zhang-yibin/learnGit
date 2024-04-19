//视频帧颜色分析与提取    1;查看像素值分布，2;设置range取得mask;3，对mask图像进行轮廓分析
#include "stdio.h"
#include<iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
////    void process_frame(Mat &image);
//const int kThreashold = 220;
//Mat frame2;
//const int kMaxVal = 255;
//int pts[999999999];
//const Size kGaussianBlueSize = Size(5, 5);
//double thickness;
int main() {
    Mat frame, channels[3], binary, Gaussian;
    frame = imread("/home/yuuki/2.bmp");
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<Vec3f> circles;
    Rect boundRect;
    RotatedRect box;
    vector<Point2f> boxPts(4);
    Mat hsv, mask;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    //色彩筛选
    inRange(hsv, Scalar(0, 43, 46), Scalar(10, 255, 255), mask);
    Mat se = getStructuringElement(MORPH_RECT, Size(15, 15));
    morphologyEx(mask, mask, MORPH_OPEN, se);
    imshow("?",mask);
    waitKey();

    vector<Point> largestContour = *max_element(contours.begin(), contours.end(), [](const vector<Point>& c1, const vector<Point>& c2) {return contourArea(c1, false) < contourArea(c2, false);
    });
//        vector<Point2f>center;
    vector<Point2f>centers;
    Point2f center;
    float radius;
    minEnclosingCircle(largestContour, center, radius);
    // Calculate moments of largest contour
    Moments M = moments(largestContour);
    // Calculate centroid
    center = Point(int(M.m10 / M.m00), int(M.m01 / M.m00));
    // If radius significant, draw circles
    if (radius > 10) {
        circle(hsv, center, radius, Scalar(0, 255, 255), 2);
        circle(hsv, center, 5, Scalar(0, 0, 255), -1);
    }
    imshow("hsv", hsv);
    waitKey();
}