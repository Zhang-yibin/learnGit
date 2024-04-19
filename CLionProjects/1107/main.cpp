#include "stdio.h"
#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
const int kThreashold = 220;
Mat frame2;
const int kMaxVal = 255;
const Size kGaussianBlueSize = Size(5, 5);
int main()
{
//    VideoCapture video("/home/yuuki/Downloads/1104.mp4");
    Mat frame,channels[3],binary,Gaussian;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Rect boundRect;
    RotatedRect box;
    vector<Point2f> boxPts(4);
//    for (;;) {
        Rect point_array[20];
        frame= imread("/home/yukki/Downloads/1116.jpg");
        resize(frame,frame,Size(),0.3,0.3);

        split(frame,channels);
        Mat canny;
        resize(frame,frame,Size(600,300));
//        Canny(frame,canny,80,160,3,false);
        Mat kernel = getStructuringElement(0,Size(3,3));
//        dilate(canny,canny,kernel);
//        vector<vector<Point>>contours;
//        vector<Vec4i>hierarchy;
//    imshow("1",channels[1]);
//    imshow("2",channels[0]);

        threshold(channels[1], binary, 164, kMaxVal, 0);
////
////
////        GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);
////        findContours(Gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
////        for(int t=0;t<contours.size();t++)
////        {
////            drawContours(Gaussian, contours, -1, Scalar(100, 120, 255), 2, 8);
////
////        }
        imshow("Frame", binary);
//        if (waitKey(30) ==27) {
//            break;
//        }
    waitKey();
    return 0;
}