// OpenCVtest.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
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
    VideoCapture video("/home/yuuki/Downloads/1104.mp4");
    Mat frame,channels[3],binary,Gaussian;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Rect boundRect;
    RotatedRect box;
    vector<Point2f> boxPts(4);
    for (;;) {
        Rect point_array[20];
        video >> frame;
        if (frame.empty()) {
            break;
        }
        split(frame,channels);
        Mat canny;
        Canny(frame,canny,80,160,3,false);
        Mat kernel = getStructuringElement(0,Size(3,3));
        dilate(canny,canny,kernel);
        vector<vector<Point>>contours;
        vector<Vec4i>hierarchy;
        threshold(canny, binary, kThreashold, kMaxVal, 0);
        GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);
        findContours(Gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
        for(int n=0;n<contours.size();n++)
        {
        Rect rect= boundingRect(contours[n]);
            rectangle(frame,rect,Scalar(0,0,255),2,8,0);
            RotatedRect rrect= minAreaRect(contours[n]);
        Point2f points[4];
        rrect.points(points);
            Point2f cpt=rrect.center;//矩形中心
            frame.copyTo(frame2);
            for(int i=0;i<4;i++)
            {
               cout<<rrect.center;
                if(i==3)
                {
                    line(frame,points[i],points[0],Scalar(0,255,0),2,8,0);
                    break;
                }
                line(frame2,points[i],points[i+1],Scalar(0,0,255),2,8,0);
            }
            circle(frame,cpt,2,Scalar(0,255,0),2,8,0);
        }
        imshow("max",frame);
      //  imshow("min",frame2);
     //   imshow("video", frame2);

       if (waitKey(30) >= 0) {
            break;
       }
    }
    return 0;}