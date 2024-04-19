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
    VideoCapture video("/home/yukki/LIT FULL.mp4");
    Mat frame, channels[3], binary, Gaussian;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<Vec3f> circles;
    Rect boundRect;
    RotatedRect box;
    vector<Point2f> boxPts(4);
    for (;;) {
        Rect point_array[20];
        video >> frame;
        if (frame.empty()) {
            break;
        }
        Mat hsv, mask;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        //色彩筛选
        inRange(hsv, Scalar(0, 43, 46), Scalar(10, 255, 255), mask);
        Mat se = getStructuringElement(MORPH_RECT, Size(15, 15));
        morphologyEx(mask, mask, MORPH_OPEN, se);
           imshow("mask",mask);
////           imshow("hsv",hsv);
//        split(frame, channels);
        Mat canny;
        Canny(mask, mask, 80, 160, 3, false);
        Mat kernel = getStructuringElement(0, Size(3, 3));
        dilate(mask, mask, kernel);
        threshold(mask, mask, 220, 255, 0);
        GaussianBlur(mask, mask, Size(5,5), 0);
        imshow("mask",mask);
        findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
        for(int n=0;n<contours.size();n++)
        {

//            if (cv::contourArea(contours[n]) >800)
           // {

                Rect rect= boundingRect(contours[n]);
                rectangle(frame,rect,Scalar(0,0,255),2,8,0);
                RotatedRect rrect= minAreaRect(contours[n]);
                Point2f points[4];
                rrect.points(points);
                Point2f cpt=rrect.center;
                Mat frame2;
                frame.copyTo(frame2);
                //inRange()
                for(int i=0;i<4;i++)
                {
//                cout<<rrect.center;
                    if(i==3)
                    {
                        line(frame,points[i],points[0],Scalar(0,255,0),2,8,0);
                        break;
                    }
                    line(frame,points[i],points[i+1],Scalar(0,0,255),2,8,0);
                }
                circle(frame,cpt,2,Scalar(0,255,0),2,8,0);
                Point2f vertices[4];
                rrect.points(vertices);//从RotatedRect类中提取出角点
                for (int i = 0; i < 4; i++) {
                    line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 2);
                    //cout << vertices[i] << endl;
                float angle;
                angle=fastAtan2(vertices[i].y,vertices[i+1].y);
                cout << angle<<endl;
                }}
            imshow("max",frame);
            //  imshow("min",frame2);
            // imshow("video", frame2);

            if (waitKey(10) >= 0) {
                break;
            }
        }
        return 0;}
//}


        //      int index = -1;
//        double max_area = 0;
//        for (size_t t = 0; t < contours.size(); t++) {
//            double area = contourArea(contours[t]);
//            double len = arcLength(contours[t], true);
////                if(area<100||len<10)continue;
////                Rect box= boundingRect(contours[t]); copy
//            if (area > max_area) {
//                max_area = area;
//                index = t;
//            }
//绘制beta
//            if(index>=0)
//            {RotatedRect rrt= minAreaRect(contours[index]);
//                ellipse(mask,rrt,Scalar(255,0,0),2,8);
//                circle(mask,rrt.center,4,Scalar(0,255,0),2,8,0);
//
//            }
//            imshow("f",frame);
//
//            }
//            double minDist = 20;
//            double min_radius = 10;
//            double max_radius = 50;
//            HoughCircles(Gaussian, circles, HOUGH_GRADIENT, 3, minDist, 100, 100, min_radius, max_radius);
//            for (size_t t = 0; t < circles.size(); t++) {
//                Point center(circles[t][0], circles[t][1]);
//                int radius = round(circles[t][2]);
//                circle(mask, center, 3, Scalar(0, 0, 255), 2, 8, 0);
//                circle(mask, center, 3, Scalar(255, 0, 0), 2, 8, 0);
////                    int points[999999999];
////                points[t]=circles[t].center;
//            }
//                for(int i=1;i<=99999999;i++) {
//                    if (points[i - 1]=0|| points[i]=0)
//                    { continue;
//                    thickness = int(np.sqrt(mybuffer / float(i + 1)) * 2.5);
//                    line(frame, points[i - 1], points[i], (0, 0, 255), thickness);
/*找到圆周运动的圆心——R*/
//        vector<vector<Point>> outlines;
//        vector<Vec4i> hierarchies;
//        int minArea = 10000;
//        int minId;
//        Point2f center;  /*定义外接圆中心坐标*/
//        float radius;  /*定义外接圆半径*/
////        findContours(mask, outlines, hierarchies, RETR_TREE, CHAIN_APPROX_NONE);
//       for (int i = 0; i < outlines.size(); i++) {
//            vector<Point>points;
//            double area = contourArea(outlines[i]);
//            /*面积排除噪声*/
//            if (area < 10 || area>10000)
//                continue;
//            /*找到没有父轮廓的轮廓*/
//            if (hierarchies[i][3] >= 0 && hierarchies[i][3] < outlines.size())
//                continue;
//            /*找有子轮廓的*/
//            if (hierarchies[i][2] < 0 || hierarchies[i][2] >= outlines.size())
//                continue;
//            /*控制误差范围*/
//            if (area <= minArea + 10 && area >= minArea - 20) {
//                minArea = area;
//                minId = i;
//                continue;
//            }
//            /*面积最小的轮廓*/
//            if (minArea >= area)
//            {
//                minArea = area;
//                minId = i;
//            }
//        }
//        /*防止minId不在范围内报错*/
//        if (minId >= 0 && minId < outlines.size()) {
//            /*画外接圆并找到圆心*/
//            minEnclosingCircle(Mat(outlines[minId]), center, radius);
//            circle(mask, center, radius, Scalar(0, 0, 255), 1, 8, 0);
//        }
//        else {
//            //退出
//        }
        //imshow("frame", frame);
//            imshow("ee",mask);
//        if (waitKey(10) >= 0) {
//            break;
//        }
//    }
//    return 0;


//        void process_frame(Mat &image)
//        {
//        Mat hsv,mask;
//        cvtColor(image,hsv,COLOR_BGR2HSV);
//        //色彩筛选
//            inRange(hsv,Scalar(0,43,46),Scalar(10,255,255),mask);
//            Mat se= getStructuringElement(MORPH_RECT,Size(15,15));
//            morphologyEx(mask,mask,MORPH_OPEN,se);
//            imshow("mask",mask);
//            imshow("hsv",hsv);
//
//        }








