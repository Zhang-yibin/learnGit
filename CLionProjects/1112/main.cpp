#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
using namespace std;
using namespace cv;
//还是得轮廓检测吗啊啊啊气死我了


int main(int argc, char *argv[])
{
    //加载图像
    Mat srcImage=imread("/home/yuuki/2.bmp");
    imshow("原始图",srcImage);

    //变换成hsv通道
    Mat hsvImage;
    cvtColor(srcImage,hsvImage,COLOR_BGR2HSV);
    imshow("未增强色调的hsv图片",hsvImage);

    //分割split 与合并merge
    vector<Mat> hsvsplit;//hsv的分离通道
    threshold(hsvImage, hsvImage, 180, 255, 0);
    imshow(",,",hsvImage);
    waitKey();

    split(hsvImage,hsvsplit);

    equalizeHist(hsvsplit[2],hsvsplit[2]);//直方图均衡化，增强对比度，hsvsplit[2]为返回的h
    merge(hsvsplit,hsvImage);//在色调调节后，重新合并
    imshow("增强色调对比度后的hsv图片",hsvImage);
    Mat thresHold;
    //
    threshold(hsvsplit[2],thresHold,240,245,THRESH_BINARY);
    imshow("二值化后图片",thresHold);
    Mat canny;
    Canny(thresHold, canny, 80, 160, 3, false);
    Mat kernel = getStructuringElement(0, Size(1, 1));
    dilate(thresHold, thresHold, kernel);
    GaussianBlur(thresHold, thresHold, Size(7,7), 0);
    morphologyEx(thresHold,thresHold,MORPH_CLOSE,kernel);
    vector<Vec3f> circles;
    double mindist = 2;
    double min_r = 10;
    double max_r = 200;
//    HoughCircles(thresHold, circles, HOUGH_GRADIENT, 1.5, mindist, 100, 100, 0, 2000);
//    cout<<circles.size();
//    for (size_t i = 0; i < circles.size(); ++i) {
//        circle(thresHold, Point(circles[i][0], circles[i][1]), circles[i][2], Scalar(0, 255, 0), 3, 8);
//        circle(thresHold, Point(circles[i][0], circles[i][1]), 10, Scalar(250, 0, 0), -1, 8);
//    }

    Canny(thresHold, thresHold, 100, 200);

    // 圆形拟合

    HoughCircles(thresHold, circles, HOUGH_GRADIENT, 1, 0, 100, 30, 0, 500);

    // 绘制圆形和圆心
    for (size_t i = 0; i < circles.size(); i++) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        circle(thresHold, center, radius, Scalar(0, 0, 255), 2);
        circle(thresHold, center, 3, Scalar(0, 255, 0), -1);
    }
    cout<<circles.size();
    imshow("23",thresHold);
    waitKey();

    return 0;




    while(1)
    {
        int key=cv::waitKey(10);
        if (key==27)
        {
            break;
        }
    }
    return(0);


}
