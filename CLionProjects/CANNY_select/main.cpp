#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include<vector>

using namespace cv;
using namespace std;

Mat dst1;
Mat frame;
Mat dst2;
Mat hsv;

int LH = 166;
int LS = 43;
int LV = 46;
int HH = 180;
int HS = 240;
int HV = 240;
int trackbar = 500;//低阈值最小值
int trackbar2 = 500;//高阈值最小值
void onLH(int, void*)
{
    inRange(hsv, Scalar(LH, LS, LV), Scalar(HH, HS, HV), dst2);					//二值化处理
    imshow("2", dst2);
}
void onLS(int, void*)
{
    inRange(hsv, Scalar(LH, LS, LV), Scalar(HH, HS, HV), dst2);					//二值化处理
    imshow("2", dst2);
}
void onLV(int, void*)
{
    inRange(hsv, Scalar(LH, LS, LV), Scalar(HH, HS, HV), dst2);					//二值化处理
    imshow("2", dst2);
}
void onHH(int, void*)
{
    inRange(hsv, Scalar(LH, LS, LV), Scalar(HH, HS, HV), dst2);					//二值化处理
    imshow("2", dst2);
}
void onHS(int, void*)
{
    inRange(hsv, Scalar(LH, LS, LV), Scalar(HH, HS, HV), dst2);					//二值化处理
    imshow("2", dst2);
}
void onHV(int, void*)
{
    inRange(hsv, Scalar(LH, LS, LV), Scalar(HH, HS, HV), dst2);					//二值化处理
    imshow("2", dst2);
}
void on_trackbar(int , void*) {
    Canny(dst2, dst2, trackbar, trackbar2);
    imshow("2", dst2);
}
int main()
{
    frame=imread("/home/yuuki/1.bmp");
    //创建窗口
    namedWindow("1",0);
    resizeWindow("1", 500, 500);
    namedWindow("2", 0);
    resizeWindow("2", 1230, 1200);
    imshow("1",frame);
//cvCvtColor(...),是Opencv里的颜色空间转换函数，可以实现RGB颜色向HSV,HSI等颜色空间的转换
//CV_BGR2HSV将图片从RGB空间转换为HSV空间。
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    //imshow("?",hsv);
    createTrackbar("LH", "2", &LH, 180, onLH);
    createTrackbar("HH", "2", &HH, 180, onHH);
    createTrackbar("LS", "2", &LS, 255, onLS);
    createTrackbar("HS", "2", &HS, 255, onHS);
    createTrackbar("LV", "2", &LV, 255, onLV);
    createTrackbar("HV", "2", &HV, 255, onHV);
    createTrackbar("低阈值", "2", &trackbar, 500, on_trackbar);
    createTrackbar("高阈值", "2", &trackbar2, 500, on_trackbar);
    onLH(0, 0);
    onLS(0, 0);
    onLV(0, 0);
    onHH(0, 0);
    onHS(0, 0);
    onHV(0, 0);
    on_trackbar(trackbar, 0);//轨迹回调函数
    on_trackbar(trackbar2, 0);
    waitKey(0);
}

