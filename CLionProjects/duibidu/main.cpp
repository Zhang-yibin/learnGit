#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
int bias{ 0 };
int alpha{ 100 };
int biasMax{ 255 };
int alphaMax{ 100 };
Mat frame=imread("/home/yuuki/1.bmp");
int colorStart{ 0 };
int contast{ 100 };
int colorMax{ 255 };
int contastMax{ 100 };
void mytrackBarCallback31(int pos, void* userdata);
void mytrackBarCallback32(int pos, void* userdata);
void mytrackBarCallback2(int pos, void* userdata);
const Size kGaussianBlueSize = Size(5, 5);
//声明并定义全局变量
Mat srcBar = imread("/home/yuuki/1.bmp", 1);

//回调函数2，使用全局变量，不使用传入参数
void mytrackBarCallback2(int, void*) {
    Mat dst;
    srcBar.convertTo(dst, -1, contast * 0.01, colorStart);
    imshow("image show", dst);

}

//trackBar测试
int main() {
    namedWindow("image show", WINDOW_FREERATIO);
    //调整亮度的TrackBar
    createTrackbar("亮度", "image show", &colorStart, colorMax, mytrackBarCallback2);
    //调整对比度的TrackBar，他们使用同一个回调函数
    createTrackbar("对比度", "image show", &contast, contastMax, mytrackBarCallback2);
    waitKey(0);
}




