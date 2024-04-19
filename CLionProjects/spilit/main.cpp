#include<iostream>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
Mat src, srcGray, srcBinary, redBinary;
int main(int argc, char** argv)
{
    src = imread("/home/yuuki/Downloads/1115");
    cvtColor(src, srcGray, COLOR_BGR2GRAY);
    threshold(srcGray, srcBinary, 180, 255, THRESH_BINARY | THRESH_OTSU);
    //进行图像的通道分割
    vector<Mat>channels;
    split(src, channels);
    Mat blue = channels[0];
    Mat green = channels[1];
    Mat red = channels[2];
    //对red通道的图像进行二值化
    threshold(red, redBinary, 180, 255, THRESH_BINARY|THRESH_OTSU);
    imshow("src", src);
    imshow("srcGray", srcGray);
    imshow("srcBinary", srcBinary);
    imshow("red", red);
    imshow("g",green);
    imshow("b",blue);
    imshow("redBinary", redBinary);
    waitKey(0);
    return 0;
}