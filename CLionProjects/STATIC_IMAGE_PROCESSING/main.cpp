#include <iostream>
#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
//const int kThreashold = 140;
Mat binary,Gaussian;
//const int kMaxVal = 255;
const Size kGaussianBlueSize = Size(5, 5);
void drawMyContours(string winName, Mat &image, std::vector<std::vector<cv::Point>> contours, bool draw_on_blank);
int main() {
    // 1.载入图像
    cv::Mat frame = cv::imread("/home/yuuki/1.bmp");
    resize(frame, frame, Size(640,480));
    cv::Mat frmHsv;
    cv::cvtColor(frame, frmHsv, cv::COLOR_BGR2HSV);
    Mat canny;
    Canny(frame, canny, 80, 160, 3, false);
    Mat kernel = getStructuringElement(0, Size(7, 7));
    dilate(canny, canny, kernel);
    morphologyEx(canny, canny, 0, kernel);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
//    threshold(canny, binary, kThreashold, kMaxVal, 0);
    inRange(frmHsv, Scalar(90, 0, 48), Scalar(137, 255, 255), binary);
    imshow("e",binary);
    GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);
    findContours(Gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    for (int n = 0; n < contours.size(); n++) {
        double area =cv::contourArea(contours[n]);
        if (area< 13000)
        {//||cv::contourArea(contours[n]) > 17999){
            continue;
        }

//        if (cv::contourArea(contours[n]) >28600){//||cv::contourArea(contours[n]) > 17999){
//            continue;
//        }
        // 4.绘制原始轮廓
        //drawMyContours("contours", Gaussian, contours, true);
        // 5.筛选轮廓
        // 初始化迭代器
        std::vector < std::vector < cv::Point >> ::iterator
        itc = contours.begin();
        std::vector<cv::Vec4i>::iterator itc_hierarchy = hierarchy.begin();
        // 5.1使用层级结构筛选轮廓
        int i = 0;
        while (itc_hierarchy != hierarchy.end()) {
            //验证轮廓大小
            if (hierarchy[i][2] > 0)//|| hierarchy[i][3] > 0) // 存在子轮廓/父轮廓
            {
                itc = contours.erase(itc);
                itc_hierarchy = hierarchy.erase(itc_hierarchy);
            } else {
                ++i;
                ++itc;
                ++itc_hierarchy;
            }
        }
        // 绘制级别筛选后的轮廓
        drawMyContours("contours after hierarchy filtering", Gaussian, contours, true);
        // 5.2使用轮廓长度滤波
        int min_size = 660;
        int max_size = 500;

        // 针对所有轮廓
        itc = contours.begin();
        itc_hierarchy = hierarchy.begin();
        while (itc != contours.end())
        {
            //验证轮廓大小
            if (itc->size() < min_size)// || itc->size() > max_size)
            {
                itc = contours.erase(itc);
                itc_hierarchy = hierarchy.erase(itc_hierarchy);
            }
            else
            {
                ++itc;
                ++itc_hierarchy;
            }
        }
        printf("%d contours remaining after length filtering", contours.size());
        // 绘制长度筛选后的轮廓
        drawMyContours("contours after length filtering", Gaussian, contours, true);
    }cout<<contours.size()<<endl;
    return 0;}
// 4. 绘制轮廓函数
void drawMyContours(string winName, Mat &image, std::vector<std::vector<cv::Point>> contours, bool draw_on_blank)
{
    cv::Mat temp;
    if (draw_on_blank) // 在白底上绘制轮廓
    {
        temp = cv::Mat(image.size(), CV_8U, cv::Scalar(255));
        cv::drawContours(
                temp,
                contours,
                -1,//画全部轮廓
                0, //用黑色画
                2);//宽度为2
    }
    else // 在原图上绘制轮廓
    {
        temp = image.clone();
        cv::drawContours(
                temp,
                contours,
                -1,//画全部轮廓
                cv::Scalar(0,0,255), //用red画
                1);//宽度为2
    }
    cv::imshow(winName, temp);
    cv::waitKey();
}