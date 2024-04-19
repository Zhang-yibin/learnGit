#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
int main() {
    Mat src= Mat (800, 200, CV_8UC3,cv::Scalar(255,255,255));
//    for (int i = 0; i < 200; i++)
//    {
//        Vec3b* data =
//                src.ptr<Vec3b>(i);
//        for (int j = 0; j < 200; j++)
//        {
//            if (data[j][0] > 200 && data[j][2] < 50)
//                data[j] = Vec3b(255, 255, 255);
//        }
//    }
//BGR BLUE GREEN RED

    for (int i = 0; i < 200; i++)
    {
        for (int j = 0; j < 200; j++)
        {
            src.at<cv::Vec3b>(i, j)[0] = 255;
            src.at<cv::Vec3b>(i, j)[1] = 0;
            src.at<cv::Vec3b>(i, j)[2] = 0;
        }
    }
    for (int i = 201; (i>200) && (i < 400); i++)
    {
        for (int j = 0; j < 200; j++)
        {
            src.at<cv::Vec3b>(i, j)[0] = 0;
            src.at<cv::Vec3b>(i, j)[1] = 255;
            src.at<cv::Vec3b>(i, j)[2] = 0;
        }
    }
    for (int i = 401; (i >400) && (i<600); i++)
    {
        for (int j = 0; j < 200; j++)
        {
            src.at<cv::Vec3b>(i, j)[0] = 0;
            src.at<cv::Vec3b>(i, j)[1] = 0;
            src.at<cv::Vec3b>(i, j)[2] = 255;
        }
    }


    imshow("white",src);

    waitKey(0);
    return 0;
}
//cv::Mat Img = cv::Mat(smallSize, CV_8UC3, cv::Scalar(0, 0, 255));// 创建三通道黑色图像。
//cv::Mat GreyImg = cv::Mat(smallSize, CV_8UC3, cv::Scalar(255,0,0));// 创建单通道黑色图像。
//Mat Green;
//Green=Mat(smallSize,CV_8UC3,Scalar(0,255,0));
// cv::Size smallSize;
//    smallSize.height = 200;
//    smallSize.width = 100;
//cv::imshow("red.jpg", Img);
//cv::imshow("BLUE.jpg", GreyImg);
//imshow("gre",Green);