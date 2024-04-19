/**
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;
int main()
{
    //对图像进行距离变换
    cv::Mat img = cv::imread("/home/yuuki/1.bmp");

    cv::Mat rice, riceBW;

    //将图像转成二值图像，用于统计连通域
    cv::cvtColor(img, rice, cv::COLOR_BGR2GRAY);
    cv::threshold(rice, riceBW, 50, 255, cv::THRESH_BINARY);

    //生成随机颜色，用于区分不同连通域
    cv::RNG rng(10086);
    cv::Mat out;
    int number = connectedComponents(riceBW, out, 8, CV_16U);  //统计图像中连通域的个数
    vector<cv::Vec3b> colors;
    for (int i = 0; i < number; i++)
    {
        //使用均匀分布的随机数确定颜色
        cv::Vec3b vec3 = cv::Vec3b(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        colors.push_back(vec3);
    }

    //以不同颜色标记出不同的连通域
    cv::Mat result = cv::Mat::zeros(rice.size(), img.type());
    int w = result.cols;
    int h = result.rows;
    for (int row = 0; row < h; row++)
    {
        for (int col = 0; col < w; col++)
        {
            int label = out.at<uint16_t>(row, col);
            if (label == 0)  //背景的黑色不改变
            {
                continue;
            }
            result.at<cv::Vec3b>(row, col) = colors[label];
        }
    }

    //显示结果
    cv::imshow("原图", img);
    cv::imshow("标记后的图像", result);

    cv::waitKey(0);

}

// Read image
Mat im = imread( "blob.jpg", IMREAD_GRAYSCALE );

// Set up the detector with default parameters.
SimpleBlobDetector detector;

// Detect blobs.
std::vector<KeyPoint> keypoints;
detector.detect( im, keypoints);

// Draw detected blobs as red circles.
// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
Mat im_with_keypoints;
drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

// Show blobs
imshow("keypoints", im_with_keypoints );
waitKey(0);

#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv4/opencv2/features2d.hpp"
#include <opencv4/opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
 using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    Mat img = imread("/home/yuuki/1.bmp");

    SimpleBlobDetector::Params params;
    params.minThreshold = 40;
    params.maxThreshold = 160;
    params.thresholdStep = 5;
    params.minArea = 100;
    params.minConvexity = .05f;
    params.minInertiaRatio = .05f;
    params.maxArea = 8000;

    SimpleBlobDetector detector(params);

    vector<KeyPoint> key_points;

    detector.detect(img, key_points);

    Mat output_img;

    drawKeypoints(img, key_points, output_img, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    namedWindow("SimpleBlobDetector");
    imshow("SimpleBlobDetector", output_img);
    waitKey(0);

    return 0;
}
 **/
//#include <opencv2/highgui.hpp>
//
//#include <opencv2/calib3d.hpp>
//
//#include <iostream>
//
//using namespace std;
//using namespace cv;
//
//int main(){
//    Mat img = imread("/home/yuuki/3.bmp",IMREAD_GRAYSCALE);
//    imshow("bef",img);
//
//    SimpleBlobDetector::Params params;
//    //阈值控制
//    params.minThreshold = 10;
//    params.maxThreshold = 200;
//    //像素面积大小控制
//    params.filterByArea = true;
//    params.minArea = 1000;
//    //形状（凸）
//    params.filterByCircularity = false;
//    params.minCircularity = 0.7;
//    //形状（凹）
//    params.filterByConvexity = true;
//    params.minConvexity = 0.9;
//    //形状（园）
//    params.filterByInertia = false;
//    params.minInertiaRatio = 0.5;
//
//
//    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();
//    vector<KeyPoint> keypoints;
//    detector->detect(img,keypoints);
//    Mat img_with_keypoints;
//    drawKeypoints(img,keypoints,img_with_keypoints,Scalar(0,0,255),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//    imshow("keypoints",img_with_keypoints);
//    waitKey(0);
//    return 0;
//}

#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv4/opencv2/features2d.hpp"
#include <opencv4/opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#include "iostream"
using namespace cv;
//using namespace std;

//int main(int argc, char** argv)
//{
//    Mat img = imread("/home/yuuki/3.bmp");
//    imshow("bf",img);
//    SimpleBlobDetector::Params params;
//    params.minThreshold = 100;
//    params.maxThreshold = 660;
//    params.thresholdStep = 5;
//    params.minArea = 100;
//    params.minConvexity = .05f;
//    params.minInertiaRatio = .05f;
//    params.maxArea = 8000;

//    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();
//
//    std::vector<KeyPoint> keypoints;
////
////    Mat img_with_keypoints;
//
//    detector->detect(img,keypoints);
//
//    Mat output_img;
//
//    drawKeypoints( img, keypoints, output_img, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
//
//    namedWindow("SimpleBlobDetector");
//    imshow("SimpleBlobDetector", output_img);
//    waitKey(0);
//
//    return 0;
//}
    using namespace std;
    using namespace cv;
    int main()
    {
        Mat src=imread("/home/yuuki/3.bmp");

        cvtColor(src,src,COLOR_BGR2HSV);
        imshow("s",src);
        //*参数设置，以下都是默认参数
        SimpleBlobDetector::Params pDefaultBLOB;
        pDefaultBLOB.thresholdStep = 6;
        pDefaultBLOB.minThreshold = 120;
        pDefaultBLOB.maxThreshold = 220;
        pDefaultBLOB.minRepeatability = 2;
        pDefaultBLOB.minDistBetweenBlobs = 10;
        pDefaultBLOB.filterByColor = true;
        pDefaultBLOB.blobColor = 0;
        pDefaultBLOB.filterByArea = true;
        pDefaultBLOB.minArea = 25;
        pDefaultBLOB.maxArea = 5000;
        pDefaultBLOB.filterByCircularity = false;
        pDefaultBLOB.minCircularity = 0.8f;
        pDefaultBLOB.maxCircularity = (float)3.40282e+038;
        pDefaultBLOB.filterByInertia = true;
        pDefaultBLOB.minInertiaRatio = 0.1f;
        pDefaultBLOB.maxInertiaRatio = (float)3.40282e+038;
        pDefaultBLOB.filterByConvexity = true;
        pDefaultBLOB.minConvexity = 0.95f;
        pDefaultBLOB.maxConvexity = (float)3.40282e+038;
        //*用参数创建对象
        Ptr<SimpleBlobDetector> blob=SimpleBlobDetector::create(pDefaultBLOB);
        //Ptr<SimpleBlobDetector> blob=SimpleBlobDetector::create();//默认参数创建
        //*blob检测
        vector<KeyPoint> key_points;
        blob->detect(src,key_points);
        Mat outImg;
        //*绘制结果
        cout<< sizeof(key_points);
        drawKeypoints(src,key_points,outImg,Scalar(255,255,0));
        imshow("blob",outImg);

        waitKey();
        return 0;
    }