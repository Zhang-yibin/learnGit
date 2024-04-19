#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

const Size kGaussianBlueSize = Size(5, 5);
int main() {
    Mat frame=imread("/home/yuuki/1.bmp");
    resize(frame, frame, Size(640, 480));
    cvtColor(frame,frame,COLOR_BGR2HSV);
    imshow("??",frame);
    Mat channels[3], binary, Gaussian;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Rect boundRect;
    RotatedRect box;

//        split(frame,channels);
        //inRange(frame,frame)
//        cvtColor(frame,frame,COLOR_BGR2HSV);
//        cv::Scalar lower_red = cv::Scalar(169,0,5);
//        cv::Scalar upper_red = cv::Scalar(180,255,255);
//        cv::Mat mask;
//        cv::inRange(frame, lower_red, upper_red, mask);
//        cv::Mat red_image;
//        //cv::bitwise_and(frame, mask, red_image);
//        imshow("1",mask);
//        Mat canny;
       // Canny(frame, canny, 80, 160, 3, false);
//        Mat kernel = getStructuringElement(0, Size(7, 7));
//        dilate(canny, canny, kernel);
//        morphologyEx(canny, canny, 0, kernel);
//    imshow("???",canny);
    inRange(frame,Scalar(156,43,46),Scalar(180,255,255),binary);
   // Canny(binary,binary,80,160,3,false);
//        threshold(canny, binary, 180, 255, 0);
    GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);
    findContours(binary,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create();
    vector<KeyPoint> keypoints;
    detector->detect(Gaussian,keypoints);
    Mat img_with_keypoints;
    drawKeypoints(Gaussian,keypoints,img_with_keypoints,Scalar(0,0,255),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("keypoints",img_with_keypoints);
        imshow("2", binary);
        waitKey();
        return 0;
    }