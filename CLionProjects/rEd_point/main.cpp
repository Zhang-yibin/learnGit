#include <iostream>
#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
Mat frame= imread("/home/yuuki/1.webp");
vector<vector<Point>> contours;
vector<Vec4i> hierarchy;
Rect boundRect;
RotatedRect box;
Point2f center;
Point2f arr[10];
Point2f get_red(Mat);
double getDistance (Point2f point1, Point2f point2)
{
    double distance = sqrtf(powf((point1.x - point2.x),2) + powf((point1.y - point2.y),2));
    return distance;
}
int main() {
    resize(frame,frame,Size(),0.5,0.5);
    imshow("1",frame);
    get_red(frame);
    cout<<center.x,center.y;
    printf("%2f %2f",center.x,center.y);
    imshow("?",frame);
    waitKey();
    return 0;
}
Point2f get_red(Mat n)
{
cvtColor(frame,frame,COLOR_BGR2HSV);
//inRange(frame, Scalar(150, 43, 46), Scalar(180, 255, 255), frame);
inRange(frame, Scalar(0,0,230), Scalar(195,195, 255), frame);
//    imshow("?",frame);
//    waitKey();
//Canny(frame,frame, 100, 200);
Mat ele= getStructuringElement(0,Size(3,3));
// morphologyEx(frame,frame,MORPH_OPEN,ele);
findContours(frame,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);
 float radius;
for (int n = 0; n < contours.size(); n++) {
    if (cv::contourArea(contours[n]) < 10) {//||cv::contourArea(contours[n]) > 17999){
        continue;
    }
    vector<Point> largestContour = *max_element(contours.begin(), contours.end(),
                                                [](const vector<Point> &c1, const vector<Point> &c2) {
                                                    return contourArea(c1, false) < contourArea(c2, false);
                                                });
//minEnclosingCircle(largestContour , center, radius);
    minEnclosingCircle(contours[n], center, radius);
    circle(frame, center, radius, Scalar(255), 2);
    Moments M = moments(largestContour);
// Calculate centroid
    center = Point(int(M.m10 / M.m00), int(M.m01 / M.m00));
}
    return center;}