#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;
const int kThreashold = 140;
double distance1;
vector<Vec4i> hierarchy;
vector<vector<Point>> contours;
vector<vector<Point>> contours1;
vector<Vec4i> hierarchy1;
Mat frame= imread("/home/yuuki/3.bmp");
double getDistance (Point2f point1, Point2f point2);
Point2f get_red(Mat n,Point2f cpt);
const int kMaxVal = 255;
const Size kGaussianBlueSize = Size(5, 5);
int main() {
        Mat channels[3], binary, Gaussian;
        vector<Point2f> boxPts(4);
        resize(frame, frame, Size(640, 480));
        Mat canny;
        Canny(frame, canny, 80, 160, 3, false);
        Mat kernel = getStructuringElement(0, Size(7, 7));
        dilate(canny, canny, kernel);
        morphologyEx(canny,canny,0,kernel);
        threshold(canny, binary, kThreashold, kMaxVal, 0);
        imshow("2", binary);
        GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);
        imshow("?!",Gaussian);
        findContours(Gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
        for (int n = 0; n < contours.size(); n++) {
            if (cv::contourArea(contours[n]) < 4000){
                continue;
            }
            if (cv::contourArea(contours[n]) >18600){
                continue;
            }
            //double dist1 = pointPolygonTest(contours[n],Point(30,30),true);
            //cout<<dist1<<endl;
            Rect rect;
            rect= boundingRect(contours[n]);
            rectangle(frame, rect, Scalar(0, 0, 255), 2, 8, 0);
            RotatedRect rrect;
            rrect = minAreaRect(contours[n]);
            Point2f points[4];
            rrect.points(points);
            Point2f cpt = rrect.center;
            Mat frame1=frame.clone();
            cvtColor(frame1, frame1, COLOR_BGR2HSV);
             inRange(frame1, Scalar(0, 0, 230), Scalar(195, 195, 255), frame1);
            findContours(frame1, contours1, hierarchy1, RETR_TREE, CHAIN_APPROX_NONE);
            imshow("????????/",frame1);
            //cvtColor(frame1,frame1,COLOR_GRAY2BGR);
            double min=0;
            cout<<contours1.size()<<endl;
            for (int i = 0; i < contours1.size(); i++) {
                for (int j = 0; j < contours1[i].size(); j++) {    //绘制出contours向量内所有的像素点
                    Point P = Point(contours1[i][j].x, contours1[i][j].y);
                    getDistance(P, cpt);
                     cout<<distance1;
           printf("%2f",distance1);
                    imshow("????????/11",frame1);
                    cv::circle(frame, P, 8, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
                    double dist = pointPolygonTest(contours[n], P, true);
//                    for (int i = 0; i < contours.size(); i++) {
//                        double dist = pointPolygonTest(contours[i], P, true);
//                        if(dist<0)
//                        { continue;}
                  //      cout<<dist<<endl;
//                        for (int j = 0; j < contours[i].size(); j++) {    //绘制出contours向量内所有的像素点
//                        }

                    drawContours(frame1, contours1, i, Scalar(255), 1, 8, hierarchy1);
                }}
                for (int i = 0; i < 4; i++) {
                   // cout << rrect.center;
                    if (i == 3) {
                        line(frame, points[i], points[0], Scalar(0, 255, 0), 2, 8, 0);
                        break;
                    }
                    line(frame, points[i], points[i + 1], Scalar(0, 0, 255), 2, 8, 0);
                }
                circle(frame, cpt, 2, Scalar(0, 255, 0), 2, 8, 0);
                Point2f vertices[4];
                rrect.points(vertices);//从RotatedRect类中提取出角点
                for (int i = 0; i < 4; i++) {
                    line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 2);
                    //cout << vertices[i] << endl;
                }
//            get_red(frame,cpt);

                imshow("max", frame);

            }
        waitKey();
        return 0;
    }
    double getDistance (Point2f point1, Point2f point2)
    {
        double distance1 = sqrtf(powf((point1.x - point2.x),2) + powf((point1.y - point2.y),2));
        return distance1;
    }
  /**void jn()
    {vector<vector<Point>> contours1;
        vector<Vec4i> hierarchy1;
        Mat frame1=n.clone();
        cvtColor(frame1, frame1, COLOR_BGR2HSV);
//inRange(frame1, Scalar(150, 43, 46), Scalar(180, 255, 255), frame1);
        inRange(frame1, Scalar(0, 0, 230), Scalar(195, 195, 255), frame1);
        findContours(frame1, contours1, hierarchy1, RETR_TREE, CHAIN_APPROX_NONE);
        imshow("????????/",frame1);
        double min=0;
        //cout<<contours.size()<<endl;
        for (int i = 0; i < contours1.size(); i++) {


            //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
            for (int j = 0; j < contours1[i].size(); j++) {    //绘制出contours向量内所有的像素点
                Point P = Point(contours1[i][j].x, contours1[i][j].y);
                getDistance(P,cpt);
//            cout<<distance1;
//            printf("%2f",distance1);
                cv::circle(frame, P, 8, cv::Scalar(0, 255, 0), 2, cv::LINE_8);
//            if(distance1=0)
//            {break;}
                double dist = pointPolygonTest(contours[i],P,true);
                //   printf("%2f",dist);
//            if(dist<min)

//            {min=dist;}
                for (int i = 0; i < contours.size(); i++) {

                    double dist = pointPolygonTest(contours[i], P, true);
//                if(dist<0)
//                { continue;}
//                    if(dist>min)
//                    {min=dist;
//                    dist=0;}
//                    else
//                    {dist =1;}
//                    if(dist=1)
//                    { continue;}

                    //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
                    for (int j = 0; j < contours[i].size(); j++) {    //绘制出contours向量内所有的像素点

                    }

                }

                drawContours(frame1, contours1, i, Scalar(255), 1, 8, hierarchy1);
            }}**/


//  *void draw(contours) {
//      int idx = 0;
//      for (; idx >= 0; idx = hierarchy[idx][0]) {
//          Scalar color(rand() & 255, rand() & 255, rand() & 255);
//          drawContours(frame, contours, idx, color, FILLED, 8, hierarchy);
//      }
//  }