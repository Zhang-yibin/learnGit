//
#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;
vector<vector<Point>> contours;
Scalar backgroundColor(28,38,45);
Scalar lowerBound(175, 129, 22);
//Scalar upperBound(182, 255, 255);
//vector<Vec4i> hierarchy;
//const int kMaxVal = 255;
//int budin=3;
//const Size kGaussianBlueSize = Size(5, 5);
//Mat newBackground;
//Mat changeBackground(const Mat& inputImage, const Scalar& backgroundColor, const Scalar& lowerBound, const Scalar& upperBound)
//{
//    Mat hsv;
//    cvtColor(inputImage, hsv, COLOR_BGR2HSV);
////    imshow("?",hsv);
//    // 在指定范围内的变为白色，不在范围内的变为黑色
//    Mat mask;
//    inRange(hsv, lowerBound, upperBound, mask);
//    bitwise_not(mask, mask);
////    imshow("mask",mask);
//
//    Mat ele= getStructuringElement(0,Size(7,7));
//    morphologyEx(mask,mask,MORPH_OPEN,ele);
//    imshow("m",mask);
//    newBackground = Mat::zeros(inputImage.size(), inputImage.type());
//    newBackground = backgroundColor;
//    inputImage.copyTo(newBackground, mask);
////    imshow("lat",newBackground);
//    return newBackground;
//}
//int main() {
//    Mat src=imread("/home/yuuki/3.bmp");
//    Mat frame, binary, Gaussian;
//    changeBackground(src,backgroundColor,lowerBound,upperBound);
//    Rect boundRect;
//    RotatedRect box;
//    vector<Point2f> boxPts(4);
//    frame=newBackground.clone();
//    imshow(">/",frame);
//    Mat canny;
//    Canny(frame, canny, 80, 160, 3, false);
//    Mat kernel = getStructuringElement(0, Size(5, 7));
////    dilate(canny, canny, kernel);// 膨胀作用==加粗
////    imshow("?????123?",canny);
//   morphologyEx(canny,canny,MORPH_CLOSE,kernel); imshow("??????",canny);
//    threshold(canny, binary, 180, kMaxVal, 0);
////     imshow("222", binary);
////    blur(canny,canny,Size(5,5));
////    imshow("?????123?",canny);
//    GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);
//    imshow("2222", Gaussian);
////    morphologyEx(Gaussian,Gaussian,3,kernel);
////    imshow("22222", Gaussian);
//
////    Gaussian=newBackground.clone();
////    imshow("...",newBackground);
//    findContours(Gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
//    for (int n = 0; n < contours.size(); n++) {
////        if (cv::contourArea(contours[n]) < 12000) {//||cv::contourArea(contours[n]) > 17999){
////            continue;
////        }
////        if (cv::contourArea(contours[n]) > 30600) {//||cv::contourArea(contours[n]) > 17999){
////            continue;
////        }
////        Rect rect;
////        rect = boundingRect(contours[n]);
////        rectangle(frame, rect, Scalar(0, 0, 255), 2, 8, 0);
//        RotatedRect rrect;
//        rrect = minAreaRect(contours[n]);
////        if (std::max(rect.width, rect.height) / std::min(rect.width, rect.height) > 1.19   )    // 长宽比约束 1.42
////        {
////            continue;
////        }
////        if (std::max(rrect.size.width, rrect.size.height) / std::min(rrect.size.width, rrect.size.height) >
////            1.19)       // 长宽比约束 1.42
////        {
////            break;
////        }
//        Point2f points[4];
//        rrect.points(points);
//        Point2f cpt = rrect.center;
//        for (int i = 0; i < 4; i++) {
//            cout<<(points[i].x,points[i])<<endl;
//            if (i == 3) {
//                line(frame, points[i], points[0], Scalar(255, 255, 255), 2, 8, 0);
//                break;
//            }
//            line(frame, points[i], points[i + 1], Scalar(255, 255, 255), 2, 8, 0);
//        }
//        circle(frame, cpt, 2, Scalar(0, 255, 0), 2, 8, 0);
//        Point2f vertices[4];
//        Point2f red_center=Point(frame.rows,frame.cols-budin);
//        cout<<red_center;
//        rrect.points(vertices);//从RotatedRect类中提取出角点
//        for (int i = 0; i < 4; i++) {
//            line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 2);
//            cout << vertices[i] << endl;
////            double Pitch=fastAtan2(vertices[i].x,red_center.x);
////            double Yaw= fastAtan2(vertices[i].y,red_center.y);
////            cout<<Pitch<<endl;
////            cout<<Yaw<<endl;
//      /**      float radian = atan2((vertices[i].y - red_center.y), (vertices[i].x - red_center.x));//弧度   该函数返回值范围是[-pi,pi]
//            float angle = radian * 180 / 3.1415926;//角度
//            cout << "radian:" << radian << '\n';
//            cout << "angle" << angle << '\n';  */
//        }
//        imshow("max", frame);
//        waitKey();
//
//    }
//
//}
void swip(Point2f a,Point2f b)
{auto temp=a;
a=b;
b=temp;}
void swappp(vector<Point>a)
{
    a={Point(10,10),Point(20,10),Point(20,20),Point(10,20)};
    for(int i=0;i<4;i++) {
        for (int j = i + 1; j < 5; j++) {
            if (a[i].x > a[j].x) {
                swip(a[i], a[j]);
            }
        }
    }
    if(a[0].y>a[1].y)
    { swip(a[0],a[1]);}
    if(a[2].y>a[3].y)
    { swip(a[2],a[3]);}


            Point2f dian[5];
            for(int i=0;i<5;i++) {
                dian[i] = a[i];
            }
    for(int i=0;i<5;i++)
    {cout<<dian[i]<<endl;}

}
int main()
{
    cout<<(unsigned char) 11;
}
