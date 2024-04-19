#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
vector<vector<Point>> contours;
Scalar backgroundColor(28,38,45);
Scalar lowerBound(175, 129, 22);
Scalar upperBound(182, 255, 255);
vector<Vec4i> hierarchy;
const int kMaxVal = 255;
Point2f dian[2][4];
Point2f zhondian[4];
int yuzhi=180;
Mat newBackground;
int budin=3;
const Size kGaussianBlueSize = Size(5, 5);
void paidian()
{
    for(int q=0;q<4;q++) {
        zhondian[q] = ((dian[0][q] + dian[1][q]) / 2);//xierudian}
    }

}
Mat changeBackground(const Mat& inputImage, const Scalar& backgroundColor, const Scalar& lowerBound, const Scalar& upperBound)
{
    Mat hsv;
    cvtColor(inputImage, hsv, COLOR_BGR2HSV);
//    imshow("?",hsv);
    // 在指定范围内的变为白色，不在范围内的变为黑色
    Mat mask;
    inRange(hsv, lowerBound, upperBound, mask);
    bitwise_not(mask, mask);
//    imshow("mask",mask);
    Mat ele= getStructuringElement(0,Size(7,7));
    morphologyEx(mask,mask,MORPH_OPEN,ele);
    imshow("m",mask);
    newBackground = Mat::zeros(inputImage.size(), inputImage.type());
    newBackground = backgroundColor;
    inputImage.copyTo(newBackground, mask);
    return newBackground;
}
int main() {
//    Mat src = imread("/home/yuuki/Downloads/1116.jpg");
    Mat src;
    VideoCapture video("/home/yuuki/1117.mp4");
    for (;;) {
        Rect point_array[20];
        video >> src;
        if (src.empty()) {
            break;
        }
    resize(src,src,Size(640,480));
    Mat frame, binary, Gaussian;
    Rect boundRect;
    RotatedRect box;
    vector<Point2f> boxPts(4);
    changeBackground(src, backgroundColor, lowerBound, upperBound);
    frame = newBackground.clone();
    frame = src.clone();
    Mat canny;
    Canny(frame, canny, 80, 160, 3, false);
    Mat kernel = getStructuringElement(0, Size(5, 7));
    //dilate(canny, canny, kernel);
    morphologyEx(canny, canny, MORPH_CLOSE, kernel);
    imshow("canny", canny);
    threshold(canny, binary, 180, kMaxVal, 0);
    // imshow("2", binary);
    GaussianBlur(binary, Gaussian, kGaussianBlueSize, 0);
    imshow("233", Gaussian);
    int k = 0;
    findContours(Gaussian, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    for (int n = 0; n < contours.size(); n++) {
        if (cv::contourArea(contours[n]) < 13500) {//||cv::contourArea(contours[n]) > 17999){
            continue;
        }
        if (cv::contourArea(contours[n]) > 32800) {//||cv::contourArea(contours[n]) > 17999){
            continue;
        }
//            if (cv::contourArea(contours[n]) > 39800) {//||cv::contourArea(contours[n]) > 17999){
//                continue;
//            }/////(dajuxing!!!!!!!!!!!!!!!!!!!!
        if (arcLength(contours[n], true) > 1110) { continue; }
//
        RotatedRect rrect;
        rrect = minAreaRect(contours[n]);
        if (std::max(rrect.size.width, rrect.size.height) / std::min(rrect.size.width, rrect.size.height) >
            1.18)       // 长宽比约束 1.42
        {
            continue;
        }
        Point2f points[4];
        rrect.points(points);
        Point2f cpt = rrect.center;

        for (int i = 0; i < 4; i++) {
            if (i == 3) {
                line(frame, points[i], points[0], Scalar(0, 255, 0), 2, 8, 0);
                break;
            }
            line(frame, points[i], points[i + 1], Scalar(0, 0, 255), 2, 8, 0);
        }
        circle(frame, cpt, 2, Scalar(0, 255, 0), 2, 8, 0);
        Point2f vertices[4];
        Point2f red_center = Point(frame.rows, frame.cols - budin);
        //cout<<red_center;
        rrect.points(vertices);//从RotatedRect类中提取出角点
        for (int i = 0; i < 4; i++) {
            line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 2);
            cout << vertices[i] << endl << endl;
            dian[k][i] = vertices[i];
//                double distancex=vertices[i].x-red_center.x
//                double Pitch=fastAtan2(vertices[i].x,red_center.x);
//                double Yaw= fastAtan2(vertices[i].y,red_center.y);
//                cout<<Pitch<<endl;
//                cout<<Yaw<<endl;
            imshow("max", frame);

        }
        k++;
    }
    paidian();
    for (int i = 0; i < 4; i++) {
        if (i == 3) {
            line(frame, zhondian[i], zhondian[0], Scalar(255, 255, 255), 2, 8, 0);
            break;
        }
        line(frame, zhondian[i], zhondian[i + 1], Scalar(255, 125, 255), 2, 8, 0);
    }
        if (waitKey(55) >= 0) {
            break;
        }
    }
}

        // Rect rect;
//            rect = boundingRect(contours[n]);
//            rectangle(frame, rect, Scalar(0, 0, 255), 2, 8, 0);

//            if (std::max(rect.width, rect.height) / std::min(rect.width, rect.height) > 1.17)       // 长宽比约束 1.42
//            {
//                break;
//            }





void judgecenter(Point2f n,Point2f cpt)
{
    if(n!=cpt)
    {
        double Pitch=fastAtan2(n.x,cpt.x);
        double Yaw=fastAtan2(n.y,cpt.y);

    }

}
