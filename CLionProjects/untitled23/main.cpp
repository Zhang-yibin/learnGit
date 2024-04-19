#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
using namespace std;
using namespace cv;
Mat frame;
Mat src, dst, dst2, gray_dst;
int threshold_value = 127;
int threshold_max = 255;
const char* output = "binary image";
void Threshold_Demo(int, void*);
int main() {
    VideoCapture video("/home/yuuki/Downloads/1104.mp4");
//    Mat frame;
    for (;;) {
        video >> frame;
        if (frame.empty()) {
            break;
        }
//    double scale = 0.5;
//    Size dsize = Size(src.cols * scale, src.rows * scale);
       // resize(frame, frame, Size(300,300));
        //为图像创建边框
        //copyMakeBorder(dst, dst2, 20, 20, 20, 20, cv::BORDER_CONSTANT, Scalar(116, 73, 16));
        //阈值化操作
//        namedWindow("input", WINDOW_AUTOSIZE);
//        namedWindow(output, WINDOW_AUTOSIZE);
//        imshow("input", frame);
//        createTrackbar("阈值滑条:", output, &threshold_value, threshold_max, Threshold_Demo);
//        Threshold_Demo(0, 0);
        if (waitKey(30) >= 0) {
            break;
        }
    }
    return 0;
}
//
//
//
//
//
//
//
//
//
//
//
// void Threshold_Demo(int, void (*)) {
//    cvtColor(frame, frame, COLOR_BGR2GRAY);
////    int threshold_value = 127;
////double alpha;
////    alpha=(double)threshold_value/threshold_max;
//    threshold(frame, frame, threshold_value, threshold_max, THRESH_BINARY);
//    imshow(output, frame);
//    waitKey();
//}
