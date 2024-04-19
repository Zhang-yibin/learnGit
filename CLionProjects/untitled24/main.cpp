#include<opencv2/opencv.hpp>
#include<iostream>
#include<math.h>
using namespace std;
using namespace cv;

Mat src, dst, dst2, gray_dst;
int threshold_value = 127;
int threshold_max = 255;
const char* output = "binary image";
void Threshold_Demo(int, void*);
int main()
{
    src = imread("/home/yuuki/Downloads/1.jpeg");
    if (!src.data) {
        printf("could not load the image...\n");
        return  -1;
    }
    double scale = 0.5;
    Size dsize = Size(src.cols * scale, src.rows * scale);
    resize(src, dst, dsize);

    //为图像创建边框
    //copyMakeBorder(dst, dst2, 20, 20, 20, 20, cv::BORDER_CONSTANT, Scalar(116, 73, 16));
    //阈值化操作
    namedWindow("input", WINDOW_AUTOSIZE);
    namedWindow(output, WINDOW_AUTOSIZE);
    imshow("input", dst);
    createTrackbar("阈值滑条:", output, &threshold_value, threshold_max, Threshold_Demo);
    //imwrite("D:/PT/shan/shan12345.jpg",dst2);
    Threshold_Demo(0, 0);
    waitKey(0);
    return 0;
}
void Threshold_Demo(int, void*) {
    cvtColor(dst, gray_dst, COLOR_BGR2GRAY);
    threshold(gray_dst, dst2, threshold_value, threshold_max, THRESH_BINARY);
    imshow(output, dst2);

}

