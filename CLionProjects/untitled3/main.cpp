#include <iostream>
#include "opencv4/opencv2/opencv.hpp"
#include "opencv4/opencv2/opencv_modules.hpp"
using namespace std;
using namespace cv;
int main() {
    cv::Mat img;
    cv::imread("/home/yuuki/Downloads/1");
    imshow("hellow",img);
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
