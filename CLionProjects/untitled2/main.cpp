#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
using namespace cv;
using namespace std;
int main() {
    Mat img;
    img=imread("1.jpeg");
    imshow("hello",img);
    waitKey();
    cout<<"fuck";
    return 0;


}