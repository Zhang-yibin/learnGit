#include <opencv2/opencv.hpp>

using namespace cv;
#define window_name "线性混合"

//按理来说白线应该是没了才对，我不理解。
const int max_value = 100;
int value;


Mat image1;
Mat image2;

int main()
{
    image1 = imread("/home/yuuki/Downloads/11.jpg");
    image2 = imread("/home/yuuki/Downloads/23.jpg");

    value = 50;
    namedWindow(window_name);
    char tranckbarname[50] = "透明值100";
    createTrackbar(tranckbarname, window_name, nullptr, max_value);
    setTrackbarPos(tranckbarname, window_name, value);
    Mat dst;
    double alpha1;
    double alpha2;
    alpha1 = (double)value / max_value;
    alpha2 = (1.0 - alpha1);
    resize(image1,image1,Size(1024,1024),0,0,INTER_LINEAR);
    resize(image2,image2,Size(1024,1024),0,0,INTER_LINEAR);

    addWeighted(image1, alpha1, image2, alpha2, 0.0, dst);
    Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 15));
    morphologyEx(dst, dst, 2, kernel);//形态学操作
    imshow(window_name, dst);

    waitKey();
    return 0;
}