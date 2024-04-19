#include<opencv2/opencv.hpp>
using namespace cv;
using namespace std;

// 提前声明三个回调函数
void onChangedConvertToAlpha(int value, void* data);
void onChangedConvertToBeta(int value, void* data);
void onChangedConvertToGama(int value, void* data);
//部分代码
static void onTrack(int lightness, void* data) {
    Mat src = *(Mat*)data;	//将void类型指针转换为Mat类型指针，然后再取数据
    Mat m = Mat::zeros(src.size(), src.type());
    Mat dst = Mat::zeros(src.size(), src.type());
    m = Scalar(lightness, lightness, lightness);

    add(src, m, dst);
    imshow("Change Lightness", dst);
}
int main(int argc, char** argv)
{
    // 读取图像，判断是否读入成功
    string fileName = samples::findFile("/home/yukki/1.webp");
    Mat src = imread(fileName, IMREAD_COLOR);
    int alpha = 10;
    int beta = 0;
    int gama = 10;
    namedWindow("demo", WINDOW_AUTOSIZE);
    createTrackbar("alpha", "demo", nullptr, 30, onChangedConvertToAlpha, (void*)&src);
    createTrackbar("beta", "demo", &beta, 100, onChangedConvertToBeta, (void*)&src);
    createTrackbar("gama", "demo", &gama, 20, onChangedConvertToGama, (void*)&src);
    // 显示地调用一次回调函数，显示默认值
    onChangedConvertToAlpha(alpha, (void*)&src);
    system("pause");
    return 0;
}
// 当改变alpha值时的响应函数
void onChangedConvertToAlpha(int value, void* data)
{
    Mat src = (*(Mat*)data).clone();
    Mat dst = Mat::zeros(src.size(), src.type());
    int beta = getTrackbarPos("beta", "demo");
    src.convertTo(dst, -1, (double)value / 10, beta);
    setTrackbarPos("gama", "demo", 10);
    imshow("demo", dst);
    waitKey(0);
}
// 当改变beta值时的响应函数
void onChangedConvertToBeta(int value, void* data)
{
    Mat src = (*(Mat*)data).clone();
    Mat dst = Mat::zeros(src.size(), src.type());
    int alpha = getTrackbarPos("alpha", "demo");
    src.convertTo(dst, -1, (double)alpha / 10.0, value);
    setTrackbarPos("gama", "demo", 10);
    imshow("demo", dst);
    waitKey(0);
}
// 当改变gama值时的响应函数
void onChangedConvertToGama(int value, void* data)
{
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    double gama = value / 10.0;
    for (int i = 0; i < 256; ++i)
        p[i] = saturate_cast<uchar>(pow(i / 255.0, gama) * 255.0);
    Mat src = (*(Mat*)data).clone();
    Mat dst = Mat::zeros(src.size(), src.type());
    LUT(src, lookUpTable, dst);
    imshow("demo", dst);
    waitKey(0);
}
