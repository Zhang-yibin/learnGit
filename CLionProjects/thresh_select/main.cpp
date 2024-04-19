#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
////    void process_frame(Mat &image);
//const int kThreashold = 220;
//Mat frame2;
//const int kMaxVal = 255;
//int pts[999999999];
//const Size kGaussianBlueSize = Size(5, 5);
//double thickness;

void onChangeTrackBar (int pos,void* data) {

    cv::Mat srcImage = *(cv::Mat *) (data); //强制类型转换
    cv::Mat dstImage;
    // 根据滑动条的值对图像进行二值化处理
    cv::threshold(srcImage, dstImage, pos, 255, cv::THRESH_BINARY);
    cv::imshow("dyn_threshold", dstImage);
}
void colorFilter(cv::Mat inputImage, cv::Mat& outputImage)
{
    // 将CvMat转换为cv::Mat
    cv::Mat img = inputImage;

    // 转换为HSV色彩空间
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    int width = hsv.cols;
    int height = hsv.rows;
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            cv::Vec3b hsvPixel = hsv.at<cv::Vec3b>(i, j);
            // 获取像素点（j, i）的HSV值
            uchar h = hsvPixel[0];
            uchar s = hsvPixel[1];
            uchar v = hsvPixel[2];

            // 红色的H范围过滤条件
            if (!((h > 0 && h < 8) || (h > 160 && h < 180)) || (s < 80) || (v < 50 || v > 220))
            {
                hsv.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0); // 设置为黑色
            }
        }
    }

    // 创建输出图像矩阵
    outputImage.create(hsv.size(), hsv.type());

    // 将处理后的HSV图像转换回BGR以便显示和存储
    cv::cvtColor(hsv, outputImage, cv::COLOR_HSV2BGR);

    // 显示结果
    cv::namedWindow("filter", cv::WINDOW_NORMAL);
    cv::imshow("filter", outputImage);
    cv::waitKey(0);
}
int main() {

    Mat frame, channels[3], binary, Gaussian;

    frame = imread("/home/yukki/QWQW.png");
    resize(frame,frame,Size(640,480));
    Mat frame1=frame.clone();
    colorFilter(frame1,frame1);
    imshow("redred",frame1);
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    imshow("mask", frame);
    Rect boundRect;
    RotatedRect box;
    vector<Point2f> boxPts(4);
    Mat hsv, mask;
    //cvtColor(frame, hsv, COLOR_BGR2HSV);
    //色彩筛选
    hsv=frame.clone();
    inRange(hsv, Scalar(150, 43, 46), Scalar(255, 255, 255), mask);
    imshow("mask1", mask);
    Mat se = getStructuringElement(MORPH_RECT, Size(15, 15));
    morphologyEx(mask, mask, MORPH_OPEN, se);
    imshow("hsv", hsv);
    split(frame, channels);
    Mat canny;
    Canny(frame, canny, 80, 160, 3, false);
    Mat kernel = getStructuringElement(0, Size(1, 1));
    dilate(hsv, hsv, kernel);
    threshold(hsv, binary, 180, 255, 0);
    cv::namedWindow("dyn_threshold");
    cv::imshow ("dyn_threshold",hsv);
    //创建滑动条createTrackbar，调用回调函数
    createTrackbar ("pos","dyn_threshold",
                    0, 255, onChangeTrackBar ,&hsv);

    cv::waitKey(0);

    imshow("?", binary);


}
    /**
    GaussianBlur(binary, Gaussian, Size(5,5), 0);
    morphologyEx(Gaussian,Gaussian,MORPH_CLOSE,kernel);




    vector<Vec3f> circles;
    double mindist = 2;
    double min_r = 10;
    double max_r = 200;
    //  HoughCircles(Gaussian, circles, HOUGH_GRADIENT, 1.5, mindist, 100, 100, 10, 200);

    for (size_t i = 0; i < circles.size(); ++i) {
        circle(Gaussian, Point(circles[i][0], circles[i][1]), circles[i][2], Scalar(0, 255, 0), 3, 8);
        circle(Gaussian, Point(circles[i][0], circles[i][1]), 10, Scalar(250, 0, 0), -1, 8);
    }
    imshow("23",Gaussian);
    waitKey();
    return 0;

}*/