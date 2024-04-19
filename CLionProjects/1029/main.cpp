
#include <iostream>
#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
Mat frame, srcImage,HsvImage;
Point2f marked(Mat)
{
    VideoCapture capture("/home/yuuki/Downloads/BigWheel.mp4"); //读入视频
    Mat frame, srcImage;
    Point2f center; //定义矩形中心
    while (true) {
        capture >> srcImage;//读入帧
      //  resize(frame, srcImage, Size(frame.cols / 3, frame.rows / 3));//转换大小(原视频太大了)
        Mat hsvImage, dstImage1, dstImage2, HsvImage;
        Mat dstImage;
        cvtColor(srcImage, dstImage, COLOR_BGR2HSV);//转换为HSV图
//        cvtColor(srcImage,dstImage,COLOR_BGRA2GRAY);
        inRange(dstImage, Scalar(156, 43, 46), Scalar(180, 255, 255), dstImage1);//二值化图像，阈值为红色域
        inRange(dstImage, Scalar(0, 43, 46), Scalar(10, 255, 255), dstImage2);//二值化图像，阈值为红色域
        add(dstImage1, dstImage2, dstImage);
       // threshold(dstImage,dstImage,100,255,THRESH_BINARY);
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(dstImage, dstImage, MORPH_OPEN, element);
        vector<vector<Point>> contours;//轮廓数组
        vector<Vec4i> hierarchy; //一个参数
        Point2f center; //用来存放找到的目标的中心坐标
//提取所有轮廓并建立网状轮廓结构
        findContours(dstImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
        for (int j = 0; j < contours.size(); j++)//再次遍历所有轮廓
        {
          if (contours[j]=1) //如果某轮廓对应数组的值为1，说明只要一个内嵌轮廓
          {
            int num = hierarchy[j][2]; //记录该轮廓的内嵌轮廓
            RotatedRect box = minAreaRect(contours[num]); //包含该轮廓所有点
            Point2f vertex[4];
            box.points(vertex);//将左下角，左上角，右上角，右下角存入点集
            for (int i = 0; i < 4; i++) {
                line(srcImage, vertex[i], vertex[(i + 1) % 4], Scalar(255, 0, 0), 4, LINE_AA); //画线
            }
            center = (vertex[0] + vertex[2]) / 2;
            return center;
//              putText(srcImage, "target", vertex[0], FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 0));//打印字体
//          }
        }
    }}
Point2f marked();
int main()
{
    VideoCapture capture("/home/yuuki/Downloads/RedMove.mp4"); //读入视频
    while (true) {
        capture>>srcImage;//读入帧
        //resize(srcImage, srcImage, Size(frame.cols / 3, frame.rows / 3));//转换大小(原视频太大了)
        Point2i center; //定义矩形中心
        //cvtColor(srcImage,srcImage,COLOR_BGRA2GRAY);
        center=marked(srcImage);  //自定义函数进行识别
            imshow("效果图", srcImage);
            cout << center << endl; //打印目标坐标
            if (waitKey(30) >= 0) //按任意键退出
                break;
        }
        Point2f center=marked(srcImage);
        cout<<center;
        return 0;
    }
