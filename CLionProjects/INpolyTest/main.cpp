#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;

int thre = 51;

Mat src, dst;

void PointPloy(int, void*);

int main()
{

    src = imread("/home/yuuki/1.bmp");
    if (src.empty())//如果src这个数据库属性为空
    {
        cout << "无法打开" << endl;
        return -1;
    }
    cvtColor(src, src, CV_BGR2GRAY);
    imshow("原图", src);
    PointPloy(0,0);
    waitKey(0);
    return 0;
}

//目标：基于轮廓与边缘的参考，计算图像中每个像素点到最近轮廓的距离，生成距离系数图像
//应用场景：对象跟踪时，可以判断一个点或对象是否在指定区域内
void PointPloy(int, void*)
{
    //不自己建立图像 引入图像时使用
    Canny(src, src, thre, thre * 2, 3, false);
    /*
    const int r = 100;
    Mat src = Mat::zeros(r * 4, r * 4, CV_8UC1);//创建一张400*400的单通道黑色图像= imread("添加图片路径")；
    vector<Point2f>vert(6);//定义六边形浮点型双坐标向量数组  6个点坐标
    vert[0] = Point(3 * r / 2, static_cast<int>(1.34 * r));//X坐标150 Y坐标134
    vert[1] = Point(1 * r, 2 * r);//100 200
    vert[2] = Point(3 * r / 2, static_cast<int>(2.866 * r));//150 286
    vert[3] = Point(5 * r / 2, static_cast<int>(2.866 * r));//250 286
    vert[4] = Point(3 * r, 2 * r);//300 200
    vert[5] = Point(5 * r / 2, static_cast<int>(1.34 * r));//250 134
    for (int i = 0; i < 6; i++) {cout << vert[i] << endl;}

    for (int i = 0; i < 6; i++)//使用画直线函数 将6个点坐标连接成一个多边形
    {
        line(src, vert[i], vert[(i + 1) % 6], Scalar(255),3,8,0);
        //为什么要用vert[(i + 1) % 6]? 因为当绘制多边形最后一个点时需要把最后一个点6与起点0连接 6余6为0
    }
    imshow("多边形原图绘制", src);*/

    //首先需要获取几何图像的轮廓：
    vector<vector<Point>>contours;
    vector<Vec4i>hierachy;
    Mat csrc;
    src.copyTo(csrc);	//findContours 会改动输入图像 src 中元素的值，所以这里需要完全复制一份src
    findContours(csrc, contours, hierachy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));//这里的轮廓发现方法最佳应该只检测最外边的轮廓

    dst = Mat::zeros(src.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++)
    {
        drawContours(dst, contours, i, Scalar(0, 255, 0), 2, 8, hierachy, 0, Point(0, 0));
    }
    imshow("图像原轮廓", dst);


    //建立存储点距离轮廓最短距离的数组
    Mat RawDist = Mat::zeros(csrc.size(), CV_32FC1);
    //建立一个和原图大小一致的纯黑背景矩阵来存储点多边形返回的double双浮点型数据(RawDist中每个像素位置处存储的是该点离轮廓的距离)
    //32_FC1 单浮点型是float为16位 双浮点数就是double也就是32FC1(C是channels表示通道数为1即每个点只存储一位数据)

    //然后计算出每个像素距离轮廓的距离：
    for (int row = 0; row < RawDist.rows; row++)
    {
        for (int col = 0; col < RawDist.cols; col++)
        {
            double dist = pointPolygonTest(contours[0], Point2f(static_cast<float>(col), static_cast<float>(row)), true);
            //点多边形测试-测试一个点是否在给定的多边形内部,边缘或外部
            //pointPolygonTest-多边形测试 返回数据为double类型

            //InputArray  contour-输入的轮廓,要求是单通道二值化后的矩阵图像

            //Point2f  pt-需要测试的点

            //bool  measureDist-是否返回距离值,如果是false,1表示在内面,0表示在边界上,-1表示在外部
            //true返回实际距离  若返回值为正,表示点在多边形内部,返回值为负,表示在多边形外部,返回值为0,表示在多边形上

            //NOTE
            //如果您不想找到距离，请确保第三个参数为False，因为这是一个耗时的过程.因此，将其设为False可提供2 - 3倍的加速.
            //contours[0]的意思是只计算第一条轮廓的点集合(最外部轮廓)

            RawDist.at<float>(row, col) = static_cast<float>(dist);
            //RawDist遍历并获取多边形测试计算出返回的数据
            //RawDist中每个像素位置处存储的是该点离轮廓的距离
            //(RawDist与for循环的配合下 可以把多边形测试返回的数据进行结构化处理,把返回的一维数据dist赋值给二维数据RawDist 让每一个像素点都能准确得到数据)
        }
    }
    for (int i = 0; i < contours.size(); i++)
    {
        cout << "输出的第" << i + 1 << "个多边形点集为" << contours[i] << endl;
    }


    // 绘制距离色差图，类似于 距离变换
    double minValue, maxValue;
    minMaxLoc(RawDist, &minValue, &maxValue, 0, 0, Mat());;//使用minMaxLoc在RawDist中找到距离最大最小值
    //数组中寻找全局最大最小值(输入的单通道矩阵数组,返回最小值的指针,返回最大值的指针(不需要 设为NULL或0),返回最小值位置的指针,返回最大值位置的指针,掩膜操作默认不掩膜)
    printf("RawDist点与轮廓距离 参数全局最小值为%f  全局最大值为%f",minValue,maxValue);//最小值为 - 198.979904  全局最大值为78.000000

    // 图形化的显示距离
    Mat drawimg = Mat::zeros(src.size(), CV_8UC3);//用彩色图反差距离
    //建立一个和原图大小一致 3通道的矩阵数组用来绘制反差图

    for (int row = 0; row< drawimg.rows; row++)
    {
        for (int col = 0; col< drawimg.cols; col++)
        {
            float dist = RawDist.at<float>(row, col);//遍历raw_dist每一个位置处存储的值
            if (dist > 0)// 轮廓内部，越靠近轮廓中心点越黑(由于上方多边形测试第三个参数为true 返回的是实际距离 大于0返回值为正的值为多边形内部)
            {
                //dst.at<Vec3b>(row, col)[0] = (uchar)(abs(dist / maxValue) * 255);//确保在0-255之间
                drawimg.at<Vec3b>(row, col)[0] = (uchar)(abs(1.0-(dist / maxValue) * 255));//输出相反的效果，像素值与距离成反比
                //Vec3b-8U类型的RGB 3通道彩色图像 向量数组 存储[0][1][2]三通道的各通道BGR分量 通道未赋值默认为0
                //(uchar)确保了像素点的取值在0-255之间的范围里 防止了数据溢出导致程序的异常
                //abs-取abs括号内的值为绝对值(abs函数仅仅支持整数的绝对值计算，必须使用fabs才能获得浮点数的绝对值)
            }
            else if (dist < 0) // 轮廓外部，越远离轮廓越黑( 小于0返回值为负的值为多边形外部)
            {
                drawimg.at<Vec3b>(row, col)[2] = (uchar)(abs(1.0 -( dist/minValue))*255);//输出相反的效果，像素值与距离成反比
                //若遍历到的点的轮廓距离值为0 将Vec3b中第二个通道的值设置为 1.0减去  当前轮廓距离值除去最小值(-198)
            }
            else // 轮廓边线上,白色(等于0返回值为0的值是为在多边形边上)
            {
                drawimg.at<Vec3b>(row, col)[0] = (uchar)(abs(255 - dist));
                drawimg.at<Vec3b>(row, col)[1] = (uchar)(abs(255 - dist));
                drawimg.at<Vec3b>(row, col)[2] = (uchar)(abs(255 - dist));
                //若遍历到的点的轮廓距离值为0 将Vec3b中三个通道的值设置为255 即白色
            }
        }
    }
    imshow("点多边形测试结果", drawimg);
}


