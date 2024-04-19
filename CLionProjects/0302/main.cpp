////#include "opencv2/highgui/highgui.hpp"
////#include "opencv2/imgproc/imgproc.hpp"
////#include <iostream>
////#include <stdio.h>
//////this file show the different between warpAffine and rotate
////using namespace cv;
////using namespace std;
////
/////// 全局变量
////char* source_window = "Source image";
////char* warp_window = "Warp";
////char* warp_rotate_window = "Warp + Rotate";
////
/////** @function main */
////int main( int argc, char** argv )
////{
////    Mat canvas = Mat::zeros(Size(512, 512), CV_8UC3);
////    int w = canvas.cols;
////    int h = canvas.rows;
////    Point p1(100, 100);
////    Point p2(300, 150);
////    Point p3(300, 350);
////    Point p4(250, 450);
////    Point p5(50, 450);
////    std::vector<Point> pts;
////    pts.push_back(p1);
////    pts.push_back(p2);
////    pts.push_back(p3);
////    pts.push_back(p3);
////    pts.push_back(p4);
////    pts.push_back(p5);
////    std::vector<std::vector<Point>> contours;
////    contours.push_back(pts);
//////    drawContours(canvas, contours, 0, Scalar(0, 0, 255), -1, 8);
//////    drawContours(canvas, contours, 0, Scalar(255, 0, 255), 2, 8);
////    fillPoly(canvas, contours, Scalar(0, 0, 255), FILLED);
////imshow("q",canvas);
////    waitKey(0);
////}
//////	Point2f srcTri[3];
//////	Point2f dstTri[3];
//////
//////	Mat rot_mat( 2, 3, CV_32FC1 );
//////	Mat warp_mat( 2, 3, CV_32FC1 );
//////	Mat src, warp_dst, warp_rotate_dst;
//////
//////	/// 加载源图像
//////	src = imread("/home/yukki/1.webp");
//////
//////	/// 设置目标图像的大小和类型与源图像一致
//////	warp_dst = Mat::zeros( src.rows, src.cols, src.type() );
//////
//////	/// 设置源图像和目标图像上的三组点以计算仿射变换
//////	srcTri[0] = Point2f( 0,0 );
//////	srcTri[1] = Point2f( src.cols - 1, 0 );
//////	srcTri[2] = Point2f( 0, src.rows - 1 );
//////
//////	dstTri[0] = Point2f( src.cols*0.0, src.rows*0.33 );
//////	dstTri[1] = Point2f( src.cols*0.85, src.rows*0.25 );
//////	dstTri[2] = Point2f( src.cols*0.15, src.rows*0.7 );
//////
//////	/// 求得仿射变换
//////	warp_mat = getAffineTransform( srcTri, dstTri );
//////
//////	/// 对源图像应用上面求得的仿射变换
//////	warpAffine( src, warp_dst, warp_mat, warp_dst.size() );
//////
//////	/** 对图像扭曲后再旋转 */
//////
//////	/// 计算绕图像中点顺时针旋转50度缩放因子为0.6的旋转矩阵
//////	Point center = Point( warp_dst.cols/2, warp_dst.rows/2 );
//////	double angle = -50.0;
//////	double scale = 0.6;
//////
//////	/// 通过上面的旋转细节信息求得旋转矩阵
//////	rot_mat = getRotationMatrix2D( center, angle, scale );
//////
//////	/// 旋转已扭曲图像
//////	warpAffine( warp_dst, warp_rotate_dst, rot_mat, warp_dst.size() );
//////
//////	/// 显示结果
//////	/**1
//////	namedWindow( source_window, WINDOW_AUTOSIZE );
//////	imshow( source_window, src );
//////
//////	namedWindow( warp_window, WINDOW_AUTOSIZE );
//////	imshow( warp_window, warp_dst );//1
//////
//////	namedWindow( warp_rotate_window, WINDOW_AUTOSIZE );
//////	imshow( warp_rotate_window, warp_rotate_dst );//2
//////**/
//////    Mat img=src.clone();
//////    // 定义原图像中的三个点和变换后的三个点
//////    std::vector<cv::Point2f> src_points = {cv::Point2f(50, 50), cv::Point2f(200, 50), cv::Point2f(50, 200)};
//////    std::vector<cv::Point2f> dst_points = {cv::Point2f(10, 100), cv::Point2f(200, 50), cv::Point2f(100, 250)};
//////
//////    // 计算仿射变换矩阵
//////    cv::Mat M = cv::getAffineTransform(src_points, dst_points);
//////
//////    // 仿射变换图像，这里使用宽度和高度作为尺寸参数
//////    cv::Mat affine_img;
//////    cv::warpAffine(img, affine_img,M,img.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
//////
//////    // 显示变换后的图像
//////    cv::namedWindow("Affine Image", cv::WINDOW_NORMAL);
//////    cv::imshow("Affine Image", affine_img);
//////
//////
//////    cv::Point2f center1(img.cols / 2.0F, img.rows / 2.0F);
//////    double angle1 = 45.0; // 旋转角度（以度为单位）
//////    double scale1 = 1.0; // 缩放比例
//////
//////    // 计算旋转矩阵
//////    cv::Mat M1 = cv::getRotationMatrix2D(center, angle, scale);
//////
//////    // 计算旋转后图像的新尺寸（考虑边界溢出）
//////    cv::Rect bbox = cv::RotatedRect(cv::Point2f(center), img.size(), angle * CV_PI / 180).boundingRect();
//////    cv::Size newSize(bbox.width + 1, bbox.height + 1); // 添加一个像素防止裁剪
//////
//////    // 更新旋转矩阵以适应新尺寸
//////    M.at<double>(0, 2) += newSize.width / 2.0 - center.x;
//////    M.at<double>(1, 2) += newSize.height / 2.0 - center.y;
//////
//////    // 创建一个新的目标图像并旋转
//////    cv::Mat rotated_img(newSize, img.type());
//////    cv::warpAffine(img, rotated_img,M1,img.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
//////
//////    // 显示旋转后的图像
//////    cv::namedWindow("Rotated Image", cv::WINDOW_NORMAL);
//////    cv::imshow("Rotated Image", rotated_img);
//////    cv::waitKey(0);
//////    cv::destroyAllWindows();
//////
//////
//////
//////
//////
//////
//////    /// 等待用户按任意按键退出程序
//////	waitKey(0);
//////
//////	return 0;
//////}
//////
//////
//////
//////
//////
//////
//////
//////
//////////area为切割区域的左上角坐标和宽高
////////void crop_img(cv::Mat &img, cv::Mat &crop_img, std::vector<int> &area) {
////////
////////    int crop_x1 = std::max(0, area[0]);
////////    int crop_y1 = std::max(0, area[1]);
////////    int crop_x2 = std::min(img
////
////
////
////#include <opencv2/opencv.hpp>
////#include <iostream>
////
////using namespace cv;
////using namespace std;
////
////int main()
////{
////    // 创建一个黑色背景的图像
////    Mat img = Mat::zeros(640, 480, CV_8UC3);
////
////    // 定义三角形三个顶点的坐标
////    Point points[1][2];
////    points[0][0] = Point(100, 100);
//////    points[0][1] = Point(150, 200);
////    points[0][1] = Point(200, 200);
////
////
////    // 定义三角形的颜色
////    Scalar color(0, 255, 0);
////
////    // 将三角形的顶点坐标存储到数组中
////    const Point* ppts[1] = {points[0]};
////    const int npts[] = {4};
////    // 在图像上绘制实心三角形
////    fillPoly(img, ppts, npts, 1, color);
////
////    // 显示结果
////    imshow("实心三角形", img);
////    waitKey(0);
////
////    return 0;
////}
////huofu under this
////
//// Created by liheng on 2019/2/24.
//// Program:霍夫P直线检测示例
////Data:2019.2.24
////Author:liheng
////Version:V1.0
////
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include <iostream>
//#include <cmath>
//using namespace cv;
//using namespace std;
///*
//RGB转换成灰度图像的一个常用公式是：
//Gray = R*0.299 + G*0.587 + B*0.114
//*/
////******************灰度转换函数*************************
////第一个参数image输入的彩色RGB图像的引用；
////第二个参数imageGray是转换后输出的灰度图像的引用；
////*******************************************************
//void ConvertRGB2GRAY(const Mat &image, Mat &imageGray);
//
////******************Sobel卷积因子计算X、Y方向梯度和梯度方向角********************
////第一个参数imageSourc原始灰度图像；
////第二个参数imageSobelX是X方向梯度图像；
////第三个参数imageSobelY是Y方向梯度图像；
////第四个参数pointDrection是梯度方向角数组指针
////*************************************************************
//void SobelGradDirction(Mat &imageSource, Mat &imageSobelX, Mat &imageSobelY);
//
////******************计算Sobel的X方向梯度幅值的平方*************************
////第一个参数imageGradX是X方向梯度图像；
////第二个参数SobelAmpXX是输出的X方向梯度图像的平方
////*************************************************************
//void SobelXX(const Mat imageGradX, Mat_<float> &SobelAmpXX);
//
////******************计算Sobel的Y方向梯度幅值的平方*************************
////第一个参数imageGradY是Y方向梯度图像；
////第二个参数SobelAmpXX是输出的Y方向梯度图像的平方
////*************************************************************
//void SobelYY(const Mat imageGradY, Mat_<float> &SobelAmpYY);
//
////******************计算Sobel的XY方向梯度幅值的乘积*************************
////第一个参数imageGradX是X方向梯度图像；
////第二个参数imageGradY是Y方向梯度图像；
////第二个参数SobelAmpXY是输出的XY方向梯度图像
////*************************************************************
//void SobelXY(const Mat imageGradX, const Mat imageGradY, Mat_<float> &SobelAmpXY);
//
////****************计算一维高斯的权值数组*****************
////第一个参数size是代表的卷积核的边长的大小
////第二个参数sigma表示的是sigma的大小
////*******************************************************
//double *getOneGuassionArray(int size, double sigma);
//
////****************高斯滤波函数的实现*****************
////第一个参数srcImage是代表的输入的原图
////第二个参数dst表示的是输出的图
////第三个参数size表示的是卷积核的边长的大小
////*******************************************************
//void MyGaussianBlur(Mat_<float> &srcImage, Mat_<float> &dst, int size);
//
////****计算局部特涨结果矩阵M的特征值和响应函数H = (A*B - C) - k*(A+B)^2******
////M
////A  C
////C  B
////Tr(M)=a+b=A+B
////Det(M)=a*b=A*B-C^2
////计算输出响应函数的值得矩阵
////****************************************************************************
//void harrisResponse(Mat_<float> &GaussXX, Mat_<float> &GaussYY, Mat_<float> &GaussXY, Mat_<float> &resultData,float k);
//
//
////***********非极大值抑制和满足阈值及某邻域内的局部极大值为角点**************
////第一个参数是响应函数的矩阵
////第二个参数是输入的灰度图像
////第三个参数表示的是输出的角点检测到的结果图
//void LocalMaxValue(Mat_<float> &resultData, Mat &srcGray, Mat &ResultImage,int kSize);
//
//int main()
//{
//    const Mat srcImage = imread("/home/yukki/blue.png");
//    if (!srcImage.data)
//    {
//        printf("could not load image...\n");
//        return -1;
//    }
//    imshow("srcImage", srcImage);
//    Mat srcGray;
//    ConvertRGB2GRAY(srcImage, srcGray);
//    Mat imageSobelX;
//    Mat imageSobelY;
//    Mat resultImage;
//    Mat_<float> imageSobelXX;
//    Mat_<float> imageSobelYY;
//    Mat_<float> imageSobelXY;
//    Mat_<float> GaussianXX;
//    Mat_<float> GaussianYY;
//    Mat_<float> GaussianXY;
//    Mat_<float> HarrisRespond;
//    //计算Soble的XY梯度
//    SobelGradDirction(srcGray, imageSobelX, imageSobelY);
//    //计算X方向的梯度的平方
//    SobelXX(imageSobelX, imageSobelXX);
//    SobelYY(imageSobelY, imageSobelYY);
//    SobelXY(imageSobelX, imageSobelY, imageSobelXY);
//    //计算高斯模糊XX YY XY
//    MyGaussianBlur(imageSobelXX, GaussianXX,3);
//    MyGaussianBlur(imageSobelYY, GaussianYY, 3);
//    MyGaussianBlur(imageSobelXY, GaussianXY, 3);
//    harrisResponse(GaussianXX, GaussianYY, GaussianXY, HarrisRespond, 0.05);
//    LocalMaxValue(HarrisRespond, srcGray, resultImage, 3);
//    imshow("imageSobelX", imageSobelX);
//    imshow("imageSobelY", imageSobelY);
//    imshow("resultImage", resultImage);
//    waitKey(0);
//    return 0;
//}
//void ConvertRGB2GRAY(const Mat &image, Mat &imageGray)
//{
//    if (!image.data || image.channels() != 3)
//    {
//        return;
//    }
//    //创建一张单通道的灰度图像
//    imageGray = Mat::zeros(image.size(), CV_8UC1);
//    //取出存储图像像素的数组的指针
//    uchar *pointImage = image.data;
//    uchar *pointImageGray = imageGray.data;
//    //取出图像每行所占的字节数
//    size_t stepImage = image.step;
//    size_t stepImageGray = imageGray.step;
//    for (int i = 0; i < imageGray.rows; i++)
//    {
//        for (int j = 0; j < imageGray.cols; j++)
//        {
//            pointImageGray[i*stepImageGray + j] = (uchar)(0.114*pointImage[i*stepImage + 3 * j] + 0.587*pointImage[i*stepImage + 3 * j + 1] + 0.299*pointImage[i*stepImage + 3 * j + 2]);
//        }
//    }
//}
//
//
////存储梯度膜长
//void SobelGradDirction(Mat &imageSource, Mat &imageSobelX, Mat &imageSobelY)
//{
//    imageSobelX = Mat::zeros(imageSource.size(), CV_32SC1);
//    imageSobelY = Mat::zeros(imageSource.size(), CV_32SC1);
//    //取出原图和X和Y梯度图的数组的首地址
//    uchar *P = imageSource.data;
//    uchar *PX = imageSobelX.data;
//    uchar *PY = imageSobelY.data;
//
//    //取出每行所占据的字节数
//    int step = imageSource.step;
//    int stepXY = imageSobelX.step;
//
//    int index = 0;//梯度方向角的索引
//    for (int i = 1; i < imageSource.rows - 1; ++i)
//    {
//        for (int j = 1; j < imageSource.cols - 1; ++j)
//        {
//            //通过指针遍历图像上每一个像素
//            double gradY = P[(i + 1)*step + j - 1] + P[(i + 1)*step + j] * 2 + P[(i + 1)*step + j + 1] - P[(i - 1)*step + j - 1] - P[(i - 1)*step + j] * 2 - P[(i - 1)*step + j + 1];
//            PY[i*stepXY + j*(stepXY / step)] = abs(gradY);
//
//            double gradX = P[(i - 1)*step + j + 1] + P[i*step + j + 1] * 2 + P[(i + 1)*step + j + 1] - P[(i - 1)*step + j - 1] - P[i*step + j - 1] * 2 - P[(i + 1)*step + j - 1];
//            PX[i*stepXY + j*(stepXY / step)] = abs(gradX);
//        }
//    }
//    //将梯度数组转换成8位无符号整型
//    convertScaleAbs(imageSobelX, imageSobelX);
//    convertScaleAbs(imageSobelY, imageSobelY);
//}
//
//
//void SobelXX(const Mat imageGradX, Mat_<float> &SobelAmpXX)
//{
//    SobelAmpXX = Mat_<float>(imageGradX.size(), CV_32FC1);
//    for (int i = 0; i < SobelAmpXX.rows; i++)
//    {
//        for (int j = 0; j < SobelAmpXX.cols; j++)
//        {
//            SobelAmpXX.at<float>(i, j) = imageGradX.at<uchar>(i, j)*imageGradX.at<uchar>(i, j);
//        }
//    }
//    //convertScaleAbs(SobelAmpXX, SobelAmpXX);
//}
//
//void SobelYY(const Mat imageGradY, Mat_<float> &SobelAmpYY)
//{
//    SobelAmpYY = Mat_<float>(imageGradY.size(), CV_32FC1);
//    for (int i = 0; i < SobelAmpYY.rows; i++)
//    {
//        for (int j = 0; j < SobelAmpYY.cols; j++)
//        {
//            SobelAmpYY.at<float>(i, j) = imageGradY.at<uchar>(i, j)*imageGradY.at<uchar>(i, j);
//        }
//    }
//    //convertScaleAbs(SobelAmpYY, SobelAmpYY);
//}
//
//void SobelXY(const Mat imageGradX, const Mat imageGradY, Mat_<float> &SobelAmpXY)
//{
//    SobelAmpXY = Mat_<float>(imageGradX.size(), CV_32FC1);
//    for (int i = 0; i < SobelAmpXY.rows; i++)
//    {
//        for (int j = 0; j < SobelAmpXY.cols; j++)
//        {
//            SobelAmpXY.at<float>(i, j) = imageGradX.at<uchar>(i, j)*imageGradY.at<uchar>(i, j);
//        }
//    }
//    //convertScaleAbs(SobelAmpXY, SobelAmpXY);
//}
//
//
//
////计算一维高斯的权值数组
//double *getOneGuassionArray(int size, double sigma)
//{
//    double sum = 0.0;
//    //定义高斯核半径
//    int kerR = size / 2;
//
//    //建立一个size大小的动态一维数组
//    double *arr = new double[size];
//    for (int i = 0; i < size; i++)
//    {
//
//        // 高斯函数前的常数可以不用计算，会在归一化的过程中给消去
//        arr[i] = exp(-((i - kerR)*(i - kerR)) / (2 * sigma*sigma));
//        sum += arr[i];//将所有的值进行相加
//
//    }
//    //进行归一化
//    for (int i = 0; i < size; i++)
//    {
//        arr[i] /= sum;
//        cout << arr[i] << endl;
//    }
//    return arr;
//}
//
//void MyGaussianBlur(Mat_<float> &srcImage, Mat_<float> &dst, int size)
//{
//    CV_Assert(srcImage.channels() == 1 || srcImage.channels() == 3); // 只处理单通道或者三通道图像
//    int kerR = size / 2;
//    dst = srcImage.clone();
//    int channels = dst.channels();
//    double* arr;
//    arr = getOneGuassionArray(size, 1);//先求出高斯数组
//
//    //遍历图像 水平方向的卷积
//    for (int i = kerR; i < dst.rows - kerR; i++)
//    {
//        for (int j = kerR; j < dst.cols - kerR; j++)
//        {
//            float GuassionSum[3] = { 0 };
//            //滑窗搜索完成高斯核平滑
//            for (int k = -kerR; k <= kerR; k++)
//            {
//
//                if (channels == 1)//如果只是单通道
//                {
//                    GuassionSum[0] += arr[kerR + k] * dst.at<float>(i, j + k);//行不变，列变换，先做水平方向的卷积
//                }
//                else if (channels == 3)//如果是三通道的情况
//                {
//                    Vec3f bgr = dst.at<Vec3f>(i, j + k);
//                    auto a = arr[kerR + k];
//                    GuassionSum[0] += a*bgr[0];
//                    GuassionSum[1] += a*bgr[1];
//                    GuassionSum[2] += a*bgr[2];
//                }
//            }
//            for (int k = 0; k < channels; k++)
//            {
//                if (GuassionSum[k] < 0)
//                    GuassionSum[k] = 0;
//                else if (GuassionSum[k] > 255)
//                    GuassionSum[k] = 255;
//            }
//            if (channels == 1)
//                dst.at<float>(i, j) = static_cast<float>(GuassionSum[0]);
//            else if (channels == 3)
//            {
//                Vec3f bgr = { static_cast<float>(GuassionSum[0]), static_cast<float>(GuassionSum[1]), static_cast<float>(GuassionSum[2]) };
//                dst.at<Vec3f>(i, j) = bgr;
//            }
//
//        }
//    }
//
//    //竖直方向
//    for (int i = kerR; i < dst.rows - kerR; i++)
//    {
//        for (int j = kerR; j < dst.cols - kerR; j++)
//        {
//            float GuassionSum[3] = { 0 };
//            //滑窗搜索完成高斯核平滑
//            for (int k = -kerR; k <= kerR; k++)
//            {
//
//                if (channels == 1)//如果只是单通道
//                {
//                    GuassionSum[0] += arr[kerR + k] * dst.at<float>(i + k, j);//行变，列不换，再做竖直方向的卷积
//                }
//                else if (channels == 3)//如果是三通道的情况
//                {
//                    Vec3f bgr = dst.at<Vec3f>(i + k, j);
//                    auto a = arr[kerR + k];
//                    GuassionSum[0] += a*bgr[0];
//                    GuassionSum[1] += a*bgr[1];
//                    GuassionSum[2] += a*bgr[2];
//                }
//            }
//            for (int k = 0; k < channels; k++)
//            {
//                if (GuassionSum[k] < 0)
//                    GuassionSum[k] = 0;
//                else if (GuassionSum[k] > 255)
//                    GuassionSum[k] = 255;
//            }
//            if (channels == 1)
//                dst.at<float>(i, j) = static_cast<float>(GuassionSum[0]);
//            else if (channels == 3)
//            {
//                Vec3f bgr = { static_cast<float>(GuassionSum[0]), static_cast<float>(GuassionSum[1]), static_cast<float>(GuassionSum[2]) };
//                dst.at<Vec3f>(i, j) = bgr;
//            }
//
//        }
//    }
//    delete[] arr;
//}
//
//void harrisResponse(Mat_<float> &GaussXX, Mat_<float> &GaussYY, Mat_<float> &GaussXY, Mat_<float> &resultData,float k)
//{
//    //创建一张响应函数输出的矩阵
//    resultData = Mat_<float>(GaussXX.size(), CV_32FC1);
//    for (int i = 0; i < resultData.rows; i++)
//    {
//        for (int j = 0; j < resultData.cols; j++)
//        {
//            float a = GaussXX.at<float>(i, j);
//            float b = GaussYY.at<float>(i, j);
//            float c = GaussXY.at<float>(i, j);
//            resultData.at<float>(i, j) = a*b - c*c - k*(a + b)*(a + b);
//        }
//    }
//}
//
//
////非极大值抑制
//void LocalMaxValue(Mat_<float> &resultData, Mat &srcGray, Mat &ResultImage, int kSize)
//{
//    int r = kSize / 2;
//    ResultImage = srcGray.clone();
//    for (int i = r; i < ResultImage.rows - r; i++)
//    {
//        for (int j = r; j < ResultImage.cols - r; j++)
//        {
//            if (resultData.at<float>(i, j) > resultData.at<float>(i - 1, j - 1) &&
//                resultData.at<float>(i, j) > resultData.at<float>(i - 1, j) &&
//                resultData.at<float>(i, j) > resultData.at<float>(i - 1, j - 1) &&
//                resultData.at<float>(i, j) > resultData.at<float>(i - 1, j + 1) &&
//                resultData.at<float>(i, j) > resultData.at<float>(i, j - 1) &&
//                resultData.at<float>(i, j) > resultData.at<float>(i, j + 1) &&
//                resultData.at<float>(i, j) > resultData.at<float>(i + 1, j - 1) &&
//                resultData.at<float>(i, j) > resultData.at<float>(i + 1, j) &&
//                resultData.at<float>(i, j) > resultData.at<float>(i + 1, j + 1))
//            {
//                if ((int)resultData.at<float>(i, j) > 18000)
//                {
//                    circle(ResultImage, Point(i, j), 5, Scalar(0,0,255), 2, 8, 0);
//                }
//            }
//
//        }
//    }
//}


