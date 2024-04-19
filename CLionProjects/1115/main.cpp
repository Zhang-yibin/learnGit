//////#include <iostream>
//////
//////#include<opencv2/opencv.hpp>
//////
//////#include <opencv2/imgproc/imgproc.hpp>
//////
//////#include <opencv2/highgui/highgui.hpp>
//////
//////#define PI 3.1415926
//////
//////using namespace cv;
//////
//////using namespace std;
//////
//////void RGB2HSV(double red, double green, double blue, double& hue, double& saturation, double& intensity)
//////
//////{
//////
//////    double r, g, b;
//////
//////    double h, s, i;
//////
//////    double sum;
//////
//////    double minRGB, maxRGB;
//////
//////    double theta;
//////
//////    r = red / 255.0;
//////
//////    g = green / 255.0;
//////
//////    b = blue / 255.0;
//////
//////    minRGB = ((r<g) ? (r) : (g));
//////
//////    minRGB = (minRGB<b) ? (minRGB) : (b);
//////
//////    maxRGB = ((r>g) ? (r) : (g));
//////
//////    maxRGB = (maxRGB>b) ? (maxRGB) : (b);
//////
//////    sum = r + g + b;
//////
//////    i = sum / 3.0;
//////
//////    if (i<0.001 || maxRGB - minRGB<0.001)
//////
//////    {
//////
//////        h = 0.0;
//////
//////        s = 0.0;
//////
//////    }
//////
//////    else
//////
//////    {
//////
//////        s = 1.0 - 3.0*minRGB / sum;
//////
//////        theta = sqrt((r - g)*(r - g) + (r - b)*(g - b));
//////
//////        theta = acos((r - g + r - b)*0.5 / theta);
//////
//////        if (b <= g)
//////
//////            h = theta;
//////
//////        else
//////
//////            h = 2 * PI - theta;
//////
//////        if (s <= 0.01)
//////
//////            h = 0;
//////
//////    }
//////
//////    hue = (int)(h * 180 / PI);
//////
//////    saturation = (int)(s * 100);
//////
//////    intensity = (int)(i * 100);
//////
//////}
//////
//////Mat picture_red(Mat input)
//////
//////{
//////
//////    Mat frame;
//////
//////    Mat srcImg = input;
//////
//////    frame = srcImg;
//////
//////    waitKey(1);
//////
//////    int width = srcImg.cols;
//////
//////    int height = srcImg.rows;
//////
//////    int x, y;
//////
//////    double B = 0.0, G = 0.0, R = 0.0, H = 0.0, S = 0.0, V = 0.0;
//////
//////    Mat vec_rgb = Mat::zeros(srcImg.size(), CV_8UC1);
//////
//////    for (x = 0; x < height; x++)
//////
//////    {
//////
//////        for (y = 0; y < width; y++)
//////
//////        {
//////
//////            B = srcImg.at<Vec3b>(x, y)[0];
//////
//////            G = srcImg.at<Vec3b>(x, y)[1];
//////
//////            R = srcImg.at<Vec3b>(x, y)[2];
//////
//////            RGB2HSV(R, G, B, H, S, V);
//////
//////            //红色范围，范围参考的网上。可以自己调
//////
//////            if ((H >= 312 && H <= 360) && (S >= 17 && S <= 100) && (V>18 && V < 100))
//////
//////                vec_rgb.at<uchar>(x, y) = 255;
//////
//////            /*cout << H << "," << S << "," << V << endl;*/
//////
//////        }
//////
//////    }
//////
//////    /*imshow("hsv", vec_rgb);*/
//////
//////    return vec_rgb;
//////
//////}
//////
//////void O_x1y1(Mat in, double *x1, double *y1, double *x2, double *y2)
//////
//////{
//////
//////    Mat matSrc = in;
//////
//////    /*Mat matSrc = imread("qwer9.png", 0);*/
//////
//////    GaussianBlur(matSrc, matSrc, Size(5, 5), 0);//高斯滤波，除噪点
//////
//////    vector<vector<Point> > contours;//contours的类型，双重的vector
//////
//////    vector<Vec4i> hierarchy;//Vec4i是指每一个vector元素中有四个int型数据。
//////
//////    //阈值
//////
//////    threshold(matSrc, matSrc, 100, 255, THRESH_BINARY);//图像二值化
//////
//////    //寻找轮廓，这里注意，findContours的输入参数要求是二值图像，二值图像的来源大致有两种，第一种用threshold，第二种用canny
//////
//////    findContours(matSrc.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
//////
//////    /// 计算矩
//////
//////    vector<Moments> mu(contours.size());
//////
//////    for (int i = 0; i < contours.size(); i++)
//////
//////    {
//////
//////        mu
//////                = moments(contours, false);
//////    }
//////
//////    /// 计算矩中心:
//////
//////    vector<Point2f> mc(contours.size());
//////
//////    for (int i = 0; i < contours.size(); i++)
//////
//////    {
//////        mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
//////    }
//////
//////    /// 绘制轮廓
//////
//////    Mat drawing = Mat::zeros(matSrc.size(), CV_8UC1);
//////
//////    for (int i = 0; i < contours.size(); i++)
//////
//////    {
//////        Scalar color = Scalar(255);
//////
//////        //drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());//绘制轮廓函数
//////
//////        circle(drawing, mc, 4, color, -1, 8, 0);
//////
//////    }
//////    *x1 = mc[0].x;
//////    *y1 = mc[0].y;
//////    *x2 = mc[contours.size()-1].x;
//////    *y2 = mc[contours.size() - 1].y;
//////
//////    imshow("outImage", drawing);
//////}
//////
//////int main()
//////{
//////
//////    double xx1, yy1, xx2, yy2;
//////    double x1, y1, x2, y2;
//////
//////    Mat matSrc = imread("qwer4.png");
//////
//////    Mat middle = picture_red(matSrc);
//////    O_x1y1(middle, &xx1, &yy1, &xx2, &yy2);
//////    x1 = xx1;
//////    y1 = yy1;
//////    x2 = xx2;
//////    y2 = yy2;
//////
//////    imshow("原图", matSrc);
//////    imshow("red", picture_red(matSrc));
//////
//////    cout << "红点：" << x1 << "， " << y1 << "; " << "红点1：" << x2 << "， " << y2 << endl;
//////    waitKey();
//////
//////    return 0;
////#include "opencv2/opencv.hpp"
////#include "./common.h"
////#include "./Camera.h"
////#include "utility"
////#include <opencv2/cudaarithm.hpp>
////#include <opencv2/cudaimgproc.hpp>
////#include "opencv2/core/cudafilters.hpp"
////#include "opencv4/opencv2/core/cuda.hpp"
////#include "cudas.h"
////#include "serial/serial.h"
////#include "pkg.h"
////
////
////static_assert(__OS__ == __OS_Linux__);
////
////
////#define TIME 0 // 是否启用时间调试
////#define DRAW 0 // 是否启用绘图调试
////
////#if TIME
////#define REC_T(t) const auto ___time_rec___##t = cv::getTickCount()
////#define PRINT_T(t1, t2) std::cout<< __func__ << " time: " #t1 "~" #t2 " = " << (double(___time_rec___##t2 - ___time_rec___##t1) / cv::getTickFrequency() * 1000) << " ms" << std::endl
////#else
////#define REC_T(t) do{}while(0)
////#define PRINT_T(t1, t2) do{}while(0)
////#endif
////#if DRAW
////
////std::string createWindow(const std::string &name) {
////
////    cv::namedWindow(name, cv::WINDOW_NORMAL);
////    return name;
////}
////
////#define SHOW(name, img) do{static const auto name_1=createWindow(name);cv::imshow(name_1, img);}while(0)
////#else
////#define SHOW(name, img) do{}while(0)
////#endif
////
////
/////// 处理器, 包含NUEDC 2023 E题所有视觉处理的功能
////class Handler {
////#define PAIR_MAT(name) cv::Mat c_##name;cv::cuda::GpuMat g_##name \
/////// 声明一对儿cpu/gpu mat
////#define PM_MALLOC(name)   memMapPool.malloc(c_##name, g_##name, src.size(), src.type()) \
/////// 使用内存映射池为cpu/gpu mat构建映射关系
////public:
////    cv::Point2d green, red;//激光位置
////    std::vector<cv::Point> path;//路径
////private:
////    serial::Serial *serial;
////    ifr::pkg::Move pkg_move;
////
////
////    cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(10, 10));
////    cv::Ptr<cv::cuda::Filter> filter1 = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, CV_8UC1, kernel1);
////    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
////    cv::Ptr<cv::cuda::Filter> filter2 = cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, kernel2);
////private:
////#if DRAW
////
////    FORCE_INLINE void
////    drawRotatedRect(cv::Mat &mat, const cv::RotatedRect &rr, const cv::Scalar &color, int thickness = 1,
////                    int lineType = cv::LINE_8) {
////        cv::Point2f vertices[4];
////        rr.points(vertices);
////
////        // 在图像上绘制旋转矩形
////        for (int i = 0; i < 4; ++i) {
////            cv::line(mat, vertices[i], vertices[(i + 1) % 4], color, thickness, lineType);
////        }
////
////    }
////
////    FORCE_INLINE void
////    drawPoly(cv::Mat &mat, const std::vector<cv::Point> &pts, const cv::Scalar &color, int thickness = 1,
////             int lineType = cv::LINE_8) {
////        const auto size = pts.size();
////        // 在图像上绘制旋转矩形
////        for (std::size_t i = 0; i < size; ++i) {
////            cv::line(mat, pts[i], pts[(i + 1) % size], color, thickness, lineType);
////        }
////
////    }
////
////#endif
////
////
////    PAIR_MAT(gray);
////    PAIR_MAT(otsu);
////    PAIR_MAT(bin);
////    bool has_bin = false;
////
////    inline auto findArea(rm_armor_finder::MemMapPool &memMapPool, const cv::Mat &src, const cv::cuda::GpuMat &gpu) {
////        PM_MALLOC(gray);
////        PM_MALLOC(otsu);
////        PM_MALLOC(bin);
////        REC_T(0);
////        cv::cuda::cvtColor(gpu, g_gray, cv::COLOR_BGR2GRAY);
////        REC_T(1);
////        cv::threshold(c_gray, c_otsu, 30, 255, cv::THRESH_OTSU);
////        REC_T(2);
////        filter1->apply(g_otsu, g_bin);
////
////
////        REC_T(3);
////        ifr::cuda::inv(g_bin);
////
////        REC_T(4);
////        std::vector<std::vector<cv::Point>> contours;
////        std::vector<cv::Vec4i> hierarchy;
////        cv::findContours(c_bin, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
////
////
////        REC_T(5);
////#if DRAW
////        cv::Mat draw;
////        cv::cvtColor(c_bin, draw, cv::COLOR_GRAY2BGR);
////#endif
////
////        std::vector<cv::Point> max_approx, max_parent_approx, parent_approx, child_approx, mid_approx{4};
////        double max_size = -1;
////        for (size_t i = 0; i < contours.size(); i++) {
////            auto child = hierarchy[i][2];
////            if (child < 0)continue;//没有子轮廓
////
////
////            const auto &contour = contours[i];
////            {
////                double epsilon = 0.04 * cv::arcLength(contour, true);
////                cv::approxPolyDP(contour, parent_approx, epsilon, true);
////                if (parent_approx.size() != 4)continue;
////            }
////            do {
////                const auto &child_contour = contours[child];
////                double epsilon = 0.04 * cv::arcLength(child_contour, true);
////                cv::approxPolyDP(child_contour, child_approx, epsilon, true);
////                if (child_approx.size() != 4)continue;
////                double min_dis, max_dis;
////#pragma unroll
////                for (int j = 0; j < 4; j++) {
////                    double dis;
////                    int min_k = 0;
////#pragma unroll
////                    for (int k = 0; k < 4; k++) {
////                        auto vec = parent_approx[j] - child_approx[k];
////                        auto dis0 = vec.ddot(vec);
////                        if (k == 0 || dis0 < dis) {
////                            dis = dis0;
////                            min_k = k;
////                        }
////                    }
////                    mid_approx[j] = (parent_approx[j] + child_approx[min_k]) / 2;
////                    if (j == 0 || dis < min_dis)min_dis = dis;
////                    if (j == 0 || dis > max_dis)max_dis = dis;
////                }
////                if (std::sqrt(max_dis) / std::sqrt(min_dis) < 3)break;//矩形贴合
////            } while ((child = hierarchy[child][0]) >= 0);
////
////#if DRAW
////            drawPoly(draw, parent_approx, {255, 255, 0}, 2, cv::LINE_AA);
////#endif
////            if (child < 0)continue;
////#if DRAW
////            drawPoly(draw, child_approx, {0, 255, 0}, 2, cv::LINE_AA);
////            drawPoly(draw, mid_approx, {0, 255, 255}, 2, cv::LINE_AA);
////#endif
////
////            auto size = cv::contourArea(contour);
////            if (i == 0 || size > max_size) {
////                max_size = size;
////                max_approx = mid_approx;
////                max_parent_approx = parent_approx;
////            }
////
////        }
////
////        REC_T(6);
////
////        if (max_size > 0) {
////            ifr::cuda::keepFill(g_bin, cv::boundingRect(max_parent_approx));
////            has_bin = true;
////        } else {
////            has_bin = false;
////        }
////        path = max_approx;
////        if (!max_approx.empty()) path.push_back(max_approx[0]);
////
////        SHOW("area", draw);
////
////        REC_T(7);
////
////        PRINT_T(0, 1);
////        PRINT_T(1, 2);
////        PRINT_T(2, 3);
////        PRINT_T(3, 4);
////        PRINT_T(4, 5);
////        PRINT_T(5, 6);
////        PRINT_T(6, 7);
////        PRINT_T(0, 7);
////
////    }
////
////    PAIR_MAT(hsv);
////    PAIR_MAT(HSV[3]);
////    PAIR_MAT(pointer);
////    cv::cuda::GpuMat g_pointer_raw;
////
////    inline auto findPointer(rm_armor_finder::MemMapPool &memMapPool, const cv::Mat &src, const cv::cuda::GpuMat &gpu) {
////        PM_MALLOC(hsv);
////#pragma unroll
////        for (int i = 0; i < 3; i++)
////            PM_MALLOC(HSV[i]);
////        PM_MALLOC(pointer);
////        if (g_pointer_raw.empty())g_pointer_raw.create(src.size(), CV_8UC1);
////
////        cv::cuda::cvtColor(gpu, g_hsv, cv::COLOR_BGR2HSV);
////        cv::cuda::split(g_hsv, g_HSV);
////
////        ifr::cuda::findPointer(g_HSV[2], has_bin ? &g_bin : nullptr, g_pointer_raw);
////        filter2->apply(g_pointer_raw, g_pointer);
////
////        std::vector<std::vector<cv::Point>> contours;
////        cv::findContours(c_pointer, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
////
////#if DRAW
////        cv::Mat draw;
////        cv::cvtColor(c_pointer, draw, cv::COLOR_GRAY2BGR);
////#endif
////
////        green = {-1, -1};
////        red = {-1, -1};
////        double size_green = 0, size_red = 0;
////        for (const auto &contour: contours) {
////            double area = cv::contourArea(contour);
////            if (area > 500 || area < 5)continue;
////            cv::Rect rect = cv::boundingRect(contour);
////
////            double aspectRatio = static_cast<double>(rect.width) / rect.height;
////            if (aspectRatio < 0.5 || aspectRatio > 2.0)continue;
////            cv::Point2d center{rect.x + rect.width / 2.0, rect.y + rect.height / 2.0};
////            const auto side = (std::max)(rect.width, rect.height);//最大边长
////
////            int type = 0;
////            const int fx = (std::max)(rect.x - side, 0), fy = (std::max)(rect.y - side, 0);
////            const int tx = (std::min)(rect.x + rect.width + side, c_pointer.rows), ty = (std::min)(
////                    rect.y + rect.height + side, c_pointer.cols);
////
////            for (auto y = fy; y < ty; ++y) {
////                for (auto x = fx; x < tx; ++x) {
////                    if (c_pointer.at<uchar>(y, x))continue;//太亮了
////                    const auto &v = c_HSV[2].at<uchar>(y, x);
////                    if (v < 10)continue;//太暗了
////                    const auto &s = c_HSV[1].at<uchar>(y, x);
////                    if (s < 60)continue;//饱和度太低了(过滤白色)
////                    const auto &h = c_HSV[0].at<uchar>(y, x);
////
////                    //[black,red,green,blue,white]
////                    //[{0,   0,   0}, {0, 255, 255}, {60, 255, 255}, {120, 255, 255}, {0,   0, 255}]
////                    auto r = (std::min)(int(h), 180 - h);
////                    auto g = std::abs(60 - h);
////                    if (r < 10)--type;
////                    else if (g < 10)++type;
////                }
////            }
////
////            if (type > 3) {
////                if (area > size_green) {
////                    green = center;
////                    size_green = area;
////                }
////            } else if (type < 3) {
////                if (area > size_red) {
////                    red = center;
////                    size_red = area;
////                }
////            }
////        }
////
////#if DRAW
////        if (green.x >= 0 && green.y >= 0)
////            cv::circle(draw, green, 5, {0, 255, 0}, -1, cv::LINE_AA);
////        if (red.x >= 0 && red.y >= 0)
////            cv::circle(draw, red, 5, {0, 0, 255}, -1, cv::LINE_AA);
////#endif
////
////        SHOW("pointer", draw);
////
////    }
////
////public:
////    Handler() {
////        serial = new serial::Serial(
////                "/dev/ttyUSB0",
////                115200,
////                serial::Timeout::simpleTimeout(500),
////                static_cast<serial::bytesize_t>(8),
////                static_cast<serial::parity_t>(0),
////                static_cast<serial::stopbits_t>(1),
////                static_cast<serial::flowcontrol_t>(0));
////        IFR_LOG_STREAM("Serial", "serial open: " << serial->isOpen());
////        if (!serial->isOpen())throw std::runtime_error("Can not open serial port");
////    }
////
////    void handler(rm_armor_finder::MemMapPool &memMapPool, cv::Mat &img, cv::cuda::GpuMat &gpu, bool flip = false) {
////        REC_T(1);
////        if (flip)
////            cv::cuda::flip(gpu, gpu, -1);  // 使用负数表示旋转180度
////        REC_T(2);
////        cv::cuda::cvtColor(gpu, gpu, cv::COLOR_BayerRG2BGR);
////        REC_T(3);
////        findArea(memMapPool, img, gpu);
////        REC_T(4);
////        if (!path.empty()) findPointer(memMapPool, img, gpu);
////        else red = green = {-1, -1};
////        REC_T(5);
////
////        PRINT_T(1, 2);
////        PRINT_T(2, 3);
////        PRINT_T(3, 4);
////        PRINT_T(4, 5);
////        PRINT_T(1, 5);
////    }
////
////private:
////    int step = -1;///< red当前目标
////    cv::Point2d end;///< red最终位置
////    const double min_distance = 20.0; ///< red到达判定
////    const double min_distance2 = std::pow(min_distance, 2); ///< red到达判定
////private:
////
////    /**
////     * @brief 计算移动向量
////     * @param go 目标点
////     * @param now 当前点
////     * @param mask 可用路径遮罩
////     * @param[out] finish 是否完成
////     */
////    inline auto calcMove(cv::Point2d go, const cv::Point2d &now, const cv::Mat &mask, bool &finish) const {
////        static constexpr const auto r90 = 90 * CV_PI / 180.0;
////        go -= now;
////        if ((finish = (go.ddot(go) < min_distance2)))return go;
////        go /= cv::norm(go);
////        cv::Rect rect{0, 0, mask.cols, mask.rows};
////        if (!now.inside(rect))return go * min_distance; //不太可能
////
////        const double angle = std::atan2(go.y - now.y, go.x - now.x) + r90;
////        const auto in_path = mask.at<uchar>(int(now.y), int(now.x));
////        const cv::Point2d detla{std::cos(angle), std::sin(angle)};
////
////        int dis_l = -1, dis_r = -1;
////        for (int i = 0, max_val = mask.size().area(); i < max_val; i++) {//从now开始向垂直于前进方向两侧寻找第一个不同点
////            if (dis_l < 0) {
////                auto p1 = now - (i * detla);
////                if (p1.inside(rect) && mask.at<uchar>(int(p1.y), int(p1.x)) != in_path) {
////                    dis_l = i;
////                    if (!in_path)break;//now 本身在黑条上
////                }
////            }
////            if (dis_r < 0) {
////                auto p2 = now + (i * detla);
////                if (p2.inside(rect) && mask.at<uchar>(int(p2.y), int(p2.x)) != in_path) {
////                    dis_r = i;
////                    if (!in_path)break;//now 本身在黑条上
////                }
////            }
////            if (dis_l >= 0 && dis_r >= 0)break;
////        }
////
////        if (in_path) {//在路径中, 以小纠正得到结果
////            if (dis_l >= 0 && dis_r >= 0) {
////                auto fix = (dis_l < dis_r ? 1 : -1) * detla;
////                go = go * 4 + fix;
////            }
////        } else {//不在路径中, 以大偏移量最快返回
////            if (dis_l >= 0 || dis_r >= 0) {
////                static constexpr const auto max_val = std::numeric_limits<int>::max();
////                if (dis_l < 0)dis_l = max_val;
////                if (dis_r < 0)dis_r = max_val;
////                auto fix = (dis_l < dis_r ? -1 : 1) * detla;
////                go = fix * 2 + go;
////            }
////        }
////        go /= cv::norm(go) * min_distance;
////        return go;
////    }
////
////    /// red 帮助函数
////    /// 尝试计算到目标点的距离, 并自动更新step
////    cv::Point2d move_step(bool &finish) {
////        if (step < 0) {
////            end = red;
////            step = 0;
////        }
////        if (step < path.size()) {
////#if 0
////            cv::Point2d go = calcMove(path[step], red, c_bin, finish);
////#else
////            cv::Point2d go = path[step];
////            go -= red;
////            go = -go;
////            finish = go.ddot(go) < min_distance2;
////#endif
////            if (finish) step++;
////            finish = false;//单步完成不是全部完成
////            return go;
////        }
////        auto go = end - red;
////        finish = go.ddot(go) < min_distance;
////        return go;
////    }
////
////    /// green 帮助函数
////    CV_NODISCARD_STD cv::Point2d follow() const {
////        if (red.x < 0 || red.y < 0 || green.x < 0 || green.y < 0)return {};
////        return green - red;
////    }
////
////public:
////    /// @brief 执行一次绿色激光笔的代码逻辑: 计算移动+设置数据
////    void do_green() {
////        auto go = follow();
////        pkg_move.set(go.x, go.y);
////        IFR_LOG_STREAM("DO", go << ", red = " << red << ", green = " << green << ", path = " << path.size());
////    }
////
////    /// @brief 执行一次红色激光笔的代码逻辑: 计算移动+串口发送
////    bool do_red() {
////        cv::Point2d go;
////        bool finish = false;
////        if (red.x < 0 || red.y < 0) {
////        } else {
////            go = move_step(finish);
////            if (finish) {
////                step = -1;
////                go = {0, 0};
////            }
////        }
////        pkg_move.set(go.x, go.y);
////        IFR_LOG_STREAM("DO",
////                       go << ", red = " << red << ", green = " << green << ", path = " << path.size() << ", finish = "
////                          << (finish ? "true" : "false") << ", step = " << step);
////        return finish;
////    }
////
////    void sendMove() {
////        serial->writeT(pkg_move);
////    }
////
////    void resetMove() {
////        pkg_move.set(0, 0);
////    }
////
////private:
////    std::string read_buf;///串口缓冲
////public:
////    /**
////     * 尝试读取字符串, 并将maps中对应值设为true
////     * @param maps 关键词到对应值的映射
////     */
////    void tryRead(std::unordered_map<std::string, bool *> &maps) {
////        auto size = serial->available();
////        if (size > 0) {
////            read_buf += serial->read(size);
////            size_t pos = -1;
////            while ((pos = read_buf.find('\n')) != std::string::npos) {
////                size_t start = 0;
////                while (start < pos && read_buf[start] == '\0') start++;
////                auto line = read_buf.substr(start, pos - start);
////                read_buf = read_buf.substr(pos + 1);
////                if (!line.empty()) {
////                    auto &ptr = maps[line];
////                    if (ptr != nullptr && !(*ptr)) {
////                        IFR_LOG_STREAM("serial", "成功读取: " << line);
////                        *ptr = true;
////                    }
////                }
////            }
////            if (read_buf.size() > 256)read_buf = "";//过长保护
////        }
////    }
////};
////
/////// @brief 执行完整代码逻辑
/////// @details 建立 相机、串口、处理器 之间的桥接
////void run() {
////    ifr::Camera camera(2000, false, 100);
////    camera.initCamera();
////    camera.runCamera();
////
////    Handler handler;
////
////    bool is_red = false, is_green = false, is_stop = false;
////    std::unordered_map<std::string, bool *> keyword = {
////            {"1_ihw9jnsh39m", &is_green},
////            {"2_9kitey3yzpd", &is_red},
////            {"3_yp4lmg19kbc", &is_stop},
////    };
////
////    int64 lstTick = 0, now;
////    while (true) {
////        PGX_FRAME_BUFFER pFrameBuffer;
////
////        IFR_GX_CHECK(GXDQBuf(camera.m_hDevice, &pFrameBuffer, 500));
////        if (pFrameBuffer->nStatus == GX_FRAME_STATUS_SUCCESS) {
////            cv::Mat src(pFrameBuffer->nHeight, pFrameBuffer->nWidth, CV_8UC1,
////                        reinterpret_cast<void *>(pFrameBuffer->pImgBuf));
////            void *cuda_src_ptr = camera.memMapPool.getDevicePointer(src.data, src.dataend - src.datastart);
////            cv::cuda::GpuMat gpu_src(src.rows, src.cols, src.type(), cuda_src_ptr);
////
////            handler.handler(camera.memMapPool, src, gpu_src, true);
////
////            handler.tryRead(keyword);
////            if ((is_red && is_green) || is_stop) {
////                handler.resetMove();
////                is_red = is_green = is_stop = false;
////            }
////            is_red = true;
////            if (is_red) {
////                if (handler.do_red())is_red = false;
////            }
////            if (is_green) {
////                handler.do_green();
////            }
////        }
////        IFR_GX_CHECK(GXQBuf(camera.m_hDevice, pFrameBuffer));
////
////        while (double((now = cv::getTickCount()) - lstTick) / cv::getTickFrequency() * 1000 < 10.0);
////        handler.sendMove();
////        lstTick = now;
////    }
////}
////
////
////int main() {
////    ifr::logger::setMainPath(__FILE__);
//////    {
//////        cv::Mat x(cv::Size{5, 1}, CV_8UC3, {0, 0, 0});
//////        x.at<cv::Vec3b>(0, 0) = {0, 0, 0};
//////        x.at<cv::Vec3b>(0, 1) = {255, 0, 0};
//////        x.at<cv::Vec3b>(0, 2) = {0, 255, 0};
//////        x.at<cv::Vec3b>(0, 3) = {0, 0, 255};
//////        x.at<cv::Vec3b>(0, 4) = {255, 255, 255};
//////        std::cout << x << std::endl;
//////        cv::cvtColor(x, x, cv::COLOR_RGB2HSV);
//////        std::cout << x << std::endl;
//////    }
//////
//////    return 0;
////    run();
////
////    return 0;
////}
//#include "opencv2/opencv.hpp"
//using namespace cv;
//
//using namespace std;
//Mat imgMoment(Mat n)//27图像的矩
//{
//    //1.查找轮廓前的预处理
//    Mat img=n.clone();
//    Mat copyImg = img.clone();
//   // cvtColor(img, img, COLOR_BGR2GRAY);
//    //threshold(img, img, 100, 255, THRESH_BINARY_INV);
//    imshow("thresh", img);
//    //2.查找轮廓
//    vector<vector<Point>> contours;
//    findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);//最外层轮廓
//    drawContours(copyImg, contours, -1, Scalar(0, 255, 0), 2, 8);
//    //3.轮廓矩的计算
//    Moments moments0 = moments(contours[0], false);//计算轮廓矩
//    cout << moments0.m00 << endl;//输出空间矩之一的m00
//    //4添加文字
//    char outText[10] = { 0 };
//    double outValue = moments0.m00;
//    sprintf(outText, "%.1f", outValue);
//    putText(copyImg, outText, Point(10, 50),//起始点
//            FONT_HERSHEY_COMPLEX, //字体
//            2, //字体大小
//            Scalar(0, 255, 0), 2, 8);
//    imshow("contours", copyImg);
//    waitKey(0);
//    return copyImg;
//}
//int main() {
//    Mat dst;
//    Mat img1 = imread("/home/yuuki/Downloads/4.jpeg");
////
////    inRange(img1, Scalar(0, 43, 46), Scalar(10, 255, 255), dst);
//    inRange(img1, Scalar (150,43,46),Scalar(180,255,255), dst);
//    imshow("???",dst);
//  //  imgMoment(dst);
//    imshow("1",dst);
//    waitKey();
//    return 0;
//
//}
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>


using namespace cv;
using namespace std;


int main(int argc, char** argv)
{
    Mat img;
    VideoCapture video("/home/yuuki/Downloads/1104.mp4");
    for (;;) {
        Rect point_array[20];
        video >> img;
        if (img.empty()) {
            break;
        }
        // Mat img = imread("/home/yuuki/1.bmp", -1);
        resize(img,img,Size(),0.3,0.3);
        pyrDown(img, img, Size(img.cols / 2, img.rows / 2), 4);
        imshow("img", img);
        //imwrite("img.jpg", img);

        //通过canny算法找轮廓，这样 findcontours 的结果会好些
        Mat canny_out;
        Canny(img, canny_out, 45, 127, 3, false);
        imshow("canny_out", canny_out);
       // imwrite("canny_out.jpg", canny_out);

        //寻找轮廓
        vector<vector<Point>> contours;
        vector<Vec4i> hierachy;
        findContours(canny_out, contours, hierachy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(-1, -1));
        drawContours(img, contours, -1, Scalar(0, 0, 255), 1, 8, hierachy);

        //定义圆形、方形、旋转矩形、椭圆的存储容器
        vector<vector<Point>> contours_ploy(contours.size());
        vector<Rect> rects_ploy(contours.size());
        vector<Point2f> circle_centers(contours.size());
        vector<float> circle_radius(contours.size());
        vector<RotatedRect> RotatedRect_ploy;//注意：由于下面赋值的过程中有个点数大于5的条件，所以这里没有直接初始化，才有下面pushback的方法添加值。
        vector<RotatedRect> ellipse_ploy;//注意，这里是画椭圆，但是容器类型是 RotatedRect

        //将结果放到各自的容器中
        for (size_t i = 0; i < contours.size(); i++) {
            if (cv::contourArea(contours[i]) < 6000) {//||cv::contourArea(contours[n]) > 17999){
                continue;
            }
            approxPolyDP(contours[i], contours_ploy[i], 5, true);
            rects_ploy[i] = boundingRect(contours_ploy[i]);
            minEnclosingCircle(contours_ploy[i], circle_centers[i], circle_radius[i]);

            if (contours_ploy[i].size() > 5) {
                RotatedRect temp1 = minAreaRect(contours_ploy[i]);
                RotatedRect_ploy.push_back(temp1);

                RotatedRect temp2 = fitEllipse(contours_ploy[i]);
                ellipse_ploy.push_back(temp2);
            }
        }

        //定义最终绘图的图片
        Mat draw_rect(img.size(), img.type(), Scalar::all(0)),
                draw_rotateRect(img.size(), img.type(), Scalar::all(0));
        //  draw_circle(img.size(), img.type(), Scalar::all(0)),
        //   draw_ellipse(img.size(), img.type(), Scalar::all(0));

        //绘图圆形、矩形
        RNG rng(12345);
        for (size_t i = 0; i < contours.size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            rectangle(draw_rect, rects_ploy[i], color, 1, 8);
            // circle(draw_circle, circle_centers[i], circle_radius[i], color, 1, 8);
        }
        imshow("draw_rect", draw_rect);
       // imwrite("draw_rect.jpg", draw_rect);
        // imshow("draw_circle", draw_circle);imwrite("draw_circle.jpg", draw_circle);

        //绘图椭圆形、旋转矩形
        Point2f pot[4];
        for (size_t i = 0; i < ellipse_ploy.size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            //ellipse(draw_ellipse, ellipse_ploy[i], color, 1, 8);

            RotatedRect_ploy[i].points(pot);
            for (int j = 0; j < 4; j++) {
                line(draw_rotateRect, pot[j], pot[(j + 1) % 4], color);
            }
        }
        // imshow("draw_ellipse", draw_ellipse);imwrite("draw_ellipse.jpg", draw_ellipse);
        imshow("draw_rotateRect", draw_rotateRect);
       // imwrite("draw_rotateRect.jpg", draw_rotateRect);
        if (waitKey(55) >= 0) {
            break;
        }

        waitKey();
    }
    return 1;
}