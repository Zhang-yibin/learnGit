#include<iostream>
#include<opencv2/opencv.hpp>
#include "dec.h"
std::vector<rm_auto_aim::ShootFan> fans;
//#define filename "/home/yukki/LIT BLUE ONE.MP4";
#define filename "/home/yukki/LIT FULL.mp4";
//#define DRAW
#define delay 50
//#define rune_color self_BLUE
//没用pnp
#define rune_color self_RED
//#define videoroi
//yituoshi                  //屎黄色102,204,255    255，204，102  yeyeye
//green 189,240,120
using namespace std;
using namespace cv;
using namespace rm_auto_aim;
//目前蓝色效果不好 针对性的做一个新的识别吧
const static cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
const static cv::Mat kernel5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
const static cv::Mat kernel7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
const static cv::Mat kernel11 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));
rm_auto_aim::RuneDetector::RuneDetector(const RuneParam & r) :
        rune_param(r)
{
}
double RuneDetector::PixelContour(cv::Mat & rot, double start, int height)
{
    double sum = 0;
    for (int i = start; i <     start + height; i++) {
        auto * data = rot.ptr<uchar>(i);
        for (int j = 0; j < rot.cols; j++) {
            if (data[j] == 255) {
                sum += 1;
            }
        }
    }
    return sum;
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
    // cv::waitKey(0);
}
bool RuneDetector::imageProcess(cv::Mat & src)
{
    if (src.empty())
        return false;
    std::vector<cv::Mat> channels;
    rune_debug = src.clone();

    //debug 1st

    cv::split(src, channels);
//实践得出结论：亮度高时均接近白光，反向通道反而可以滤掉圆心，有一定b用
    if (rune_color == self_BLUE)
        binary_img = channels.at(2);
    else
        binary_img = channels.at(1);
    //单通道近似灰度图
//    imshow("通道二值化",binary_img);
    threshold(binary_img, binary_img, rune_param.binary_threshold, 255, cv::THRESH_BINARY);

    morphologyEx(binary_img, binary_img, cv::MORPH_DILATE, kernel3);
//    imshow("膨胀",binary_img);
    return true;
}
//img process 没啥好写的




//预处理->扇叶候选队列
std::vector<rm_auto_aim::ShootFan> rm_auto_aim::RuneDetector::fillContour()
{
    int yukki_fan=0;
    ////创建轮廓填充图 填充轮廓
    filled_contour_img = cv::Mat::zeros(cv::Size(rune_debug.cols, rune_debug.rows), CV_8UC1);
    // 原图大小的黑底
    //永远只有黑白，包括draw
    std::vector<std::vector<cv::Point>> contours;


    ////寻找主要轮廓并填充（把填充的轮廓画在黑底上）
    /// 需要调整area大小
    findContours(binary_img, filled_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //find contours all in before
    for (size_t i = 0; i < filled_contours.size(); i++) {
        double area = contourArea(filled_contours[i]);
        if (area < rune_param.min_filledContours_area) continue;
        //把不符合条件的（小）轮廓给过滤掉
        drawContours(
                filled_contour_img, filled_contours, static_cast<int>(i), cv::Scalar(189,240,120),
                cv::FILLED);
    }//大小合适的轮廓画出来，并塞满

//这里也没啥好改的 虽然有点多余

//    std::vector<ShootFan> fans;

    ////膨胀轮廓使其连续
    //    dilate(filled_contour_img,filled_contour_img,kernel7);
    morphologyEx(filled_contour_img, filled_contour_img, cv::MORPH_CLOSE, kernel3);
//去除细小空洞，感觉没啥用。准备做掉！！！！！！！
//    imshow("what the fuck you find",filled_contour_img);
    //画完的黑底


    //第二次筛轮廓，不知道为啥要这么繁琐的筛，先看着
    findContours(filled_contour_img, filled_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < filled_contours.size(); i++) {
        double area = contourArea(filled_contours[i]);

        drawContours(rune_debug, filled_contours, static_cast<int>(i), cv::Scalar(0, 255, 255), 2);

        //金黄色外围轮廓，r标到底要不要（我觉得不要）

        auto rect = cv::minAreaRect(filled_contours[i]);
        //RRECT rect
        // std::cout<<area<<std::endl;
        //?
        if (area < rune_param.min_contourArea || area > rune_param.max_contourArea) continue;
        //大小

        float ratio = rect.size.width > rect.size.height ? rect.size.width / rect.size.height
                                                         : rect.size.height / rect.size.width;
// 长比宽
//         std::cout << "area:" << rect.size.area() << std::endl;
//         std::cout <<"ratio" <<ratio<<std::endl;
//         std::cout<<"wqwq"<<rect.size.area() / area <<std::endl;
        if ((rect.size.area() > rune_param.min_fan_area && rect.size.area() < rune_param.max_fan_area) &&
            rect.size.area() / area > rune_param.min_area_ratio
            &&  (ratio > rune_param.min_fan_ratio && ratio < rune_param.max_fan_ratio)
//this is key
                )
            //rect area&&contour area &&ratio
        {
            //this is null
//            cout<<"eee"<<endl;
//            imshow("EEE??",rune_debug);
            //todo:: 需要添加更多的条件  放入扇叶候选队列/
            // std::cout<<"qwqwqwq"<<std::endl;

            cout<<yukki_fan<<endl;
            yukki_fan++;
            fans.emplace_back(filled_contours[i], rect);//fans有时候不止一个，甚至可能有三个
            //get fan by"filled contours"
        } else
            continue;
    }
    //todo::ifdef只是个画图
#define DRAW
#ifdef DRAW
    for (const auto & fan : fans) {
        cv::Point2f vertices[4];
        fan.rrect.points(vertices);
        cv::putText(
                rune_debug, "fan_angle" + std::to_string(fan.rrect.angle), cv::Point2f(10, 30), 2, 2,
                cv::Scalar(189,240,120), 4);
        for (int i = 0; i < 4; i++) {
            line(
                    rune_debug, vertices[i], vertices[(i + 1) % 4],
                    cv::Scalar(189,240,120));  //四个角点连成线，最终形成旋转的矩形。
        }
    }
//   cv::imshow("debug1st", rune_debug);
#endif
//        imshow("ccc",contour_filled);
    return fans;
}
//fansizer 没改
//直到这里都还可以 顶多也就是少量误差





























bool RuneDetector::fanSizer(std::vector<rm_auto_aim::ShootFan> fans)
{



    //11111///寻找长边中心点（分类找长边）////
    for (auto & fan : fans) {
        cv::Point2f fan_pts[4];
        fan.rrect.points(fan_pts);
        if (cv::norm(fan_pts[0] - fan_pts[1]) > cv::norm(fan_pts[1] - fan_pts[2])) {
            // 0-1 is the long side
            sorted_pts = {fan_pts[0], fan_pts[1], fan_pts[2], fan_pts[3]};
        } else {
            // 1-2 is the long side
            sorted_pts = {fan_pts[1], fan_pts[2], fan_pts[3], fan_pts[0]};
        }
        //貌似只能保证顺时针？
        fan.long_side = sorted_pts[0] - sorted_pts[1];
        cv::Point2f longcenter1, longcenter2;
        longcenter1 = (sorted_pts[0] + sorted_pts[1]) / 2;
        longcenter2 = (sorted_pts[2] + sorted_pts[3]) / 2;
        fan.longside_centers.emplace_back(longcenter1);
        fan.longside_centers.emplace_back(longcenter2);



#define DRAW_CIRCLE
#ifdef DRAW_CIRCLE
//                    line(rune_debug,sorted_pts[0],sorted_pts[1],Scalar(255,255,0),3);
//                line(rune_debug,sorted_pts[2],sorted_pts[3],Scalar(255,255,0),3);

//        circle(rune_debug, sorted_pts[2], 7, Scalar(102,204,255));
        circle(rune_debug, sorted_pts[1], 7, Scalar(255));
        //感觉不准的是黄框框，因为那是外接矩形，是顶点的轨迹
        //但感觉并不影响。
        circle(rune_debug, longcenter1, 7, Scalar(102,204,255));
        circle(rune_debug, longcenter2, 7, Scalar(0,255,255));

        /// 长边中心画圈圈是要干啥。.......但这里识别还没问题!!!!!!!!!!!!!!sort的也暂时没有问题
        //////......................................?why find side center?
        // get！ goto ->409
#endif
    }




    auto getROI = [&](
            const std::vector<cv::Point> & roi_pts1,
            const std::vector<cv::Point> & roi_pts2) -> cv::Mat {
        cv::Mat mask = cv::Mat::zeros(filled_contour_img.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> vpts = {roi_pts1, roi_pts2};
        cv::fillPoly(mask, vpts, cv::Scalar(255));
        return filled_contour_img & mask;
        cout<<roi_pts1<<111<<endl<<roi_pts2<<222<<endl;

    };
    //BYD 在beta地方写了个函数

//    for (auto & fana : fans)
//    {
//        //cv::Mat flow_roi;
//flow_roi=cv::Mat(getROI())
//    }






//fans ->fana
    cv::Mat flow_roi;
    for (auto & fana : fans)//问题是 候选扇叶只有一个。。。。。。。。。。。。。。
        //这里是构思依托 建议改了？不知道跑的有没有问题，我的意见是直接flowptss不就行了 虽然rect可能不止一个。。。，但是，这种b写法也没啥优点，总比这坨好吧？
        //比起直接rect排序后锁定 有个坤八优点
    {
        std::vector<cv::Point> lights_roi1_pts1 = {
                fana.longside_centers[1] + fana.long_side / norm(fana.long_side) * 20,//WTF
                fana.longside_centers[1] + 100 * fana.long_side / norm(fana.long_side),
                fana.longside_centers[0] + 100 * fana.long_side / norm(fana.long_side),
                fana.longside_centers[0] + fana.long_side / norm(fana.long_side) * 20
        };
        //我的理解是 他太依赖老板代码了。
        std::vector<cv::Point> lights_roi1_pts2 = {
                fana.longside_centers[1] - fana.long_side / norm(fana.long_side) * 20,
                fana.longside_centers[1] - 100 * fana.long_side / norm(fana.long_side),
                fana.longside_centers[0] - 100 * fana.long_side / norm(fana.long_side),
                fana.longside_centers[0] - fana.long_side / norm(fana.long_side) * 20};

        ///////////寻找流水灯条////////////
        //实测结论：老绿框（step2后的识别没啥问题，黄框的识别有问题（Step3））
        circle(rune_debug, lights_roi1_pts1[1], 7, Scalar(0,0,255));

        circle(rune_debug, lights_roi1_pts1[2], 7, Scalar(0,0,255));
        //变形的有点严重我只能说。30-40环也就是极限了，太畸形了
        //直接重写吧 不演了
        flow_roi = cv::Mat(getROI(lights_roi1_pts1, lights_roi1_pts2));

        imshow("00010007",flow_roi);
        std::vector<std::vector<cv::Point>> flow_roi_contours;
        cv::findContours(flow_roi, flow_roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//我看只能find出个鸡毛来

        for (const auto & contour


                : flow_roi_contours) {

            double area = contourArea(contour);

            //是这个rect 也就是contour（刚刚find鸡毛出来的flowcontour的角点，自然是个鸡毛。）
            //...
            auto rect = cv::minAreaRect(contour);
            if (
                    rect.size.area() > rune_param.min_flow_area &&
                    rect.size.area() < rune_param.max_flow_area) {
                float ratio = rect.size.width > rect.size.height ? rect.size.width / rect.size.height
                                                                 : rect.size.height / rect.size.width;
                if (
                        rune_param.min_ratio < ratio && ratio < rune_param.max_ratio
                        //bug!!!!bug!!!!!!todo::bug!!!!!
                        &&rect.size.area() / area > rune_param.min_flow_area_ratio
                        ) {

                    cv::Point2f flow_pts[4];
                    rect.points(flow_pts);

                    if (cv::norm(flow_pts[0] - flow_pts[1]) > cv::norm(flow_pts[1] - flow_pts[2])) {
                        // 0-1 is the long side
                        sorted_pts = {flow_pts[0], flow_pts[1], flow_pts[2], flow_pts[3]};
                    } else {
                        // 1-2 is the long side
                        sorted_pts = {flow_pts[1], flow_pts[2], flow_pts[3], flow_pts[0]};
                    }
                    //sort 了个坤8 这个flowpts更是一坨屎
                    //dst框 理所应当的依托答辩
                    line(rune_debug, sorted_pts[0], sorted_pts[1], cv::Scalar(255,255,0), 5);

                    //0 is near the center
                    if (
                            cv::norm(sorted_pts[0] - fana.rrect.center) >
                            cv::norm(sorted_pts[1] - fana.rrect.center)) {
                        swap(sorted_pts[0], sorted_pts[1]);
                    }
                    if (
                            cv::norm(sorted_pts[2] - fana.rrect.center) >
                            cv::norm(sorted_pts[3] - fana.rrect.center)) {
                        swap(sorted_pts[2], sorted_pts[3]);
                    }

                    circle(rune_debug, sorted_pts[0], 7, cv::Scalar(255,255,0), 5);
                    //睾丸。
                    circle(rune_debug, sorted_pts[2], 7, cv::Scalar(255,204,102), 5);
                    fana.flow_far_from_center = (sorted_pts[0] + sorted_pts[2]) / 2;
                    fana.towards = (sorted_pts[1] - sorted_pts[0]) / norm(sorted_pts[0] - sorted_pts[1]);
                    ////////////////找到灯条角度/////////////
                    auto ang =
                            cv::fastAtan2(sorted_pts[0].y - sorted_pts[1].y, sorted_pts[0].x - sorted_pts[1].x);
                    fana.fan_angle = ang;
                    putText(
                            rune_debug, "long_angle shit 1" + std::to_string(ang), cv::Point2f(10, 80), 2, 2, cv::Scalar(135, 206, 235),
                            2);
                    cv::Point2f vertices[4];

                    //ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
                    fana.rrect.points(vertices);
                    //shittttttttttttttttttttttttttttttttttttttttt
                    //  ttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt

                    std::vector<cv::Point2f> roi_pts = {vertices, vertices + 4};
                    for(int i=0;i<4;i++){
                        line(rune_debug,vertices[i],vertices[(i+1)%4],Scalar(255,255,255),3);//四个角点连成线，最终形成旋转的矩形。
                    }

                    cv::Rect rect = cv::boundingRect(fana.fan_contours);
//                                          if (!makeRectSafe(rect, image_process_.src_.size())) continue;
//                     cv::Point2f coo = rect.tl();
                    cv::Mat roi = filled_contour_img(rect);
                    // 扇叶的最小外接矩形
                    cv::RotatedRect rrect = cv::minAreaRect(fana.fan_contours);
                    if (rrect.size.width > rrect.size.height) {
                        fan_angle = 90 + rrect.angle;
                        std::swap(rrect.size.width, rrect.size.height);
                    } else {
                        fan_angle = rrect.angle;
                    }
                    cv::Point2f roi_center = cv::Point2f(roi.cols / 2, roi.rows / 2);

                    // 旋转图形，使图片信息不丢失q
                    cv::Mat rot = getRotationMatrix2D(roi_center, ang - 90, 1);

                    cv::Mat rot_g(3, 3, CV_64F);
                    for (int i = 0; i < 2; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            rot_g.at<double>(i, j) = rot.at<double>(i, j);
                        }
                    }
                    rot_g.at<double>(2, 0) = 0;
                    rot_g.at<double>(2, 1) = 0;
                    rot_g.at<double>(2, 2) = 1;
                    invert(rot_g, rot_g);
                    cv::Rect2f bbox = cv::RotatedRect(roi_center, roi.size(), fan_angle).boundingRect2f();
                    rot.at<double>(0, 2) += bbox.width / 2.0 - roi.cols / 2.0;
                    rot.at<double>(1, 2) += bbox.height / 2.0 - roi.rows / 2.0;
                    cv::Mat rot_roi;
                    warpAffine(roi, rot_roi, rot, bbox.size());

                    // 扇叶中心旋转后的点
                    cv::Mat rrect_center_mat =
                            (cv::Mat_<double>(3, 1) << rrect.center.x - rect.tl().x, rrect.center.y - rect.tl().y,
                                    1);
                    cv::Mat rot_mat = rot * rrect_center_mat;
                    cv::Point2f rot_center = cv::Point2f(rot_mat.at<double>(0, 0), rot_mat.at<double>(1, 0));

                    //截取矫正的图形
                    cv::Mat dst;
                    getRectSubPix(rot_roi, rrect.size, rot_center, dst);
                    circle(dst, rot_center, 3, cv::Scalar(255, 255, 255), 2);





                    std::vector<cv::Point2f> armor_pts;
                    double start = dst.rows / 3;
                    double pixel_ratio = 0;
                    while (pixel_ratio < 0.3) {
                        pixel_ratio = PixelContour(dst, start++, 1) / (1 * dst.cols);
                    }
                    //?????????????????









                    cv::Point2f roi_target_center(dst.cols / 2, (start + dst.rows) / 2);
                    fana.fan_cols = dst.cols;
                    fana.fan_center = rrect.center;
                    //                        auto qwq=inverse_affine(rot_g,roi_target_center);
                    //                        auto awa=qwq+rrect.center;
                    //                        Point2f junction(rrect.center.x+cos(ang/180*CV_PI)*roi_target_center.y,rrect.center.y+cos(ang/180*CV_PI)*roi_target_center.y);
                    cv::Point2f target_center(dst.cols / 2, (start + dst.rows) / 2 - dst.rows / 2);

                    //                        Point2f point_4_left_top    (rrect.center.x+cos(ang/180*CV_PI)*dst.cols,rrect.center.y+sin(ang/180*CV_PI)*dst.rows);
                    //                        Point2f point_4_right_top   (rrect.center.x+cos(ang/180*CV_PI)*0,       rrect.center.y+sin(ang/180*CV_PI)*dst.rows);
                    //                        Point2f point_4_right_bottom(rrect.center.x+cos(ang/180*CV_PI)*0,       rrect.center.y+sin(ang/180*CV_PI)*start);
                    //                        Point2f point_4_left_bottom (rrect.center.x+cos(ang/180*CV_PI)*dst.cols,rrect.center.y+sin(ang/180*CV_PI)*start);
                    cv::Mat point_left_top = (cv::Mat_<double>(3, 1) << dst.cols, dst.rows, 1);
                    cv::Mat point_right_top = (cv::Mat_<double>(3, 1) << 0, dst.rows, 1);
                    cv::Mat point_right_bottom = (cv::Mat_<double>(3, 1) << 0, start, 1);
                    cv::Mat point_left_bottom = (cv::Mat_<double>(3, 1) << dst.cols, dst.rows, 1);
                    cv::Mat yuantu_point_left_top = rot_g * point_left_top;
                    cv::Mat yuantu_right_top = rot_g * point_right_top;
                    cv::Mat yuantu_right_bottom = rot_g * point_right_bottom;
                    cv::Mat yuantu_point_left_bottom = rot_g * point_left_bottom;
                    // cv::Point2f point_4_left_top = cv::Point2f(
                    //   yuantu_point_left_top.at<double>(0, 0), yuantu_point_left_top.at<double>(1, 0));
                    // cv::Point2f point_4_right_top =
                    //   cv::Point2f(yuantu_right_top.at<double>(0, 0), yuantu_right_top.at<double>(1, 0));
                    // cv::Point2f point_4_right_bottom =
                    //   cv::Point2f(yuantu_right_bottom.at<double>(0, 0), yuantu_right_bottom.at<double>(1, 0));
                    // cv::Point2f point_4_left_bottom = cv::Point2f(
                    //   yuantu_point_left_bottom.at<double>(0, 0), yuantu_point_left_bottom.at<double>(1, 0));
                    cv::Point2f junction(
                            rrect.center.x + cos(ang / 180 * CV_PI) * target_center.y,
                            rrect.center.y + sin(ang / 180 * CV_PI) * target_center.y);
                    ////

                    fana.target_center = junction;
                    cv::Mat debug = dst.clone();
                    cvtColor(dst, debug, cv::COLOR_GRAY2BGR);
                    circle(debug, cv::Point2f(dst.cols / 2, start), 2, cv::Scalar(0, 255, 0), 2);
                    circle(debug, roi_target_center, 6, cv::Scalar(0, 255, 0), 2);
                    circle(rune_debug, rrect.center, 10, cv::Scalar(38, 255, 255), 2);
                    circle(rune_debug, junction, 8, cv::Scalar(255, 0, 200), 2);
                    // circle(rune_debug, point_4_left_top + coo, 8, cv::Scalar(255, 0, 200), 2);
                    // circle(rune_debug, point_4_right_top + coo, 8, cv::Scalar(255, 0, 200), 2);
                    // circle(rune_debug, point_4_right_bottom + coo, 8, cv::Scalar(255, 0, 200), 2);
                    // circle(rune_debug, point_4_left_bottom + coo, 8, cv::Scalar(255, 0, 200), 2);
                    //                        circle(rune_debug,point_4_left_top,4,Scalar(255,0,200),2);
                    imshow("destination", debug);
                    imshow("rune_debug circle",rune_debug);
                    final_fan = fana;

                    return true;
                } else {
                    // RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "No flow found!");
                    continue;
                }
            } else {
                continue;
            }
        }

        imshow("flow_roi",flow_roi);
//        imwrite("flow_roi",flow_roi);
        imshow("rune_debug",rune_debug);
//        imwrite("rune_debug",rune_debug);
//        printf("1111111");
    }
    return false;
}

bool RuneDetector::findCenter(cv::Mat & src)
{
    // auto time_q = std::chrono::steady_clock::now();
//    cv::waitKey(4);
    if (imageProcess(src)) {//
        if (fanSizer(fillContour())) {
            cv::Mat mask = cv::Mat::zeros(binary_img.size(), CV_8UC1);
            //        int radius = cv::norm(final_fan.fan_cols) * 2;

            auto r_coord = final_fan.fan_center + final_fan.towards * final_fan.fan_cols * 1.3;
            cv::circle(mask, r_coord, 25, cv::Scalar(255), -1);
            cv::circle(rune_debug, r_coord, 25, cv::Scalar(255,255,255), 2);
            cv::Mat R_roi = binary_img.mul(mask);
            // Find the center
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(R_roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            // std::cout<<contours.size()<<std::endl;
            for (const auto & contour : contours) {
                cv::Rect rect = cv::boundingRect(contour);
                // std::cout<<"rect_area"<<rect.area()<<std::endl;
                // std::cout<<"min"<<rune_param.min_r_area<<std::endl;
                // std::cout<<"max"<<rune_param.max_r_area<<std::endl;
                if (rune_param.min_r_area < rect.area() && rect.area() < rune_param.max_r_area) {

                    float ratio = (float)rect.height / (float)rect.width;
#define SHOW_R_ROI
#define changeTo2f(x) std::to_string(int(x)) + "." + std::to_string(int(x * 100 + 0.5) % 100)

#ifdef SHOW_R_ROI
                    cv::putText(
                            R_roi, "a:" + changeTo2f(rect.area()), cv::Point2f(rect.br()) + cv::Point2f(0, 0),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);

                    cv::putText(
                            R_roi, "r:" + changeTo2f(ratio), cv::Point2f(rect.br()) + cv::Point2f(0, 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);

                    cv::imshow("R roi1", R_roi);
#endif
                    // std::cout<<ratio<<std::endl;
                    if (rune_param.min_r_ratio < ratio && ratio < rune_param.max_r_ratio) {
                        r_center = (rect.br() + rect.tl()) * 0.5;  //R标中心
                        std::cout<<r_center.x<<std::endl;
                        std::cout<<r_center.y<<std::endl;



                        final_fan.fan_angle =  cv::fastAtan2(final_fan.flow_far_from_center.y - r_center.y, final_fan.flow_far_from_center.x - r_center.x);
                        cv::circle(rune_debug, r_center, 3, cv::Scalar(251, 206, 235), 2);

#define SHOW_RUNE_CENTER
#ifdef SHOW_RUNE_CENTER
//                         circle(rune_detector_debug, foot_point, 1, cv::Scalar(255, 255, 255), 10);
                        imshow("R roi", R_roi);
                        imshow("rune detector debug", rune_detector_debug);
#endif
                        // auto time_cap = std::chrono::steady_clock::now();
                        // auto time1 = (std::chrono::duration<double, std::milli>(time_cap - time_q).count());
                        return true;
                    }
                }
            }
            return false;
        } else
            return false;
    }
    return false;
}




int main() {
    /*容器，存放分离通道后的图像*/
    Mat image;
    image= imread("/home/yukki/Static.png");
    vector<Mat> Channels;
    split(image, Channels);


    Mat redimage = Channels.at(2) - Channels.at(0);
    /*二值化*/
    Mat binaryImage;
    threshold(redimage, binaryImage, 140, 255, THRESH_BINARY);
    /*蓝色*
    Mat blueImage = Channels.at(0) - Channels.at(2);
    /*找到圆周运动的圆心——R*/
    vector<vector<Point>> outlines;
    vector<Vec4i> hierarchies;
    int minArea = 10000;
    int minId;
    Point2f center;  /*定义外接圆中心坐标*/
    float radius;  /*定义外接圆半径*/
    findContours(binaryImage, outlines, hierarchies, RETR_TREE, CHAIN_APPROX_NONE);
    for (int i = 0; i < outlines.size(); i++) {
        vector<Point>points;
        double area = contourArea(outlines[i]);
        /*面积排除噪声*/
        if (area < 10 || area>10000)
            continue;
        /*找到没有父轮廓的轮廓*/
        if (hierarchies[i][3] >= 0 && hierarchies[i][3] < outlines.size())
            continue;
        /*找有子轮廓的*/
        if (hierarchies[i][2] < 0 || hierarchies[i][2] >= outlines.size())
            continue;
        /*控制误差范围*/
        if (area <= minArea + 10 && area >= minArea - 20) {
            minArea = area;
            minId = i;
            continue;
        }
        /*面积最小的轮廓*/
        if (minArea >= area)
        {
            minArea = area;
            minId = i;
        }
    }

    /*防止minId不在范围内报错*/
    Mat test=binaryImage.clone();
    if (minId >= 0 && minId < outlines.size()) {
        /*画外接圆并找到圆心*/
        minEnclosingCircle(Mat(outlines[minId]), center, radius);
        circle(test, center, radius, Scalar(0, 0, 255), 1, 8, 0);
    }
    else {
        //退出
    }
    /*膨胀操作*/
    Mat element = getStructuringElement(0, Size(3, 3));
    Mat dilateImage;
    /*dilate最后一个数字是膨胀次数*/
    dilate(binaryImage, dilateImage, element, Point(-1, -1), 2);
    /*轮廓发现*/
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    double maxArea = -1;
    int maxId;
    findContours(dilateImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
    for (int i = 0; i < contours.size(); i++) {
        vector<Point>points;
        double area = contourArea(contours[i]);
        /*面积排除噪声*/
        if (area < 20 || area>10000)
            continue;
        /*找到没有父轮廓的轮廓*/
        if (hierarchy[i][3] >= 0 && hierarchy[i][3] < contours.size())
            continue;
        /*找没子轮廓的*/
        if (hierarchy[i][2] >= 0 && hierarchy[i][2] < contours.size())
            continue;
        /*找面积最大的轮廓*/
        if (maxArea <= area)
        {
            maxArea = area;
            maxId = i;
        }
        /*控制误差范围*/
        if (area <= maxArea + 50 && area >= maxArea - 50) {
            maxArea = area;
            maxId = i;
        }
    }
    if (maxId >= 0 && maxId < contours.size()) {
        /*画出需打部位轮廓*/
        drawContours(test, contours, maxId, Scalar(0, 255, 255), 1, 8);
    }
    Point2f rectMid;/*半径参考长度所在轮廓几何中心*/

    if (maxId >= 0 && maxId < contours.size()) {
        /*计算矩*/
        Moments rect;
        rect = moments(contours[maxId], false);
        /*计算中心矩:*/
        Point2f rectmid;
        rectmid = Point2f(rect.m10 / rect.m00, rect.m01 / rect.m00);
        /*画出需打部位轮廓*/
        drawContours(test, contours, maxId, Scalar(0, 255, 255), 1, 8);
        rectMid = rectmid;
    }

    /*长度2：1计算需打击部位,存放*/

    Point2f target;/*目标点*/
    double multiple = 1.5;/*倍率，换算目标点所用*/
//一个不错的思路，可惜target 在r标（挺准。）
    /*第一象限*/
    if (rectMid.x >= center.x && rectMid.y <= center.y) {
        target = Point2f(center.x + (rectMid.x - center.x) * multiple, center.y - (center.y - rectMid.y) * multiple);

    }
    /*第二象限*/
    if (rectMid.x <= center.x && rectMid.y <= center.y) {
        target = Point2f(center.x - (center.x - rectMid.x) * multiple, center.y - (center.y - rectMid.y) * multiple);

    }
    /*第三象限*/
    if (rectMid.x <= center.x && rectMid.y >= center.y) {
        target = Point2f(center.x - (center.x - rectMid.x) * multiple, center.y + (rectMid.y - center.y) * multiple);

    }
    /*第四象限*/
    if (rectMid.x >= center.x && rectMid.y >= center.y) {
        target = Point2f(center.x + (rectMid.x - center.x) * multiple, center.y + (rectMid.y - center.y) * multiple);

    }
    circle(test, target, 10, Scalar(255), -1, 8, 0);

    imshow("image", test);
    waitKey();
    return 0;
}
















































/**
VideoCapture imread("/home/yuuki/Downloads/RedMove.mp4");
Mat frame;
for (;;) {
Rect point_array[20];
imread >> frame;
if (frame.empty()) {
break;
}
Mat gray_img, thresh_img;
//灰度
cvtColor(frame, gray_img, COLOR_BGR2GRAY);
threshold(gray_img, thresh_img, 0, 255, THRESH_TRIANGLE);
//开运算
Mat ellipse = getStructuringElement(MORPH_ELLIPSE, Size(13, 13));
morphologyEx(thresh_img, thresh_img, MORPH_OPEN, ellipse, Point(-1, -1), 2);
//寻找轮廓
vector<vector<Point>> contours;
vector<Vec4i> hierarchy1;
findContours(thresh_img, contours, hierarchy1, RETR_LIST, CHAIN_APPROX_NONE, Point());
//获取某一轮廓重心点
Moments M;
M = moments(contours[0]);
double cX = double(M.m10 / M.m00);
double cY = double(M.m01 / M.m00);
//绘制轮廓
drawContours(frame, contours, 0, Scalar(0, 255, 0), 2, 8, hierarchy1);
//显示轮廓重心并提取坐标点
circle(frame, Point2d(cX, cY), 6, Scalar(0, 255, 0), 2, 8);
namedWindow("Center Point", WINDOW_NORMAL);
imshow("Center Point", frame);
//imwrite("D:\\Besktop\\1\\22_21_27.bmp", img);
putText(frame, "center", Point2d(cX - 20, cY - 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1, 8);
cout << "重心坐标：" << cX << " " << cY << endl << endl;
if (waitKey(50) >= 0) {
break;
}

 // 假设 RuneDetector 类的简化版本
class RuneDetector {
public:
    // 假设 imageProcess 函数接收一个图像对象引用，并进行处理
    void imageProcess(Image& src) {
        // 这里是对图像src进行处理的代码
        // 例如：灰度化、二值化、特征提取等
        processImage(src);
    }

private:
    // 这里是内部用于图像处理的辅助函数
    void processImage(Image& img) {
        // 实现你的图像处理算法
    }
};

// 使用示例
int main() {
    // 创建一个Image对象
    Image src = loadImage("input.jpg");

    // 创建RuneDetector对象
    RuneDetector detector;

    // 对图像进行处理
    detector.imageProcess(src);

    return 0;
}
















 }**/