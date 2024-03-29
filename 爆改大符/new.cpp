#include<iostream>
#include<opencv2/opencv.hpp>
//#include "1111.h"

#include "HaiKangCamera.h"
#include "serial.h"
#define show_debug
//#define OPEN_SERIAL
int debug=1;
#include <chrono>
#include <thread>
using namespace std;
using namespace std::chrono;
SendData send_data;
SerialPort SP;
#include<iostream>
#include<iostream>
#include<opencv2/opencv.hpp>
#include "1111.h"
//std::vector<rm_auto_aim::ShootFan> fans;
//#define filename "/home/yukki/LIT BLUE ONE.MP4";
#define filename "/home/yukki/LIT FULL.mp4";
//#define DRAW
#define delay 150
//fan angle是基于rrect angle的，是一个X轴的0-90的
//int debug=1;
//#define rune_color self_BLUE
//没用pnp
//Mar16--junction和roicenter真的有关系吗
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
        binary_img = channels.at(0);//???
        //
//        binary_img=channels.at(0);
    //单通道近似灰度图
    imshow("通道二值化",binary_img);
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
    }//大小合适的轮廓画出来，并塞满??

//这里也没啥好改的 虽然有点多余

    std::vector<ShootFan> fans;

    ////膨胀轮廓使其连续
    //    dilate(filled_contour_img,filled_contour_img,kernel7);
    morphologyEx(filled_contour_img, filled_contour_img, cv::MORPH_CLOSE, kernel3);
//去除细小空洞，感觉没啥用。准备做掉！！！！！！！
    if(debug)
        imshow("what the fuck you find",filled_contour_img);
    //画完的黑底


    //第二次筛轮廓，不知道为啥要这么繁琐的筛，先看着
    findContours(filled_contour_img, filled_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < filled_contours.size(); i++) {
        double area = contourArea(filled_contours[i]);

        drawContours(rune_debug, filled_contours, static_cast<int>(i), cv::Scalar(0, 255, 255), 2);

        //金黄色外围轮廓，r标到底要不要（我觉得不要）
        //第二次仍然是挺准的

        auto rect = cv::minAreaRect(filled_contours[i]);
        //this is good

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
    //true 靠画出来的
#define DRAW
#ifdef DRAW

    for (const auto & fan : fans) {
        cv::Point2f vertices[4];
        fan.rrect.points(vertices);

        cv::putText(
                rune_debug, "fan_angle" + std::to_string(fan.rrect.angle), cv::Point2f(10+yukki_fan*5, 30+yukki_fan*2), 2, 2,
                cv::Scalar(189,240,120), 4);
        //  你说你这能不重叠吗,本来就不止一个fan

        for (int i = 0; i < 4; i++) {
            line(
                    rune_debug, vertices[i], vertices[(i + 1) % 4],
                    cv::Scalar(189,240,170));  //四个角点连成线，最终形成旋转的矩形。
        }
    }

   cv::imshow("debug1st", rune_debug);
#endif
//        imshow("ccc",contour_filled);
    return fans;
}
//fansizer 没改
//直到这里都还可以 顶多也就是少量误差



























#define f1
#ifdef f1
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
        cout<<cv::norm(fan_pts[0] - fan_pts[1])<<cv::norm(fan_pts[1] - fan_pts[2])<<"norm"<<111<<endl;
        //116.372   220.411norm111//接近1：2吧，01短12长没啥好说的？,这个排序可能还行
        //貌似只能保证顺时针？
        fan.long_side = sorted_pts[0] - sorted_pts[1];
        cv::Point2f longcenter1, longcenter2;
        longcenter1 = (sorted_pts[0] + sorted_pts[1]) / 2;
        //01,23的这个应该是没问题
        longcenter2 = (sorted_pts[2] + sorted_pts[3]) / 2;

        fan.longside_centers.emplace_back(longcenter1);
        fan.longside_centers.emplace_back(longcenter2);


#define DRAW_CIRCLE
#ifdef DRAW_CIRCLE
//                    line(rune_debug,sorted_pts[0],sorted_pts[1],Scalar(255,255,0),3);
//                line(rune_debug,sorted_pts[2],sorted_pts[3],Scalar(255,255,0),3);

//        circle(rune_debug, sorted_pts[2], 7, Scalar(102,204,255));
        circle(rune_debug, sorted_pts[1], 3, Scalar(255));
        //感觉不准的是黄框框，因为那是外接矩形，是顶点的轨迹
        //但感觉并不影响。
        circle(rune_debug, longcenter1, 3, Scalar(102,204,255));
        circle(rune_debug, longcenter2, 3, Scalar(0,255,255));

        /// 长边中心画圈圈是要干啥。.......但这里识别还没问题!!!!!!!!!!!!!!sort的也暂时没有问题
        //////......................................?why find side center?
        // get！ goto ->409?
#endif
    }









    auto getROI = [&](
            const std::vector<cv::Point> & roi_pts1,
            const std::vector<cv::Point> & roi_pts2) -> cv::Mat {
        cv::Mat mask = cv::Mat::zeros(filled_contour_img.size(), CV_8UC1);

        std::vector<std::vector<cv::Point>> vpts = {roi_pts1, roi_pts2};
        cv::fillPoly(mask, vpts, cv::Scalar(255,123,123));
        return filled_contour_img & mask;
        cout<<roi_pts1<<111<<endl<<roi_pts2<<222<<endl;
        //一刀切出flowroi？？mar13

    };
    //BYD 在beta地方写了个函数

//    for (auto & fana : fans)
//    {
//        //cv::Mat flow_roi;
//flow_roi=cv::Mat(getROI())
//    }

//light is agnoring


//可以修正的地方还是有很多。MAR9
//Rcenter怎么也有点屎

//fans ->fana
    cv::Mat flow_roi;
    for (auto & fana : fans)//问题是 候选扇叶只有一个。。。。。。。。。。。。。。
        //这里是构思依托 建议改了？不知道跑的有没有问题，我的意见是直接flowptss不就行了 虽然rect可能不止一个。。。，但是，这种b写法也没啥优点，总比这坨好吧？
        //比起直接rect排序后锁定 有个坤八优点
    {
        std::vector<cv::Point> lights_roi1_pts1 = {
                fana.longside_centers[1] + fana.long_side / norm(fana.long_side) * 20,//延长对角线画出roi,
                fana.longside_centers[1] + 100 * fana.long_side / norm(fana.long_side),//首先是lonsidecenter要准，然后可以稍微给大点？
                fana.longside_centers[0] + 100 * fana.long_side / norm(fana.long_side),//b视频里不准的原因大概是lightroi会吧偏然后roi跟着偏了
                fana.longside_centers[0] + fana.long_side / norm(fana.long_side) * 20
        };
        //这里乱的大概率原因是lightroipts是用norm算出来的，所以很死，视频得换
        cout<<"shit "<<fana.long_side / norm(fana.long_side) * 20<<endl;
        //我的理解是 他太依赖老板代码了。
        //自己画的一个fanaroi pts，但是没做排序，搞鸡毛
        std::vector<cv::Point> lights_roi1_pts2 = {
                fana.longside_centers[1] - fana.long_side / norm(fana.long_side) * 20,
                fana.longside_centers[1] - 100 * fana.long_side / norm(fana.long_side),

                fana.longside_centers[0] - 100 * fana.long_side / norm(fana.long_side),
                fana.longside_centers[0] - fana.long_side / norm(fana.long_side) * 20};

        ///////////寻找流水灯条////////////
        //实测结论：老绿框（step2后的识别没啥问题，黄框的识别有问题（Step3））
        circle(rune_debug, lights_roi1_pts1[1], 9, Scalar(0,255,255),-1);
        circle(rune_debug, lights_roi1_pts2[1], 19, Scalar(200,200,255));
        //按理说是一个放大一点点的roi？但是吧，light_roi_pts选取的简直就是狗屎
        // biger than dkck
        //MAR6 突然一切问题都解决了。。。。。。。。。。。。。。。。。。。
//其实注释的挺清楚的，light_roi_pts
//话说这个画的是啥，怪，还是很怪
        circle(rune_debug, lights_roi1_pts1[2], 9, Scalar(0,255,255),-1);
        circle(rune_debug, lights_roi1_pts2[2], 13, Scalar(200,200,255));
        //顺序不能确定（一般外圈，有时候内圈，排序的问题应该是视频的问题，要理清楚排序逻辑）
        //但是lightroi pts可能不一定需要排序正确》
        //在底下的时候甚至没有白圈

//        变形的有点严重我只能说。30-40环也就是极限了，太畸形了
//        直接重写吧 不演了
        flow_roi = cv::Mat(getROI(lights_roi1_pts1, lights_roi1_pts2));
        //是砍断两个中心的意思？？
        //good idea to get flow
//为什么0010007有时候海星有时候一坨屎呢，因为排序做的不行？//是吗//不是------------MAR5
//if(debug)
//        imshow("00010007",flow_roi);
        std::vector<std::vector<cv::Point>> flow_roi_contours;

        cv::findContours(flow_roi, flow_roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<std::vector<cv::Point>> flow_roi_contours1=flow_roi_contours;
//我看只能find出个鸡毛来//xx
//暂时当flow_roi没问题往下做，实际上符合距离的话可能会没问题吧，确实有那么几帧是没问题的

//test
#define test_mar
#ifdef test_mar
        {
            vector<Vec4i> hierarchy_test;
            cv::findContours(flow_roi, flow_roi_contours1, hierarchy_test, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//我看只能find出个鸡毛来//xxx
//暂时当flow_roi没问题往下做，实际上符合距离的话可能会没问题吧，确实有那么几帧是没问题的
            int index = 0;
            Mat debug_mar20 = flow_roi;
            cv::Mat debug_mar = cv::Mat::zeros(binary_img.size(), CV_8UC1);
            for (; index >= 0; index = hierarchy_test[index][0]) {
                Scalar color(rand() & 255, rand() & 255, rand() & 255);

                drawContours(debug_mar, flow_roi_contours1, index, cv::Scalar(255, 255, 255), 8);
            }
//            circle(debug_mar, lights_roi1_pts2[1], 9, Scalar(0, 255, 255));
            circle(debug_mar, lights_roi1_pts1[0], 13, Scalar(123, 255, 255));
            if (debug)
                imshow("00010007", debug_mar);
            //鉴定完毕，flowroi虽然还是存在几个误识别，但真的海星

//
        }
#endif
        for (const auto & contour: flow_roi_contours) {
//flow roi contours is important-----------------MAR4
            double area = contourArea(contour);

            //是这个rect 也就是contour（刚刚find鸡毛出来的flowcontour的角点，自然是个鸡毛。）
            //...
            auto rect = cv::minAreaRect(contour);
            if (
                    rect.size.area() > rune_param.min_flow_area &&
                    rect.size.area() < rune_param.max_flow_area) {
                float ratio = rect.size.width > rect.size.height ? rect.size.width / rect.size.height
                                                                 : rect.size.height / rect.size.width;
                //写的第二个ratio了吧
                if (
                        rune_param.min_ratio < ratio && ratio < rune_param.max_ratio
                        //bug!!!!bug!!!!!!todo::bug!!!!!
                        //what bug？
                        &&rect.size.area() / area > rune_param.min_flow_area_ratio
                        )
                    //多了一个b筛选，然后画出第一步找好的矩形开凿
                {

                    cv::Point2f flow_pts[4];
                    //shit pts
                    rect.points(flow_pts);
                    //this by my
                    for (int i = 0; i < 4; i++) {
                        line(
                                rune_debug, flow_pts[i], flow_pts[(i + 1) % 4],
                                cv::Scalar(255,0,255),6);  //四个角点连成线，最终形成旋转的矩形。//就是流水灯条呗。
                    }
//那我在这里再整一个筛选另一个的，不就是要的了？

                    if (cv::norm(flow_pts[0] - flow_pts[1]) > cv::norm(flow_pts[1] - flow_pts[2])) {
                        // 0-1 is the long side
                        sorted_pts = {flow_pts[0], flow_pts[1], flow_pts[2], flow_pts[3]};
                    } else {
                        // 1-2 is the long side
                        sorted_pts = {flow_pts[1], flow_pts[2], flow_pts[3], flow_pts[0]};
                    }
                    //sort 了个坤8 这个flowpts更是一坨屎
                    //dst框 理所应当的依托答辩、、nop
                    line(rune_debug, sorted_pts[0], sorted_pts[1], cv::Scalar(255,255,123), 5);//坤坤
                    circle(rune_debug, sorted_pts[1], 5, cv::Scalar(255,255,123), 5);
                    circle(rune_debug, sorted_pts[0], 5, cv::Scalar(255,255,123), 5);
//                    by yuki-mar5，貌似偶尔1near center捏。
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

                    circle(rune_debug, sorted_pts[0], 3, cv::Scalar(255,255,123), 5);
                    //睾丸。
                    //貌似0并不是很near center
                    /**
                    circle(rune_debug, sorted_pts[1], 7, cv::Scalar(255,204,102), 5);
                    circle(rune_debug, sorted_pts[3], 7, cv::Scalar(255,204,102), 5);
                     所以这个sorted只是为了求个角度而已吗？
                    **/
                    circle(rune_debug, sorted_pts[2], 3, cv::Scalar(255,204,102), 5);
                    fana.flow_far_from_center = (sorted_pts[0] + sorted_pts[2]) / 2;
                    fana.towards = (sorted_pts[1] - sorted_pts[0]) / norm(sorted_pts[0] - sorted_pts[1]);
                    cout<<fana.towards<<111<<endl;
                    ////////////////找到灯条角度/////////////
                    auto ang =
                            cv::fastAtan2(sorted_pts[0].y - sorted_pts[1].y, sorted_pts[0].x - sorted_pts[1].x);
                    fana.fan_angle = ang;
                    int i = 0;//解决重叠问题
                    putText(
                            rune_debug, "long_angle shit 1" + std::to_string(ang), cv::Point2f(10+i, 80+i), 2, 2, cv::Scalar(135, 206, 235),
                            2);
                    //不在
                    cv::Point2f vertices[4];
                    if(debug)
                        imshow("sb",rune_debug);//这里是灯条的框框
                    //ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
                    fana.rrect.points(vertices);//这里是之前填进去的fana吧，在这里又画一遍同样的东西，但？

#ifdef my_roi


#endif
                    //shittttttttttttttttttttttttttttttttttttttttt
                    //  ttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt

                    std::vector<cv::Point2f> roi_pts = {vertices, vertices + 4};
                    for(int i=0;i<4;i++){
                        line(rune_debug,vertices[i],vertices[(i+1)%4],Scalar(255,255,255),3);//四个角点连成线，最终形成旋转的矩形。
                    }

                    cv::Rect rect = cv::boundingRect(fana.fan_contours);
                    //and this rect&&bounding rect??
//                                          if (!makeRectSafe(rect, image_process_.src_.size())) continue;
//                     cv::Point2f coo = rect.tl();
                    cv::Mat roi = filled_contour_img(rect);
//                    cv::rectangle(rune_debug, rect.tl(),rect.br(), cv::Scalar(0, 0, 255),-1);
                    //就是那样，把画一个黑底的rect？filled是这样的//MAR12但是这玩意是个正矩形啊，不转的，为啥

int wid=rect.width;
int heg=rect.height;
auto tl=(rect.tl().x,rect.tl().y);
                  auto  imageROI = rune_debug(Rect(rect.tl().x,rect.tl().y,wid+10,10+heg));
                    imshow("rooo",imageROI);
#ifdef my_roi

#endif
                    // 扇叶的最小外接矩形
                    cv::RotatedRect rrect = cv::minAreaRect(fana.fan_contours);
//                    cv::Point2f vertdebug[4];
//                    rrect.points(vertdebug);
//                    for (int i = 0; i < 4; i++) {
//                        line(rune_debug, vertdebug[i], vertdebug[(i + 1) % 4], cv::Scalar(255, 255, 0), 8);
//                    }

                    if (rrect.size.width > rrect.size.height) {
                        fan_angle = 90 + rrect.angle;
                        std::swap(rrect.size.width, rrect.size.height);
                    } else {
                        fan_angle = rrect.angle;
                    }
                    cv::Point2f roi_center = cv::Point2f(roi.cols / 2, roi.rows / 2);
                    //还是觉得是那个rect的中心，但这个是boundingRect的中心，真不如旋转矩形的比较准的中心吧？？？MAR12
                    // 旋转图形，使图片信息不丢失q//偶尔还是会丢失捏-MAR11
                    cv::Mat rot = getRotationMatrix2D(roi_center, ang - 90, 1);

//90？？？？？？？？？？？？？？？
                    cv::Mat rot_g(3, 3, CV_64F);
                    for (int i = 0; i < 2; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            rot_g.at<double>(i, j) = rot.at<double>(i, j);
                        }
                    }
                    //why is pixel   --------------------------------Mar4
                    //know about it.mar10
                    rot_g.at<double>(2, 0) = 0;
                    rot_g.at<double>(2, 1) = 0;
                    rot_g.at<double>(2, 2) = 1;
                    //可能或许是旋转矩阵那个公式？又不太像、、90°的话应该就是了吧。
                    invert(rot_g, rot_g);
                    //这几行到底是什么shit

                    cv::Rect2f bbox = cv::RotatedRect(roi_center, roi.size(), fan_angle).boundingRect2f();
                    rot.at<double>(0, 2) += bbox.width / 2.0 - roi.cols / 2.0;//(get the center of rotation/warpaffine
                    //屎。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。MAR5

                    rot.at<double>(1, 2) += bbox.height / 2.0 - roi.rows / 2.0;
                    cv::Mat rot_roi;
#define debug_roi 0
                    if(debug_roi)
                        imshow("before_roi",roi);// 直到这里为止几乎是完美的。
                    warpAffine(roi, rot_roi, rot, bbox.size());
                    if(debug_roi)
                        imshow("warp_roi",rot_roi); //还怪大的，又好又准，后面对他做了啥。

                    // 扇叶中心旋转后的点

                    cv::Mat rrect_center_mat =
                            (cv::Mat_<double>(3, 1) << rrect.center.x - rect.tl().x, rrect.center.y - rect.tl().y,
                                    1);//big fan ==rrect     //rrect-rect,rect=rrect时=0，就不转了呗（001）
                    //这个是第二鬼畜的。
                    cv::Mat rot_mat = rot * rrect_center_mat;


//              realize about it from cappp
                    cv::Point2f rot_center = cv::Point2f(rot_mat.at<double>(0, 0), rot_mat.at<double>(1, 0));
                    //this is shit of allllllllllllllllll//非得乘一个rot
                    //截取矫正的图形
                    cv::Mat dst;
                    getRectSubPix(rot_roi, rrect.size, rot_center, dst);
                    //the key of all！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！1


                    circle(dst, rot_center, 3, cv::Scalar(255, 255, 255), -1);
//这个是那个目前比较飘的白点，貌似最终锁定的圆心是那个墨绿色的，但这个白点是啥，乱飘
                    imshow("dst_orgin", dst);



                    std::vector<cv::Point2f> armor_pts;
                    double start = dst.rows / 3;
//                    double pixel_ratio = 0;
//                    while (pixel_ratio < 0.3) {
//                        pixel_ratio = PixelContour(dst, start++, 1) / (1 * dst.cols);
//                    }
                    //????????????????
                    //鉴定为老版的shit没删完//我的妈这玩意居然对程序有影响吗









                    cv::Point2f roi_target_center(dst.cols / 2, (start + dst.rows) / 2);
                    //真是抽象.........,真得换一个方式计算吧，至少roi_target_center得新写一版

                    fana.fan_cols = dst.cols;
                    fana.fan_center = rrect.center;
                    //                        auto qwq=inverse_affine(rot_g,roi_target_center);
                    //                        auto awa=qwq+rrect.center;
                    //                        Point2f junction(rrect.center.x+cos(ang/180*CV_PI)*roi_target_center.y,rrect.center.y+cos(ang/180*CV_PI)*roi_target_center.y);
//                    cv::Point2f target_center(dst.cols / 2, start/2);//屎中屎 +完-
//                    cv::Point2f target_center(dst.cols / 2, (start + dst.rows) / 2 - dst.rows / 2);
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
                            //1:targetcenter,2"junction...shi
                            rrect.center.x
                            + cos(ang / 180 * CV_PI) * target_center.y//rad to radian
                            ,
                            rrect.center.y
                            + sin(ang / 180 * CV_PI) * target_center.y
                    );
                    //原来那两个并不是小圈和大圈，而是两坨屎

                    ////
                    //直接干三角函数，这不是小符吗
                    fana.target_center = junction;
                    cv::Mat debug = dst.clone();
                    cvtColor(dst, debug, cv::COLOR_GRAY2BGR);
                    circle(debug, cv::Point2f(dst.cols / 2, start)//干啥的这个点。。。没懂、M16
                            , 2, cv::Scalar(0, 255, 0), 2);

                    circle(debug, roi_target_center, 3, cv::Scalar(11, 123, 0), 2);
                    //屏蔽掉某pixel后出来的固定位置大绿圈，并没有参与后续计算，不知道干嘛的//难道他真是圆心？
                    //so why findcenter.wtf、、

                    circle(rune_debug, rrect.center, 3, cv::Scalar(118, 255, 255), 2);
                    //单纯的rrect.center画回去

//                    circle(rune_debug, junction, 8, cv::Scalar(255, 0, 200), 2);//圆心！！！！！！
                    circle(rune_debug, junction, 3, cv::Scalar(255, 255, 255), -1);
                    //这玩意就是最终的击打点了
//                     circle(rune_debug, point_4_left_top + coo, 8, cv::Scalar(255, 0, 200), 2);
//                     circle(rune_debug, point_4_right_top + coo, 8, cv::Scalar(255, 0, 200), 2);
//                     circle(rune_debug, point_4_right_bottom + coo, 8, cv::Scalar(255, 0, 200), 2);
//                     circle(rune_debug, point_4_left_bottom + coo, 8, cv::Scalar(255, 0, 200), 2);
//                                            circle(rune_debug,point_4_left_top,4,Scalar(255,0,200),2);
                    imshow("destination", debug);
                    //偶尔挺准，不知道为啥）
                    imshow("rune_debug circle",rune_debug);
                    //到此为止，下面两个imshow没有出来
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

        imshow("flow_roi111",flow_roi);
//        imwrite("flow_roi",flow_roi);
        if(debug)
            imshow("rune_debug——fin",rune_debug);
//        imwrite("rune_debug",rune_debug);
//        printf("1111111");
    }
    return false;
}
#endif
//#define fan2
#ifdef fan2
bool RuneDetector::fanSizer(std::vector<rm_auto_aim::ShootFan> fans) {
    //11111///寻找长边中心点（分类找长边）////
    for (auto &fan: fans) {
        cv::Point2f fan_pts[4];
        fan.rrect.points(fan_pts);
        if (cv::norm(fan_pts[0] - fan_pts[1]) > cv::norm(fan_pts[1] - fan_pts[2])) {
            // 0-1 is the long side
            sorted_pts = {fan_pts[0], fan_pts[1], fan_pts[2], fan_pts[3]};
        } else {
            // 1-2 is the long side
            sorted_pts = {fan_pts[1], fan_pts[2], fan_pts[3], fan_pts[0]};
        }
        cout << cv::norm(fan_pts[0] - fan_pts[1]) << cv::norm(fan_pts[1] - fan_pts[2]) << "norm" << 111 << endl;
        //116.372   220.411norm111//接近1：2吧，01短12长没啥好说的？,这个排序可能还行
        //貌似只能保证顺时针？
        fan.long_side = sorted_pts[0] - sorted_pts[1];
        cv::Point2f longcenter1, longcenter2;
        longcenter1 = (sorted_pts[0] + sorted_pts[1]) / 2;
        //01,23的这个应该是没问题
        longcenter2 = (sorted_pts[2] + sorted_pts[3]) / 2;

        fan.longside_centers.emplace_back(longcenter1);
        fan.longside_centers.emplace_back(longcenter2);


#define DRAW_CIRCLE
#ifdef DRAW_CIRCLE
//                    line(rune_debug,sorted_pts[0],sorted_pts[1],Scalar(255,255,0),3);
//                line(rune_debug,sorted_pts[2],sorted_pts[3],Scalar(255,255,0),3);

//        circle(rune_debug, sorted_pts[2], 7, Scalar(102,204,255));
        circle(rune_debug, sorted_pts[1], 3, Scalar(255));
        //感觉不准的是黄框框，因为那是外接矩形，是顶点的轨迹
        //但感觉并不影响。
        circle(rune_debug, longcenter1, 3, Scalar(102, 204, 255));
        circle(rune_debug, longcenter2, 3, Scalar(0, 255, 255));

        /// 长边中心画圈圈是要干啥。.......但这里识别还没问题!!!!!!!!!!!!!!sort的也暂时没有问题
        //////......................................?why find side center?
        // get！ goto ->409?
#endif
    }
    auto getROI = [&](
            const std::vector<cv::Point> &roi_pts1,
            const std::vector<cv::Point> &roi_pts2) -> cv::Mat {
        cv::Mat mask = cv::Mat::zeros(filled_contour_img.size(), CV_8UC1);

        std::vector<std::vector<cv::Point>> vpts = {roi_pts1, roi_pts2};
        //1234&&5678
        //完美的切割，天才的构思，至少切出来算角度够用
        cv::fillPoly(mask, vpts, cv::Scalar(255, 123, 123));
        return filled_contour_img & mask;
        cout << roi_pts1 << 111 << endl << roi_pts2 << 222 << endl;
        //一刀切出flowroi？？mar13

    };
    //BYD 在beta地方写了个函数

//    for (auto & fana : fans)
//    {
//        //cv::Mat flow_roi;
//flow_roi=cv::Mat(getROI())
//    }


//可以修正的地方还是有很多。MAR9
//Rcenter怎么也有点屎

//fans ->fana
    cv::Mat flow_roi;
    for (auto &fana: fans)//问题是 候选扇叶只有一个。。。。。。。。。。。。。。
        //这里是构思依托 建议改了？不知道跑的有没有问题，我的意见是直接flowptss不就行了 虽然rect可能不止一个。。。，但是，这种b写法也没啥优点，总比这坨好吧？
        //比起直接rect排序后锁定 有个坤八优点
    {
        std::vector<cv::Point> lights_roi1_pts1 = {
                fana.longside_centers[1] + fana.long_side / norm(fana.long_side) * 20,//延长对角线画出roi,
                fana.longside_centers[1] + 100 * fana.long_side / norm(fana.long_side),//首先是lonsidecenter要准，然后可以稍微给大点？
                fana.longside_centers[0] +
                100 * fana.long_side / norm(fana.long_side),//b视频里不准的原因大概是lightroi会吧偏然后roi跟着偏了
                fana.longside_centers[0] + fana.long_side / norm(fana.long_side) * 20
        };
        //norm为正的时候pts1就在上面，因为是+norm
        //这里乱的大概率原因是lightroipts是用norm算出来的，所以很死，视频得换//xxxxxxxxxxxxx--mar23

        //按理来说是个延长（到原图也是延长）但是在roi里迷之就是几个对应点，很神奇//mar23
        //好吧他居然是个向量..毕竟是vector 啊//mar23
        cout << "shit " << fana.long_side / norm(fana.long_side) * 20 << endl;
        //我的理解是 他太依赖老板代码了。
        //自己画的一个fanaroi pts，但是没做排序，搞鸡毛
        std::vector<cv::Point> lights_roi1_pts2 = {
                fana.longside_centers[1] - fana.long_side / norm(fana.long_side) * 20,
                fana.longside_centers[1] - 100 * fana.long_side / norm(fana.long_side),

                fana.longside_centers[0] - 100 * fana.long_side / norm(fana.long_side),
                fana.longside_centers[0] - fana.long_side / norm(fana.long_side) * 20};

        ///////////寻找流水灯条////////////
        //实测结论：老绿框（step2后的识别没啥问题，黄框的识别有问题（Step3））

//        circle(rune_debug, lights_roi1_pts1[1], 9, Scalar(0,255,255),-1);
//        circle(rune_debug, lights_roi1_pts1[2], 9, Scalar(0,255,255),-1);
//        circle(rune_debug, lights_roi1_pts1[0], 9, Scalar(0,0,255),-1);
//        circle(rune_debug, lights_roi1_pts1[3], 9, Scalar(0,0,255),-1);

        //话说point为什么会有01............
        //按理说是一个放大一点点的roi？但是吧，light_roi_pts选取的简直就是狗屎
        //MAR6 突然一切问题都解决了。。。。。。。。。。。。。。。。。。。
//其实注释的挺清楚的，light_roi_pts
//话说这个画的是啥，怪，还是很怪
        circle(rune_debug, lights_roi1_pts2[1], 2, Scalar(20, 200, 255));
        circle(rune_debug, lights_roi1_pts2[2], 2, Scalar(20, 200, 255));
        circle(rune_debug, lights_roi1_pts2[0], 2, Scalar(20, 200, 255));
        circle(rune_debug, lights_roi1_pts2[3], 2, Scalar(20, 200, 255));
        //顺序不能确定（一般外圈，有时候内圈，排序的问题应该是视频的问题，要理清楚排序逻辑）
        //但是lightroi pts可能不一定需要排序正确》
        //在底下的时候甚至没有白圈

//        变形的有点严重我只能说。30-40环也就是极限了，太畸形了
//        直接重写吧 不演了
        flow_roi = cv::Mat(getROI(lights_roi1_pts1, lights_roi1_pts2));
        //是砍断两个中心的意思？？
        //good idea to get flow
//为什么0010007有时候海星有时候一坨屎呢，因为排序做的不行？//是吗//不是------------MAR5
//if(debug)
//        imshow("00010007",flow_roi);
        std::vector<std::vector<cv::Point>> flow_roi_contours;

        cv::findContours(flow_roi, flow_roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<std::vector<cv::Point>> flow_roi_contours1 = flow_roi_contours;
//我看只能find出个鸡毛来//xx
//暂时当flow_roi没问题往下做，实际上符合距离的话可能会没问题吧，确实有那么几帧是没问题的

//test
//#define test_mar
#ifdef test_mar
        {
            vector<Vec4i> hierarchy_test;
            cv::findContours(flow_roi, flow_roi_contours1, hierarchy_test, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//我看只能find出个鸡毛来//xxx
//暂时当flow_roi没问题往下做，实际上符合距离的话可能会没问题吧，确实有那么几帧是没问题的
            int index = 0;
            Mat debug_mar20 = flow_roi;
            cv::Mat debug_mar = cv::Mat::zeros(binary_img.size(), CV_8UC1);
            for (; index >= 0; index = hierarchy_test[index][0]) {
                Scalar color(rand() & 255, rand() & 255, rand() & 255);

                drawContours(debug_mar, flow_roi_contours1, index, cv::Scalar(255, 255, 255), 3);
            }
//            circle(debug_mar, lights_roi1_pts2[1], 9, Scalar(0, 255, 255));
            circle(debug_mar, lights_roi1_pts1[0], 13, Scalar(123, 255, 255));
            circle(debug_mar, lights_roi1_pts2[0], 13, Scalar(123, 255, 255));
            circle(debug_mar, lights_roi1_pts1[1], 19, Scalar(0, 255, 255));
            circle(debug_mar, lights_roi1_pts2[1], 19, Scalar(255, 255, 255));
//why just3
            if (debug)
#ifdef show_debug
                imshow("00010007", debug_mar);
            //鉴定完毕，flowroi虽然还是存在几个误识别，但真的海星
#endif
//
        }
#endif
        for (const auto &contour: flow_roi_contours) {
//flow roi contours is important-----------------MAR4
            double area = contourArea(contour);

            //是这个rect 也就是contour（刚刚find鸡毛出来的flowcontour的角点，自然是个鸡毛。）
            //...
            auto rect = cv::minAreaRect(contour);
#define text_flowroi

#ifdef text_flowroi

            cv::Mat rune_debug1 = cv::Mat::zeros(filled_contour_img.size(), CV_8UC1);
            for (const auto &contour: flow_roi_contours) {


                //是这个rect 也就是contour（刚刚find鸡毛出来的flowcontour的角点，自然是个鸡毛。）
                //...
                drawContours(rune_debug1, flow_roi_contours, -1, cv::Scalar(255, 255, 255), 3);
                auto rect = cv::minAreaRect(contour);
                circle(rune_debug1, rect.center, 3, cv::Scalar(255, 255, 255), -1);
                circle(rune_debug, rect.center, 2, cv::Scalar(55, 255, 255), 3);
                imshow("atgext",rune_debug);
                cv::Point2f flow_pts[4];
                //shit pts
                rect.points(flow_pts);
                //this by my
                for (int i = 0; i < 4; i++) {
                    line(
                            rune_debug1, flow_pts[i], flow_pts[(i + 1) % 4],
                            cv::Scalar(255, 0, 255), 6);  //四个角点连成线，最终形成旋转的矩形。//就是流水灯条呗。
                }
                cout << "sb" << sizeof(contour) << "sb" << endl;
//                drawContours(rune_debug1, contour, -1, cv::Scalar(255, 255, 255), 3);


            }
#endif
            imshow("textflow", rune_debug1);

            if (
                    rect.size.area() > rune_param.min_flow_area &&
                    rect.size.area() < rune_param.max_flow_area) {


                float ratio = rect.size.width > rect.size.height ? rect.size.width / rect.size.height
                                                                 : rect.size.height / rect.size.width;
                //写的第二个ratio了吧
                if (
                        rune_param.min_ratio < ratio && ratio < rune_param.max_ratio
                        //bug!!!!bug!!!!!!todo::bug!!!!!
                        //what bug？
                        && rect.size.area() / area > rune_param.min_flow_area_ratio
                        )
                    //多了一个b筛选，然后画出第一步找好的矩形开凿

                {

                    cv::Point2f flow_pts[4];
                    //shit pts
                    rect.points(flow_pts);
                    //this by my
                    for (int i = 0; i < 4; i++) {
                        line(
                                rune_debug, flow_pts[i], flow_pts[(i + 1) % 4],
                                cv::Scalar(255, 0, 255), 6);  //四个角点连成线，最终形成旋转的矩形。//就是流水灯条呗。
                    }
//那我在这里再整一个筛选另一个的，不就是要的了？

                    if (cv::norm(flow_pts[0] - flow_pts[1]) > cv::norm(flow_pts[1] - flow_pts[2])) {
                        // 0-1 is the long side
                        sorted_pts = {flow_pts[0], flow_pts[1], flow_pts[2], flow_pts[3]};
                    } else {
                        // 1-2 is the long side
                        sorted_pts = {flow_pts[1], flow_pts[2], flow_pts[3], flow_pts[0]};
                    }
                    //sort 了个坤8 这个flowpts更是一坨屎
                    //dst框 理所应当的依托答辩、、nop
                    line(rune_debug, sorted_pts[0], sorted_pts[1], cv::Scalar(255, 255, 123), 5);//坤坤
                    circle(rune_debug, sorted_pts[1], 2, cv::Scalar(255, 255, 123), 5);
                    circle(rune_debug, sorted_pts[0], 2, cv::Scalar(255, 255, 123), 5);
//                    by yuki-mar5，貌似偶尔1near center捏。
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

                    circle(rune_debug, sorted_pts[0], 2, cv::Scalar(255, 255, 123), 5);
                    //睾丸。
                    //貌似0并不是很near center
                    /**
                    circle(rune_debug, sorted_pts[1], 7, cv::Scalar(255,204,102), 5);
                    circle(rune_debug, sorted_pts[3], 7, cv::Scalar(255,204,102), 5);
                     所以这个sorted只是为了求个角度而已吗？
                    **/
                    circle(rune_debug, sorted_pts[2], 2, cv::Scalar(255, 204, 102), 5);
                    fana.flow_far_from_center = (sorted_pts[0] + sorted_pts[2]) / 2;
                    fana.towards = (sorted_pts[1] - sorted_pts[0]) / norm(sorted_pts[0] - sorted_pts[1]);
                    cout << fana.towards << 111 << endl;
                    ////////////////找到灯条角度/////////////
                    auto ang =
                            cv::fastAtan2(sorted_pts[0].y - sorted_pts[1].y, sorted_pts[0].x - sorted_pts[1].x);
                    fana.fan_angle = ang;
                    int i = 0;//解决重叠问题
                    putText(
                            rune_debug, "long_angle shit 1" + std::to_string(ang), cv::Point2f(10 + i, 80 + i), 2, 2,
                            cv::Scalar(135, 206, 235),
                            2);
                    //不在
                    cv::Point2f vertices[4];

//                    if (debug)
#ifdef show_debug
                    imshow("sb", rune_debug);//这里
#endif// 是灯条的框框
                    //ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
                    fana.rrect.points(vertices);//这里是之前填进去的fana吧，在这里又画一遍同样的东西，但？
                    //我建议什么呢，新建一个fana.flow.rect，拿那玩意比例去计算
                    //shittttttttttttttttttttttttttttttttttttttttt
                    //  ttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt

                    std::vector<cv::Point2f> roi_pts = {vertices, vertices + 4};
                    for (int i = 0; i < 4; i++) {
                        line(rune_debug, vertices[i], vertices[(i + 1) % 4], Scalar(255, 255, 255),
                             3);//四个角点连成线，最终形成旋转的矩形。
                    }

                    cv::Rect rect = cv::boundingRect(fana.fan_contours);
//                                          if (!makeRectSafe(rect, image_process_.src_.size())) continue;
//                     cv::Point2f coo = rect.tl();
                    cv::Mat roi = filled_contour_img(rect);
//                    cv::rectangle(rune_debug, rect.tl(),rect.br(), cv::Scalar(0, 0, 255),-1);
                    //就是那样，把画一个黑底的rect？filled是这样的//MAR12但是这玩意是个正矩形啊，不转的，为啥


                    // 扇叶的最小外接矩形
                    cv::RotatedRect rrect = cv::minAreaRect(fana.fan_contours);
//                    cv::Point2f vertdebug[4];
//                    rrect.points(vertdebug);
//                    for (int i = 0; i < 4; i++) {
//                        line(rune_debug, vertdebug[i], vertdebug[(i + 1) % 4], cv::Scalar(255, 255, 0), 8);
//                    }

                    if (rrect.size.width > rrect.size.height) {
                        fan_angle = 90 + rrect.angle;
                        std::swap(rrect.size.width, rrect.size.height);
                    } else {
                        fan_angle = rrect.angle;
                        //first fan angle
                    }

                    cv::Point2f roi_center = cv::Point2f(roi.cols / 2, roi.rows / 2);
                    //还是觉得是那个rect的中心，但这个是boundingRect的中心，真不如旋转矩形的比较准的中心吧？？？MAR12
                    // 旋转图形，使图片信息不丢失q//偶尔还是会丢失捏-MAR11
                    cv::Mat rot = getRotationMatrix2D(roi_center, ang - 90, 1);

//90？？？？？？？？？？？？？？？
                    cv::Mat rot_g(3, 3, CV_64F);
                    for (int i = 0; i < 2; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            rot_g.at<double>(i, j) = rot.at<double>(i, j);
                        }
                    }
                    //why is pixel   --------------------------------Mar4
                    //know about it.mar10
                    rot_g.at<double>(2, 0) = 0;
                    rot_g.at<double>(2, 1) = 0;
                    rot_g.at<double>(2, 2) = 1;
                    //可能或许是旋转矩阵那个公式？又不太像、、90°的话应该就是了吧。
                    invert(rot_g, rot_g);
                    //这几行到底是什么shit

                    cv::Rect2f bbox = cv::RotatedRect(roi_center, roi.size(), fan_angle).boundingRect2f();
                    //导致部分裁剪的万恶之源


                    rot.at<double>(0, 2) += bbox.width / 2.0 - roi.cols / 2.0;//(get the center of rotation/warpaffine
                    //屎。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。MAR5

                    rot.at<double>(1, 2) += bbox.height / 2.0 - roi.rows / 2.0;
                    cv::Mat rot_roi;
#define debug_roi 0
                    if (debug_roi)
                        imshow("before_roi", roi);// 直到这里为止几乎是完美的。
                    warpAffine(roi, rot_roi, rot, bbox.size());
                    if (debug_roi)
                        imshow("warp_roi", rot_roi); //还怪大的，又好又准，后面对他做了啥。

                    // 扇叶中心旋转后的点

                    cv::Mat rrect_center_mat =
                            (cv::Mat_<double>(3, 1) << rrect.center.x - rect.tl().x, rrect.center.y - rect.tl().y,
                                    1);//big fan ==rrect     //rrect-rect,rect=rrect时=0，就不转了呗（001）
                    //这个是第二鬼畜的。
                    cv::Mat rot_mat = rot * rrect_center_mat;


//              realize about it from cappp
                    cv::Point2f rot_center = cv::Point2f(rot_mat.at<double>(0, 0), rot_mat.at<double>(1, 0));
                    //this is shit of allllllllllllllllll//非得乘一个rot
                    //截取矫正的图形
                    cv::Mat dst;
                    getRectSubPix(rot_roi, rrect.size, rot_center, dst);
                    //the key of all！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！1


                    circle(dst, rot_center, 5, cv::Scalar(255, 255, 255), -1);
//这个是那个目前比较飘的白点，貌似最终锁定的圆心是那个墨绿色的，但这个白点是啥，乱飘
#ifdef show_debug
                    imshow("dst_orgin", dst);
#endif

                    std::vector<cv::Point2f> armor_pts;
                    double start = dst.rows / 3;
//                    double pixel_ratio = 0;
//                    while (pixel_ratio < 0.3) {
//                        pixel_ratio = PixelContour(dst, start++, 1) / (1 * dst.cols);
//                    }
                    //????????????????
                    //鉴定为老版的shit没删完//我的妈这玩意居然对程序有影响吗









                    cv::Point2f roi_target_center(dst.cols / 2, (start + dst.rows) / 2);
                    //话说targetcenter不影响junction啊。///mar21
                    //真是抽象.........,真得换一个方式计算吧，至少roi_target_center得新写一版

                    fana.fan_cols = dst.cols;
                    fana.fan_center = rrect.center;
                    //                        auto qwq=inverse_affine(rot_g,roi_target_center);
                    //                        auto awa=qwq+rrect.center;
                    //                        Point2f junction(rrect.center.x+cos(ang/180*CV_PI)*roi_target_center.y,rrect.center.y+cos(ang/180*CV_PI)*roi_target_center.y);
//                    cv::Point2f target_center(dst.cols / 2, start/2);//屎中屎 +完-
                    cv::Point2f target_center(dst.cols / 2, (start + dst.rows) / 2 - dst.rows / 2);




//                    cv::Point2f target_center(dst.cols / 2,(start + dst.rows)/2);
                    //貌似是第二次？？roi-tar..
//                    cv::Point2f target_center(dst.cols / 2, (start + dst.rows) / 2 - dst.rows / 2);

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

                    //一滩狗屎。。。


                    cv::Point2f junction(
                            //1:targetcenter,2"junction...shi
                            rrect.center.x
                            + cos(ang / 180 * CV_PI) * target_center.y//rad to radian
                            ,
                            rrect.center.y
                            + sin(ang / 180 * CV_PI) * target_center.y
                    );
                    //原来那两个并不是小圈和大圈，而是两坨屎

                    ////
                    //直接干三角函数，这不是小符吗
                    fana.target_center = junction;
                    cv::Mat debug = dst.clone();
                    cvtColor(dst, debug, cv::COLOR_GRAY2BGR);
                    circle(debug, cv::Point2f(dst.cols / 2, start)//干啥的这个点。。。没懂、M16
                            //有·像junction
                            , 2, cv::Scalar(0, 255, 0), 2);

                    circle(debug, roi_target_center, 2, cv::Scalar(11, 123, 0), 2);
                    //屏蔽掉某pixel后出来的固定位置大绿圈，并没有参与后续计算，不知道干嘛的//难道他真是圆心？、、貌似不影响圆心捏
                    //roi_target_center按理来说要和target——center完全一致才能有debug效果
                    //so why findcenter.wtf、、

                    circle(rune_debug, rrect.center, 2, cv::Scalar(118, 255, 255), 2);
                    //单纯的rrect.center画回去

#define text_moment1
#ifdef text_moment1
                    auto text_moment = final_fan.fan_center;
                    circle(rune_debug, text_moment, 2, cv::Scalar(255, 204, 102), -1);
#endif
                    //
//                    circle(rune_debug, junction, 8, cv::Scalar(255, 0, 200), 2);//圆心！！！！！！
                    circle(rune_debug, junction, 2, cv::Scalar(255, 255, 255), -1);
                    //这玩意就是最终的击打点了
//                     circle(rune_debug, point_4_left_top + coo, 8, cv::Scalar(255, 0, 200), 2);
//                     circle(rune_debug, point_4_right_top + coo, 8, cv::Scalar(255, 0, 200), 2);
//                     circle(rune_debug, point_4_right_bottom + coo, 8, cv::Scalar(255, 0, 200), 2);
//                     circle(rune_debug, point_4_left_bottom + coo, 8, cv::Scalar(255, 0, 200), 2);
//                                            circle(rune_debug,point_4_left_top,4,Scalar(255,0,200),2);
#ifdef show_debug
                    imshow("destination", debug);
                    //偶尔挺准，不知道为啥）
                    imshow("rune_debug circle", rune_debug);
                    //到此为止，下面两个imshow没有出来
#endif
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


#ifdef show_shit
        imshow("flow_roi111",flow_roi);
//        imwrite("flow_roi",flow_roi);
if(debug)
        imshow("rune_debug——fin",rune_debug);
//        imwrite("rune_debug",rune_debug);
//        printf("1111111");
    }
#endif
    }
    return false;
}


#endif







bool RuneDetector::findCenter(cv::Mat & src)
{
    // auto time_q = std::chrono::steady_clock::now();
//    cv::waitKey(4);
    if (imageProcess(src)) {//一口气全跑完，按理来说只要一个findcenter
        if (fanSizer(fillContour())) {
            cv::Mat mask = cv::Mat::zeros(binary_img.size(), CV_8UC1);
            //黑底
            //        int radius = cv::norm(final_fan.fan_cols) * 2;

            auto r_coord = final_fan.fan_center + final_fan.towards * final_fan.fan_cols * 1.3;

            //another key



            cv::circle(mask, r_coord, 3, cv::Scalar(255), -1);
            //?
            cv::circle(rune_debug, r_coord, 3, cv::Scalar(255,255,255), 2);
            //这三坨到底是啥.........MAR6
            cv::Mat R_roi = binary_img.mul(mask);
            //R标从大变小再消失，狗屎//所以为什么这玩意不写在上面。。
            // Find the center
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(R_roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            // std::cout<<contours.size()<<std::endl;
            for (const auto & contour : contours) {
                //anohter rect
                cv::Rect rect = cv::boundingRect(contour);
//                rectangle(rune_debug, rect, cv::Scalar(255, 255, 0), -1);
//                std::vector<std::vector<cv::Point>> vpts1 = {rect.tl(), rect.br()};
//                cv::fillPoly(mask, vpts1, cv::Scalar(255,123,123));
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
                    imshow("lalala",rune_debug);
                    cv::imshow("R roi1", R_roi);
#endif
                    // std::cout<<ratio<<std::endl;
                    if (rune_param.min_r_ratio < ratio && ratio < rune_param.max_r_ratio) {
                        r_center = (rect.br() + rect.tl()) * 0.5;  //R标中心
                        std::cout<<r_center.x<<std::endl;
                        std::cout<<r_center.y<<std::endl;


//好吧这一版已经是用找的比较准的灯条来算的角度了//所以为什么要算两次角度（MAR15）
                        final_fan.fan_angle =  cv::fastAtan2(final_fan.flow_far_from_center.y - r_center.y, final_fan.flow_far_from_center.x - r_center.x);
                        cv::circle(rune_debug, r_center, 3, cv::Scalar(251, 206, 235), 2);

#define SHOW_RUNE_CENTER
#ifdef SHOW_RUNE_CENTER
//                         circle(rune_detector_debug, foot_point, 1, cv::Scalar(255, 255, 255), 10);
                        imshow("R roi", R_roi);
//                        line(rune_debug,r_center,final_fan.fan_center, cv::Scalar(255, 0, 0), 1);

//                    imshow("test..",rune_debug);
//             imshow("rune detector debug", rune_detector_debug);
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
//const static cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//const static cv::Mat kernel5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
//const static cv::Mat kernel7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
//const static cv::Mat kernel11 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));
//rm_auto_aim::RuneDetector::RuneDetector(const RuneParam & r) :
//        rune_param(r)
//{
//}
//double RuneDetector::PixelContour(cv::Mat & rot, double start, int height)
//{
//    double sum = 0;
//    for (int i = start; i <     start + height; i++) {
//        auto * data = rot.ptr<uchar>(i);
//        for (int j = 0; j < rot.cols; j++) {
//            if (data[j] == 255) {
//                sum += 1;
//            }
//        }
//    }
//    return sum;
//}
//
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
//bool RuneDetector::imageProcess(cv::Mat & src)
//{
//    if (src.empty())
//        return false;
//    std::vector<cv::Mat> channels;
//    rune_debug = src.clone();
//
//    //debug 1st
//
//    cv::split(src, channels);
////实践得出结论：亮度高时均接近白光，反向通道反而可以滤掉圆心，有一定b用
//    if (rune_color == self_BLUE)
//        binary_img = channels.at(2);
//    else
//        binary_img = channels.at(1);
//    //单通道近似灰度图
////    imshow("通道二值化",binary_img);
//    threshold(binary_img, binary_img, rune_param.binary_threshold, 255, cv::THRESH_BINARY);
//
//    morphologyEx(binary_img, binary_img, cv::MORPH_DILATE, kernel5);
////    imshow("膨胀",binary_img);
//    return true;
//}
////img process 没啥好写的
//
//
//
//
////预处理->扇叶候选队列
//std::vector<rm_auto_aim::ShootFan> rm_auto_aim::RuneDetector::fillContour()
//{
//    int yukki_fan=0;
//    ////创建轮廓填充图 填充轮廓
//    filled_contour_img = cv::Mat::zeros(cv::Size(rune_debug.cols, rune_debug.rows), CV_8UC1);
//    // 原图大小的黑底
//    //永远只有黑白，包括draw
//    std::vector<std::vector<cv::Point>> contours;
//
//
//    ////寻找主要轮廓并填充（把填充的轮廓画在黑底上）
//    /// 需要调整area大小
//    findContours(binary_img, filled_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//    //find contours all in before
//    for (size_t i = 0; i < filled_contours.size(); i++) {
//        double area = contourArea(filled_contours[i]);
//        if (area < rune_param.min_filledContours_area) continue;
//        //把不符合条件的（小）轮廓给过滤掉
//        drawContours(
//                filled_contour_img, filled_contours, static_cast<int>(i), cv::Scalar(189,240,120),
//                cv::FILLED);
//    }//大小合适的轮廓画出来，并塞满??
//
////这里也没啥好改的 虽然有点多余
//
//    std::vector<ShootFan> fans;
//
//    ////膨胀轮廓使其连续
//    //    dilate(filled_contour_img,filled_contour_img,kernel7);
//    morphologyEx(filled_contour_img, filled_contour_img, cv::MORPH_CLOSE, kernel3);
////去除细小空洞，感觉没啥用。准备做掉！！！！！！！
//    if(debug)
//        imshow("what the fuck you find",filled_contour_img);
//    //画完的黑底
//
//
//    //第二次筛轮廓，不知道为啥要这么繁琐的筛，先看着
//    findContours(filled_contour_img, filled_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//    for (size_t i = 0; i < filled_contours.size(); i++) {
//        double area = contourArea(filled_contours[i]);
//
//        drawContours(rune_debug, filled_contours, static_cast<int>(i), cv::Scalar(0, 255, 255), 2);
//
//        //金黄色外围轮廓，r标到底要不要（我觉得不要）
//        //第二次仍然是挺准的
//
//        auto rect = cv::minAreaRect(filled_contours[i]);
//        //this is good
//
//        //RRECT rect
//        // std::cout<<area<<std::endl;
//        //?
//        if (area < rune_param.min_contourArea || area > rune_param.max_contourArea) continue;
//        //大小
//
//        float ratio = rect.size.width > rect.size.height ? rect.size.width / rect.size.height
//                                                         : rect.size.height / rect.size.width;
//// 长比宽
////         std::cout << "area:" << rect.size.area() << std::endl;
////         std::cout <<"ratio" <<ratio<<std::endl;
////         std::cout<<"wqwq"<<rect.size.area() / area <<std::endl;
//        if ((rect.size.area() > rune_param.min_fan_area && rect.size.area() < rune_param.max_fan_area) &&
//            rect.size.area() / area > rune_param.min_area_ratio
//            &&  (ratio > rune_param.min_fan_ratio && ratio < rune_param.max_fan_ratio)
////this is key
//                )
//            //rect area&&contour area &&ratio
//        {
//            //this is null
////            cout<<"eee"<<endl;
////            imshow("EEE??",rune_debug);
//            //todo:: 需要添加更多的条件  放入扇叶候选队列/
//            // std::cout<<"qwqwqwq"<<std::endl;
//
//            cout<<yukki_fan<<endl;
//            yukki_fan++;
//            fans.emplace_back(filled_contours[i], rect);//fans有时候不止一个，甚至可能有三个
//            //get fan by"filled contours"
//        } else
//            continue;
//    }
//    //todo::ifdef只是个画图
//    //true 靠画出来的
//#define DRAW
//#ifdef DRAW
//
//    for (const auto & fan : fans) {
//        cv::Point2f vertices[4];
//        fan.rrect.points(vertices);
//
//        cv::putText(
//                rune_debug, "fan_angle" + std::to_string(fan.rrect.angle), cv::Point2f(10+yukki_fan*5, 30+yukki_fan*2), 2, 2,
//                cv::Scalar(189,240,120), 4);
//        //  你说你这能不重叠吗,本来就不止一个fan
//
//        for (int i = 0; i < 4; i++) {
//            line(
//                    rune_debug, vertices[i], vertices[(i + 1) % 4],
//                    cv::Scalar(189,240,170));  //四个角点连成线，最终形成旋转的矩形。
//        }
//    }
//
////   cv::imshow("debug1st", rune_debug);
//#endif
////        imshow("ccc",contour_filled);
//    return fans;
//}
////fansizer 没改
////直到这里都还可以 顶多也就是少量误差
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//bool RuneDetector::fanSizer(std::vector<rm_auto_aim::ShootFan> fans)
//{
//
//
//
//    //11111///寻找长边中心点（分类找长边）////
//    for (auto & fan : fans) {
//        cv::Point2f fan_pts[4];
//        fan.rrect.points(fan_pts);
//        if (cv::norm(fan_pts[0] - fan_pts[1]) > cv::norm(fan_pts[1] - fan_pts[2])) {
//            // 0-1 is the long side
//            sorted_pts = {fan_pts[0], fan_pts[1], fan_pts[2], fan_pts[3]};
//        } else {
//            // 1-2 is the long side
//            sorted_pts = {fan_pts[1], fan_pts[2], fan_pts[3], fan_pts[0]};
//        }
//        cout<<cv::norm(fan_pts[0] - fan_pts[1])<<cv::norm(fan_pts[1] - fan_pts[2])<<"norm"<<111<<endl;
//        //116.372   220.411norm111//接近1：2吧，01短12长没啥好说的？,这个排序可能还行
//        //貌似只能保证顺时针？
//        fan.long_side = sorted_pts[0] - sorted_pts[1];
//        cv::Point2f longcenter1, longcenter2;
//        longcenter1 = (sorted_pts[0] + sorted_pts[1]) / 2;
//        //01,23的这个应该是没问题
//        longcenter2 = (sorted_pts[2] + sorted_pts[3]) / 2;
//
//        fan.longside_centers.emplace_back(longcenter1);
//        fan.longside_centers.emplace_back(longcenter2);
//
//
//#define DRAW_CIRCLE
//#ifdef DRAW_CIRCLE
////                    line(rune_debug,sorted_pts[0],sorted_pts[1],Scalar(255,255,0),3);
////                line(rune_debug,sorted_pts[2],sorted_pts[3],Scalar(255,255,0),3);
//
////        circle(rune_debug, sorted_pts[2], 7, Scalar(102,204,255));
//        circle(rune_debug, sorted_pts[1], 7, Scalar(255));
//        //感觉不准的是黄框框，因为那是外接矩形，是顶点的轨迹
//        //但感觉并不影响。
//        circle(rune_debug, longcenter1, 7, Scalar(102,204,255));
//        circle(rune_debug, longcenter2, 7, Scalar(0,255,255));
//
//        /// 长边中心画圈圈是要干啥。.......但这里识别还没问题!!!!!!!!!!!!!!sort的也暂时没有问题
//        //////......................................?why find side center?
//        // get！ goto ->409?
//#endif
//    }
//
//
//
//
//
//
//
//
//
//    auto getROI = [&](
//            const std::vector<cv::Point> & roi_pts1,
//            const std::vector<cv::Point> & roi_pts2) -> cv::Mat {
//        cv::Mat mask = cv::Mat::zeros(filled_contour_img.size(), CV_8UC1);
//
//        std::vector<std::vector<cv::Point>> vpts = {roi_pts1, roi_pts2};
//        cv::fillPoly(mask, vpts, cv::Scalar(255,123,123));
//        return filled_contour_img & mask;
//        cout<<roi_pts1<<111<<endl<<roi_pts2<<222<<endl;
//
//    };
//    //BYD 在beta地方写了个函数
//
////    for (auto & fana : fans)
////    {
////        //cv::Mat flow_roi;
////flow_roi=cv::Mat(getROI())
////    }
//
//
//
//
//
//
////fans ->fana
//    cv::Mat flow_roi;
//    for (auto & fana : fans)//问题是 候选扇叶只有一个。。。。。。。。。。。。。。
//        //这里是构思依托 建议改了？不知道跑的有没有问题，我的意见是直接flowptss不就行了 虽然rect可能不止一个。。。，但是，这种b写法也没啥优点，总比这坨好吧？
//        //比起直接rect排序后锁定 有个坤八优点
//    {
//        std::vector<cv::Point> lights_roi1_pts1 = {
//                fana.longside_centers[1] + fana.long_side / norm(fana.long_side) * 20,//延长对角线画出roi,
//                fana.longside_centers[1] + 100 * fana.long_side / norm(fana.long_side),//首先是lonsidecenter要准，然后可以稍微给大点？
//                fana.longside_centers[0] + 100 * fana.long_side / norm(fana.long_side),//b视频里不准的原因大概是lightroi会吧偏然后roi跟着偏了
//                fana.longside_centers[0] + fana.long_side / norm(fana.long_side) * 20
//        };
//        //这里乱的大概率原因是lightroipts是用norm算出来的，所以很死，视频得换
//        cout<<"shit "<<fana.long_side / norm(fana.long_side) * 20<<endl;
//        //我的理解是 他太依赖老板代码了。
//        //自己画的一个fanaroi pts，但是没做排序，搞鸡毛
//        std::vector<cv::Point> lights_roi1_pts2 = {
//                fana.longside_centers[1] - fana.long_side / norm(fana.long_side) * 20,
//                fana.longside_centers[1] - 100 * fana.long_side / norm(fana.long_side),
//                fana.longside_centers[0] - 100 * fana.long_side / norm(fana.long_side),
//                fana.longside_centers[0] - fana.long_side / norm(fana.long_side) * 20};
//
//        ///////////寻找流水灯条////////////
//        //实测结论：老绿框（step2后的识别没啥问题，黄框的识别有问题（Step3））
//        circle(rune_debug, lights_roi1_pts1[1], 9, Scalar(0,255,255));
//        //按理说是一个放大一点点的roi？但是吧，light_roi_pts选取的简直就是狗屎
//        //MAR6 突然一切问题都解决了。。。。。。。。。。。。。。。。。。。
////其实注释的挺清楚的，light_roi_pts
////话说这个画的是啥，怪，还是很怪
//        circle(rune_debug, lights_roi1_pts1[2], 11, Scalar(123,255,255));
//        //顺序不能确定（一般外圈，有时候内圈，排序的问题应该是视频的问题，要理清楚排序逻辑）
//        //但是lightroi pts可能不一定需要排序正确》
//        //在底下的时候甚至没有白圈
//
////        变形的有点严重我只能说。30-40环也就是极限了，太畸形了
////        直接重写吧 不演了
//        flow_roi = cv::Mat(getROI(lights_roi1_pts1, lights_roi1_pts2));
////为什么0010007有时候海星有时候一坨屎呢，因为排序做的不行？//是吗//不是------------MAR5
//        if(debug)
//            imshow("00010007",flow_roi);
//        std::vector<std::vector<cv::Point>> flow_roi_contours;
//
//
//
//
//
//        cv::findContours(flow_roi, flow_roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
////我看只能find出个鸡毛来
////暂时当flow_roi没问题往下做，实际上符合距离的话可能会没问题吧，确实有那么几帧是没问题的
//
//        for (const auto & contour: flow_roi_contours) {
////flow roi contours is important-----------------MAR4
//            double area = contourArea(contour);
//
//            //是这个rect 也就是contour（刚刚find鸡毛出来的flowcontour的角点，自然是个鸡毛。）
//            //...
//            auto rect = cv::minAreaRect(contour);
//            if (
//                    rect.size.area() > rune_param.min_flow_area &&
//                    rect.size.area() < rune_param.max_flow_area) {
//                float ratio = rect.size.width > rect.size.height ? rect.size.width / rect.size.height
//                                                                 : rect.size.height / rect.size.width;
//                //写的第二个ratio了吧
//                if (
//                        rune_param.min_ratio < ratio && ratio < rune_param.max_ratio
//                        //bug!!!!bug!!!!!!todo::bug!!!!!
//                        //what bug？
//                        &&rect.size.area() / area > rune_param.min_flow_area_ratio
//                        ) {
//
//                    cv::Point2f flow_pts[4];
//                    //shit pts
//                    rect.points(flow_pts);
//                    //this by my
//                    for (int i = 0; i < 4; i++) {
//                        line(
//                                rune_debug, flow_pts[i], flow_pts[(i + 1) % 4],
//                                cv::Scalar(123,2,255),10);  //四个角点连成线，最终形成旋转的矩形。//就是流水灯条呗。
//                    }
//
//
//                    if (cv::norm(flow_pts[0] - flow_pts[1]) > cv::norm(flow_pts[1] - flow_pts[2])) {
//                        // 0-1 is the long side
//                        sorted_pts = {flow_pts[0], flow_pts[1], flow_pts[2], flow_pts[3]};
//                    } else {
//                        // 1-2 is the long side
//                        sorted_pts = {flow_pts[1], flow_pts[2], flow_pts[3], flow_pts[0]};
//                    }
//                    //sort 了个坤8 这个flowpts更是一坨屎
//                    //dst框 理所应当的依托答辩、、nop
//                    line(rune_debug, sorted_pts[0], sorted_pts[1], cv::Scalar(255,255,123), 5);//坤坤
//                    circle(rune_debug, sorted_pts[1], 11, cv::Scalar(255,255,123), 5);
//                    circle(rune_debug, sorted_pts[0], 11, cv::Scalar(255,255,123), 5);
////                    by yuki-mar5，貌似偶尔1near center捏。
//                    //0 is near the center
//                    if (
//                            cv::norm(sorted_pts[0] - fana.rrect.center) >
//                            cv::norm(sorted_pts[1] - fana.rrect.center)) {
//                        swap(sorted_pts[0], sorted_pts[1]);
//                    }
//                    if (
//                            cv::norm(sorted_pts[2] - fana.rrect.center) >
//                            cv::norm(sorted_pts[3] - fana.rrect.center)) {
//                        swap(sorted_pts[2], sorted_pts[3]);
//                    }
//
//                    circle(rune_debug, sorted_pts[0], 9, cv::Scalar(255,255,123), 5);
//                    //睾丸。
//                    //貌似0并不是很near center
//                    /**
//                    circle(rune_debug, sorted_pts[1], 7, cv::Scalar(255,204,102), 5);
//                    circle(rune_debug, sorted_pts[3], 7, cv::Scalar(255,204,102), 5);
//                     所以这个sorted只是为了求个角度而已吗？
//                    **/
//                    circle(rune_debug, sorted_pts[2], 7, cv::Scalar(255,204,102), 5);
//                    fana.flow_far_from_center = (sorted_pts[0] + sorted_pts[2]) / 2;
//                    fana.towards = (sorted_pts[1] - sorted_pts[0]) / norm(sorted_pts[0] - sorted_pts[1]);
//                    cout<<fana.towards<<111<<endl;
//                    ////////////////找到灯条角度/////////////
//                    auto ang =
//                            cv::fastAtan2(sorted_pts[0].y - sorted_pts[1].y, sorted_pts[0].x - sorted_pts[1].x);
//                    fana.fan_angle = ang;
//                    int i = 0;//解决重叠问题
//                    putText(
//                            rune_debug, "long_angle shit 1" + std::to_string(ang), cv::Point2f(10+i, 80+i), 2, 2, cv::Scalar(135, 206, 235),
//                            2);
//                    //不在
//                    cv::Point2f vertices[4];
//                    if(debug)
//                        imshow("sb",rune_debug);//这里是灯条的框框
//                    //ooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
//                    fana.rrect.points(vertices);
//                    //shittttttttttttttttttttttttttttttttttttttttt
//                    //  ttttttttttttttttttttttttttttttttttttttttttttttttttttttttttt
//
//                    std::vector<cv::Point2f> roi_pts = {vertices, vertices + 4};
//                    for(int i=0;i<4;i++){
//                        line(rune_debug,vertices[i],vertices[(i+1)%4],Scalar(255,255,255),3);//四个角点连成线，最终形成旋转的矩形。
//                    }
//
//                    cv::Rect rect = cv::boundingRect(fana.fan_contours);
////                                          if (!makeRectSafe(rect, image_process_.src_.size())) continue;
////                     cv::Point2f coo = rect.tl();
//                    cv::Mat roi = filled_contour_img(rect);
//                    // 扇叶的最小外接矩形
//                    //呃呃 吐了
//
//
//                    cv::RotatedRect rrect = cv::minAreaRect(fana.fan_contours);
//                    if (rrect.size.width > rrect.size.height) {
//                        fan_angle = 90 + rrect.angle;
//                        std::swap(rrect.size.width, rrect.size.height);
//                    } else {
//                        fan_angle = rrect.angle;
//                    }
//                    cv::Point2f roi_center = cv::Point2f(roi.cols / 2, roi.rows / 2);
//
//                    // 旋转图形，使图片信息不丢失q
//                    cv::Mat rot = getRotationMatrix2D(roi_center, ang - 90, 1);
////90？？？？？？？？？？？？？？？、
//                    cv::Mat rot_g(3, 3, CV_64F);
//                    for (int i = 0; i < 2; ++i) {
//                        for (int j = 0; j < 3; ++j) {
//                            rot_g.at<double>(i, j) = rot.at<double>(i, j);
//                        }
//                    }
//                    //why is pixel in 123123123123123123  --------------------------------Mar4
//                    rot_g.at<double>(2, 0) = 0;
//                    rot_g.at<double>(2, 1) = 0;
//                    rot_g.at<double>(2, 2) = 1;
//                    //可能或许是旋转矩阵那个公式？又不太像、、90°的话应该就是了吧。
//                    invert(rot_g, rot_g);
//                    //这几行到底是什么shit
//                    cv::Rect2f bbox = cv::RotatedRect(roi_center, roi.size(), fan_angle).boundingRect2f();
//                    rot.at<double>(0, 2) += bbox.width / 2.0 - roi.cols / 2.0;
//                    //屎。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。MAR5
//                    rot.at<double>(1, 2) += bbox.height / 2.0 - roi.rows / 2.0;
//                    cv::Mat rot_roi;
//                    warpAffine(roi, rot_roi, rot, bbox.size());
//
////                    imshow("rot_roi",rot_roi); //shi
//
//                    // 扇叶中心旋转后的点
//
//                    cv::Mat rrect_center_mat =
//                            (cv::Mat_<double>(3, 1) << rrect.center.x - rect.tl().x, rrect.center.y - rect.tl().y,
//                                    1);
//                    cv::Mat rot_mat = rot * rrect_center_mat;
//
//
//
//                    cv::Point2f rot_center = cv::Point2f(rot_mat.at<double>(0, 0), rot_mat.at<double>(1, 0));
//                    //this is shit of allllllllllllllllll
//
//                    //截取矫正的图形
//                    cv::Mat dst;
//
//
//                    getRectSubPix(rot_roi, rrect.size, rot_center, dst);
//                    //the key of all！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！1
//
//
//                    circle(dst, rot_center, 5, cv::Scalar(255, 255, 255), -1);
////这个是那个目前比较飘的白点，貌似最终锁定的圆心是那个墨绿色的
//
//
//
//
//                    std::vector<cv::Point2f> armor_pts;
//                    double start = dst.rows / 3;
////                    double pixel_ratio = 0;
////                    while (pixel_ratio < 0.3) {
////                        pixel_ratio = PixelContour(dst, start++, 1) / (1 * dst.cols);
////                    }
//                    //????????????????
//                    //鉴定为老版的shit没删完//我的妈这玩意居然对程序有影响吗
//
//
//
//
//
//
//
//
//
//                    cv::Point2f roi_target_center(dst.cols / 2, (start + dst.rows) / 2);
//                    fana.fan_cols = dst.cols;
//                    fana.fan_center = rrect.center;
//                    //                        auto qwq=inverse_affine(rot_g,roi_target_center);
//                    //                        auto awa=qwq+rrect.center;
//                    //                        Point2f junction(rrect.center.x+cos(ang/180*CV_PI)*roi_target_center.y,rrect.center.y+cos(ang/180*CV_PI)*roi_target_center.y);
//                    cv::Point2f target_center(dst.cols / 2, (start + dst.rows) / 2 - dst.rows / 2);
//
//                    //                        Point2f point_4_left_top    (rrect.center.x+cos(ang/180*CV_PI)*dst.cols,rrect.center.y+sin(ang/180*CV_PI)*dst.rows);
//                    //                        Point2f point_4_right_top   (rrect.center.x+cos(ang/180*CV_PI)*0,       rrect.center.y+sin(ang/180*CV_PI)*dst.rows);
//                    //                        Point2f point_4_right_bottom(rrect.center.x+cos(ang/180*CV_PI)*0,       rrect.center.y+sin(ang/180*CV_PI)*start);
//                    //                        Point2f point_4_left_bottom (rrect.center.x+cos(ang/180*CV_PI)*dst.cols,rrect.center.y+sin(ang/180*CV_PI)*start);
//                    cv::Mat point_left_top = (cv::Mat_<double>(3, 1) << dst.cols, dst.rows, 1);
//                    cv::Mat point_right_top = (cv::Mat_<double>(3, 1) << 0, dst.rows, 1);
//                    cv::Mat point_right_bottom = (cv::Mat_<double>(3, 1) << 0, start, 1);
//                    cv::Mat point_left_bottom = (cv::Mat_<double>(3, 1) << dst.cols, dst.rows, 1);
//                    cv::Mat yuantu_point_left_top = rot_g * point_left_top;
//                    cv::Mat yuantu_right_top = rot_g * point_right_top;
//                    cv::Mat yuantu_right_bottom = rot_g * point_right_bottom;
//                    cv::Mat yuantu_point_left_bottom = rot_g * point_left_bottom;
//                    // cv::Point2f point_4_left_top = cv::Point2f(
//                    //   yuantu_point_left_top.at<double>(0, 0), yuantu_point_left_top.at<double>(1, 0));
//                    // cv::Point2f point_4_right_top =
//                    //   cv::Point2f(yuantu_right_top.at<double>(0, 0), yuantu_right_top.at<double>(1, 0));
//                    // cv::Point2f point_4_right_bottom =
//                    //   cv::Point2f(yuantu_right_bottom.at<double>(0, 0), yuantu_right_bottom.at<double>(1, 0));
//                    // cv::Point2f point_4_left_bottom = cv::Point2f(
//                    //   yuantu_point_left_bottom.at<double>(0, 0), yuantu_point_left_bottom.at<double>(1, 0));
//                    cv::Point2f junction(
//                            rrect.center.x + cos(ang / 180 * CV_PI) * target_center.y,
//                            rrect.center.y + sin(ang / 180 * CV_PI) * target_center.y);
//                    ////
//
//                    fana.target_center = junction;
//                    cv::Mat debug = dst.clone();
//                    cvtColor(dst, debug, cv::COLOR_GRAY2BGR);
//                    circle(debug, cv::Point2f(dst.cols / 2, start), 2, cv::Scalar(0, 255, 0), 2);
//                    circle(debug, roi_target_center, 6, cv::Scalar(11, 123, 0), 2);
//                    //屏蔽掉某pixel后出来的固定位置大绿圈
//                    circle(rune_debug, rrect.center, 10, cv::Scalar(118, 255, 255), 2);
//
////                    circle(rune_debug, junction, 8, cv::Scalar(255, 0, 200), 2);//圆心！！！！！！
//                    circle(rune_debug, junction, 8, cv::Scalar(255, 255, 0), 2);
//                    // circle(rune_debug, point_4_left_top + coo, 8, cv::Scalar(255, 0, 200), 2);
//                    // circle(rune_debug, point_4_right_top + coo, 8, cv::Scalar(255, 0, 200), 2);
//                    // circle(rune_debug, point_4_right_bottom + coo, 8, cv::Scalar(255, 0, 200), 2);
//                    // circle(rune_debug, point_4_left_bottom + coo, 8, cv::Scalar(255, 0, 200), 2);
//                    //                        circle(rune_debug,point_4_left_top,4,Scalar(255,0,200),2);
//                    imshow("destination", debug);
//                    //偶尔挺准，不知道为啥）
//                    imshow("rune_debug circle",rune_debug);
//                    //到此为止，下面两个imshow没有出来
//                    final_fan = fana;
//
//                    return true;
//                } else {
//                    // RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "No flow found!");
//                    continue;
//                }
//            } else {
//                continue;
//            }
//        }
//
//        imshow("flow_roi111",flow_roi);
////        imwrite("flow_roi",flow_roi);
//        imshow("rune_debug——fin",rune_debug);
////        imwrite("rune_debug",rune_debug);
////        printf("1111111");
//    }
//    return false;
//}
//
//
//
//
//
//
//
//
//
//
//bool RuneDetector::findCenter(cv::Mat & src)
//{
//    // auto time_q = std::chrono::steady_clock::now();
////    cv::waitKey(4);
//    if (imageProcess(src)) {//一口气全跑完，按理来说只要一个findcenter
//        if (fanSizer(fillContour())) {
//            cv::Mat mask = cv::Mat::zeros(binary_img.size(), CV_8UC1);
//            //黑底
//            //        int radius = cv::norm(final_fan.fan_cols) * 2;
//
//            auto r_coord = final_fan.fan_center + final_fan.towards * final_fan.fan_cols * 1.3;
//
//            //another key
//
//
//
//            cv::circle(mask, r_coord, 25, cv::Scalar(255), -1);
//            //?
//            cv::circle(rune_debug, r_coord, 25, cv::Scalar(255,255,255), 2);
//            //这三坨到底是啥.........MAR6
//            cv::Mat R_roi = binary_img.mul(mask);
//            //R标从大变小再消失，狗屎//所以为什么这玩意不写在上面。。
//            // Find the center
//            std::vector<std::vector<cv::Point>> contours;
//            cv::findContours(R_roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//            // std::cout<<contours.size()<<std::endl;
//            for (const auto & contour : contours) {
//                //anohter rect
//                cv::Rect rect = cv::boundingRect(contour);
//                // std::cout<<"rect_area"<<rect.area()<<std::endl;
//                // std::cout<<"min"<<rune_param.min_r_area<<std::endl;
//                // std::cout<<"max"<<rune_param.max_r_area<<std::endl;
//                if (rune_param.min_r_area < rect.area() && rect.area() < rune_param.max_r_area) {
//
//                    float ratio = (float)rect.height / (float)rect.width;
//#define SHOW_R_ROI
//#define changeTo2f(x) std::to_string(int(x)) + "." + std::to_string(int(x * 100 + 0.5) % 100)
//
//#ifdef SHOW_R_ROI
//                    cv::putText(
//                            R_roi, "a:" + changeTo2f(rect.area()), cv::Point2f(rect.br()) + cv::Point2f(0, 0),
//                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
//
//                    cv::putText(
//                            R_roi, "r:" + changeTo2f(ratio), cv::Point2f(rect.br()) + cv::Point2f(0, 20),
//                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);
//
//                    cv::imshow("R roi1", R_roi);
//#endif
//                    // std::cout<<ratio<<std::endl;
//                    if (rune_param.min_r_ratio < ratio && ratio < rune_param.max_r_ratio) {
//                        r_center = (rect.br() + rect.tl()) * 0.5;  //R标中心
//                        std::cout<<r_center.x<<std::endl;
//                        std::cout<<r_center.y<<std::endl;
//
//
//
//                        final_fan.fan_angle =  cv::fastAtan2(final_fan.flow_far_from_center.y - r_center.y, final_fan.flow_far_from_center.x - r_center.x);
//                        cv::circle(rune_debug, r_center, 3, cv::Scalar(251, 206, 235), 2);
//
//#define SHOW_RUNE_CENTER
//#ifdef SHOW_RUNE_CENTER
////                         circle(rune_detector_debug, foot_point, 1, cv::Scalar(255, 255, 255), 10);
//                        imshow("R roi", R_roi);
////             imshow("rune detector debug", rune_detector_debug);
//#endif
//                        // auto time_cap = std::chrono::steady_clock::now();
//                        // auto time1 = (std::chrono::duration<double, std::milli>(time_cap - time_q).count());
//                        return true;
//                    }
//                }
//            }
//            return false;
//        } else
//            return false;
//    }
//    return false;
//}
//
//
//
//
//



/**first
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

    morphologyEx(binary_img, binary_img, cv::MORPH_DILATE, kernel7);
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
    }//大小合适的轮廓画出来，并塞满??

//这里也没啥好改的 虽然有点多余

//    std::vector<ShootFan> fans;

    ////膨胀轮廓使其连续
    //    dilate(filled_contour_img,filled_contour_img,kernel7);
    morphologyEx(filled_contour_img, filled_contour_img, cv::MORPH_CLOSE, kernel3
    );
//去除细小空洞，感觉没啥用。准备做掉！！！！！！！
//not connected have the only reason that ----------------------float is not enough
//    imshow("what the fuck you find",filled_contour_img);
    //this time also uncorrect with this first!!!!!!!!!!!!!!!shitttttttttttttt
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
    //true 靠画出来的
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
        //自己画的一个fanaroi pts，但是没做排序，搞鸡毛
        std::vector<cv::Point> lights_roi1_pts2 = {
                fana.longside_centers[1] - fana.long_side / norm(fana.long_side) * 20,
                fana.longside_centers[1] - 100 * fana.long_side / norm(fana.long_side),
                fana.longside_centers[0] - 100 * fana.long_side / norm(fana.long_side),
                fana.longside_centers[0] - fana.long_side / norm(fana.long_side) * 20};

        ///////////寻找流水灯条////////////
        //实测结论：老绿框（step2后的识别没啥问题，黄框的识别有问题（Step3））
        circle(rune_debug, lights_roi1_pts1[1], 9, Scalar(0,0,255));

        circle(rune_debug, lights_roi1_pts1[2], 11, Scalar(255,255,255));
        //顺序不能确定
        //在底下的时候甚至没有白圈

        //变形的有点严重我只能说。30-40环也就是极限了，太畸形了
        //直接重写吧 不演了
        flow_roi = cv::Mat(getROI(lights_roi1_pts1, lights_roi1_pts2));
//为什么0010007有时候海星有时候一坨屎呢，因为排序做的不行
        imshow("00010007",flow_roi);
        std::vector<std::vector<cv::Point>> flow_roi_contours;
        cv::findContours(flow_roi, flow_roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//我看只能find出个鸡毛来

        for (const auto & contour: flow_roi_contours) {

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
                    line(rune_debug, sorted_pts[0], sorted_pts[1], cv::Scalar(255,255,0), 5);//坤坤

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
//                     putText(
//                       rune_debug, "long_angle shit 1" + std::to_string(ang), cv::Point2f(10, 80), 2, 2, cv::Scalar(135, 206, 235),
//                       2);
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
                    //呃呃 吐了


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

                    //imshow("rot_roi",rot_roi); //shi

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
            //黑底
            //        int radius = cv::norm(final_fan.fan_cols) * 2;

            auto r_coord = final_fan.fan_center + final_fan.towards * final_fan.fan_cols * 1.3;
            ////ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff
            cv::circle(mask, r_coord, 15, cv::Scalar(255,255,255), -1);
            cv::circle(rune_debug, r_coord, 25, cv::Scalar(255,255,255), 2);
            cv::Mat R_roi = binary_img.mul(mask);
            //shitshit
            //R标从大变小再消失，狗屎
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
//             imshow("rune detector debug", rune_detector_debug);
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

**/

int main() {
    //at least yeallow is right
    HaiKangCamera HaiKang;
    HaiKang.StartDevice(0);
    HaiKang.SetResolution(640,480);
    HaiKang.SetFPS(400);
    HaiKang.SetStreamOn();
    int exposet=4500;
    HaiKang.SetExposureTime(exposet);
    //still not the flow include(but why?)
    HaiKang.SetGAIN(0, 10);
    HaiKang.GetImageParam(HaiKang.m_param);

#ifdef OPEN_SERIAL
    while( bool usb = SP.initSerial("/dev/ttyUSB0", 115200, 'N', 8, 1) == false);
    send_data.clear();
#endif
    while (true) {
        Mat src;
        HaiKang.GetMat(src);
        RuneDetector rune_detector = RuneDetector(RuneDetector::RuneParam());
        //1
//        rune_detector.imageProcess(src);
//        //2
//        rune_detector.fillContour();

//        //3
//        rune_detector.fanSizer(fans);




//        imshow("1w",src);
//        //畸变抗性000000000000000000000000000.1
////        resize(src,src,Size(0,0),0.7,0.7);
        Mat hsv=src.clone();
//#ifdef videoroi
//        Mat srcROI = src(Rect(590, 250, 600, 400));
//        imshow("?rlkoi",srcROI);
#ifdef hsv_test
        Mat bbq=hsv.clone();
        colorFilter(bbq,bbq);
        imshow("hsv",bbq);
        Mat mask;
        inRange(hsv, Scalar(150, 43, 46), Scalar(255, 255, 255), mask);
        imshow("1234567",mask);
//        src=srcROI.clone();
#endif
        rune_detector.findCenter(src);
//        imshow("1",src);


















#ifdef OPEN_SERIAL
        if(!SP.sendData(7,send_data,0xAA,0xAF)){
            while(!SP.initSerial("/dev/ttyUSB0", 115200, 'N', 8, 1));
        }

        cout << "start sleeping..." << endl;
        std::this_thread::sleep_for(milliseconds(221)
        );
        cout << "sleeping finished." << endl;
#endif
        if (waitKey(1) == 'q')
            break;
    }
    return 0;
}



//
//int main() {
//    char fn[100]=filename;
//    VideoCapture video(fn);
//    Mat src;
//    for (;;) {
//        video >> src;
//        if (src.empty()) {
//            waitKey(90000);
//            waitKey(100);
//            break;
//        }
////        imshow("1w",src);
//        //畸变抗性000000000000000000000000000.1
////        resize(src,src,Size(0,0),0.7,0.7);
////        Mat hsv=src.clone();
//#ifdef videoroi
//        Mat srcROI = src(Rect(590, 250, 600, 400));
//        imshow("?rlkoi",srcROI);
//        //Mat bbq=hsv.clone();
//      //  colorFilter(bbq,bbq);
////        Mat mask;
////        inRange(hsv, Scalar(150, 43, 46), Scalar(255, 255, 255), mask);
////        imshow("1234567",mask);
//        src=srcROI.clone();
//#endif
////        cv::threshold(src, src,233,255,cv::THRESH_BINARY);
//        //why i should threshold by myself
//        //wtf
//
////wtfufind 里其实find海星？、、//好吧应该不太行 感觉很垃圾写的。
//
////        imshow("mask1", mask);
//        RuneDetector rune_detector = RuneDetector(RuneDetector::RuneParam());
//        //1
//        rune_detector.imageProcess(src);
//        //2
//        rune_detector.fillContour();
//        //3
//        rune_detector.fanSizer(fans);
////        //4
////
//        rune_detector.findCenter(src);
////        imshow("1",src);
//        if (waitKey(delay) >= 10000) {
////            imwrite("111",rune_detector.R_roi)
//            waitKey(100);
//            break;
//        }
//    }




//}












































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