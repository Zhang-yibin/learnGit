//
// Created by root on 2022/7/4.
//

#include "rune_detector.h"
#include <fmt/format.h>
#include <fmt/color.h>

void RuneDetector::colorSpilt(Mat& input,Mat& output,int div) {
//    output=input.clone();
    int rowNumber = output.rows;
    int colNumber = output.cols;
    for (int i = 0; i < rowNumber; i++) {
        auto *datai = input.ptr<uchar>(i);
        auto *datao = output.ptr<uchar>(i);
        for (int j = div, k = 0; j < colNumber * input.channels(); j += 3, k++) {
            datao[k] = datai[j];
        }
    }
}
double RuneDetector::PixelCounter(Mat rot,double start,int height){
    double sum=0;
    for(int i=start;i<start+height;i++){
        auto*data=rot.ptr<uchar>(i);
        for(int j=0;j<rot.cols;j++){
            if(data[j]==255){
                sum+=1;
            }
        }
    }
    return sum;
}






Point2f inverse_affine(Mat rot,Point2f q){
    Point2f ans;
//    cout<<rot.at<double>(0,0)<<rot.at<float>(0,1)<<rot.at<float>(0,2);
    ans.x = rot.at<double>(0,0) *q.x+rot.at<double>(0,1) *q.y+rot.at<double>(0,2);
    ans.y = rot.at<double>(1,0) *q.x+rot.at<double>(1,1) *q.y+rot.at<double>(1,2);
    return ans;
}

void createRoi(Mat& roi,Mat& src,Rect a) {
//    output=input.clone();
    int rowNumber = a.height;
    int colNumber = a.width;
    for (int i = 0,ii=a.tl().y; i < rowNumber,ii<a.br().y; i++,ii++) {
        auto *datai = roi.ptr<uchar>(i);
        auto *dataii= src.ptr<uchar>(ii);
        for (int j = 0,jj=a.tl().x; j < colNumber ,jj<a.br().x; j++,jj++) {
            datai[j]=dataii[jj];
//            datai[j]=255;
        }
    }
}


//111
bool RuneDetector::imageProcess(Mat src) {
    if (src.empty())
        return false;
    vector <Mat> channels;
    rune_debug=src.clone();
    cv::split(src,channels);
//    cout<<"asdasd"<<detect_color<<endl;
    if(detect_color==self_RED)
        binary = channels.at(2);
    else
        binary = channels.at(0);
    threshold(binary, binary, rune_param. binary_threshold, 255, cv::THRESH_BINARY);
    //TODO:
    morphologyEx(binary, binary, cv::MORPH_DILATE, kernel5);
//    morphologyEx(binary, binary, cv::MORPH_DILATE, kernel3);

#ifdef SHOW_RUNE_BINARY
    imshow("binary", binary);
#endif
    return true;
}
//222
std::vector<ShootFan> RuneDetector::fillContour() {

    ////创建轮廓填充图 填充轮廓
    contour_filled=Mat::zeros(Size(rune_debug.cols,rune_debug.rows),CV_8UC1);

    std::vector<std::vector<Point> > contours;

    ////寻找主要轮廓并填充 需要调整area大小
    findContours(binary, filledcontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < filledcontours.size(); i++) {
        double area = contourArea(filledcontours[i]);
        if (area < rune_param.min_filledcontours_area)
            continue;
        drawContours(contour_filled, filledcontours, static_cast<int>(i), cv::Scalar(255, 255, 255), FILLED);
    }

    std::vector<ShootFan> fans;

    ////膨胀轮廓使其连续
        dilate(contour_filled,contour_filled,kernel3);
    morphologyEx(contour_filled, contour_filled, cv::MORPH_CLOSE, kernel5);

    findContours(contour_filled, filledcontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (size_t i = 0; i < filledcontours.size(); i++) {
        double area = contourArea(filledcontours[i]);
//        drawContours(rune_debug, filledcontours, static_cast<int>(i), cv::Scalar(0, 255, 255), 2);
        auto rect = cv::minAreaRect(filledcontours[i]);
        if (area < rune_param.min_contourArea||area>rune_param.max_contourArea)
            continue;
        float ratio = rect.size.width > rect.size.height ? rect.size.width / rect.size.height
                                                         : rect.size.height / rect.size.width;
        if(rect.size.area() > rune_param.min_fan_area
           && rect.size.area() < rune_param.max_fan_area
           && rect.size.area() / area > rune_param.min_area_ratio
           && (ratio>rune_param.min_fan_ratio && ratio<rune_param.max_fan_ratio)){
//todo:: 需要添加更多的条件  放入扇叶候选队列
            fans.emplace_back(filledcontours[i],rect);
        }else continue;

    }
    //todo::ifdef只是个画图
#ifdef DRAW
    for (const auto &fan : fans) {
        Point2f vertices[4];
        fan.rrect.points(vertices);
        putText(rune_debug, "fan_angle"+to_string(fan.rrect.angle),Point2f(10,30),2,2,Scalar(135,206,235),2);
        for(int i=0;i<4;i++){
            line(rune_debug,vertices[i],vertices[(i+1)%4],Scalar(0,255,255));//四个角点连成线，最终形成旋转的矩形。
        }
    }
    cv::imshow("1",rune_debug);
#endif
//    imshow("ccc",contour_filled);
    return fans;
}
//////////扇叶再次筛选////////////
//333
bool RuneDetector::fanSizer(vector<ShootFan> fans) {
    if(fans.empty()){
        fmt::print(fmt::fg(fmt::color::red), "fan is empty!\n");
        return false;
    }

    ////寻找长边中心点////
    for (auto &fan : fans) {
        Point2f fan_pts[4];
        fan.rrect.points(fan_pts);
        if (cv::norm(fan_pts[0] - fan_pts[1]) > cv::norm(fan_pts[1] - fan_pts[2])) {
            // 0-1 is the long side
            sorted_pts = {fan_pts[0], fan_pts[1], fan_pts[2], fan_pts[3]};
        } else {
            // 1-2 is the long side
            sorted_pts = {fan_pts[1], fan_pts[2], fan_pts[3], fan_pts[0]};
        }
        fan.long_side = sorted_pts[0] - sorted_pts[1];
        Point2f longcenter1,longcenter2;
        longcenter1=(sorted_pts[0]+sorted_pts[1])/2;
        longcenter2=(sorted_pts[2]+sorted_pts[3])/2;
        fan.longside_centers.emplace_back(longcenter1);
        fan.longside_centers.emplace_back(longcenter2);

#ifdef DRAW_CIRCLE
        //            line(frame,sorted_pts[0],sorted_pts[1],Scalar(255,255,0),3);
//            line(frame,sorted_pts[2],sorted_pts[3],Scalar(255,255,0),3);
            circle(rune_debug,longcenter1,3,Scalar(0,255,255));
            circle(rune_debug,longcenter2,3,Scalar(0,255,255));
#endif
    }

    auto getROI = [&](const std::vector<cv::Point> &roi_pts1,const std::vector<cv::Point> &roi_pts2) -> cv::Mat {
        cv::Mat mask = cv::Mat::zeros(contour_filled.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> vpts = {roi_pts1,roi_pts2};
        cv::fillPoly(mask, vpts, cv::Scalar(255));
        return contour_filled & mask;

    //海星
    };
    Mat flow_roi;
    for (auto &fana : fans) {
        std::vector<cv::Point> lights_roi1_pts1 = {
            fana.longside_centers[1] + fana.long_side / norm(fana.long_side) * 20,
            fana.longside_centers[1] + 100 * fana.long_side / norm(fana.long_side),
            fana.longside_centers[0] + 100 * fana.long_side / norm(fana.long_side),
            fana.longside_centers[0] + fana.long_side / norm(fana.long_side) * 20};
        std::vector<cv::Point> lights_roi1_pts2 = {
            fana.longside_centers[1] - fana.long_side / norm(fana.long_side) * 20,
            fana.longside_centers[1] - 100 * fana.long_side / norm(fana.long_side),
            fana.longside_centers[0] - 100 * fana.long_side / norm(fana.long_side),
            fana.longside_centers[0] - fana.long_side / norm(fana.long_side) * 20};

///////////寻找流水灯条////////////
cout<<lights_roi1_pts1[0]<<endl;
        flow_roi = Mat(getROI(lights_roi1_pts1, lights_roi1_pts2));

      imshow("roiroi", flow_roi);
        std::vector<std::vector<cv::Point>> flow_roi_contours;
        cv::findContours(flow_roi, flow_roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto &contour: flow_roi_contours) {
            double area = contourArea(contour);
            auto rect = cv::minAreaRect(contour);
//            cout<<rect.size.area()<<endl;
            if (rect.size.area() >rune_param.min_flow_area && rect.size.area() < rune_param.max_flow_area) {
                float ratio = rect.size.width > rect.size.height ? rect.size.width / rect.size.height
                                                                 : rect.size.height / rect.size.width;
                if (rune_param.min_ratio < ratio && ratio < rune_param.max_ratio
                    &&rect.size.area() / area > rune_param.min_flow_area_ratio) {
                    Point2f flow_pts[4];
                    rect.points(flow_pts);
                    if (cv::norm(flow_pts[0] - flow_pts[1]) > cv::norm(flow_pts[1] - flow_pts[2])) {
                        // 0-1 is the long side
                        sorted_pts = {flow_pts[0], flow_pts[1], flow_pts[2], flow_pts[3]};
                    } else {
                        // 1-2 is the long side
                        sorted_pts = {flow_pts[1], flow_pts[2], flow_pts[3], flow_pts[0]};
                    }
                        line(rune_debug,sorted_pts[0],sorted_pts[1],Scalar(230,255,60),5);
                        circle(rune_debug,sorted_pts[0],5,Scalar(134,87,223),3);
                    //0 is near the center
                    if( norm( sorted_pts[0] - fana.rrect.center ) > norm( sorted_pts[1] - fana.rrect.center ) ){
                        swap(sorted_pts[0],sorted_pts[1]);
                    }
                    fana.towards=(sorted_pts[1]-sorted_pts[0])/norm(sorted_pts[0]-sorted_pts[1]);
                    ////////////////找到灯条角度/////////////
                    auto ang= fastAtan2(sorted_pts[0].y-sorted_pts[1].y,sorted_pts[0].x-sorted_pts[1].x);
                    fana.fan_angle=ang;
                    putText(rune_debug, "long_angle"+to_string(ang),Point2f(10,80),2,2,Scalar(135,206,235),2);
                    Point2f vertices[4];
                    fana.rrect.points(vertices);
                    vector<Point2f> roi_pts={vertices,vertices+4};
//                        for(int i=0;i<4;i++){
//                            line(rune_debug,vertices[i],vertices[(i+1)%4],Scalar(255,255,255),3);//四个角点连成线，最终形成旋转的矩形。
//                        }

                    Rect rect = boundingRect(fana.fancontours);
//                      if (!makeRectSafe(rect, image_process_.src_.size())) continue;
                    Point2f coo=rect.tl();
                    Mat roi = contour_filled(rect);
                    // 扇叶的最小外接矩形
                    RotatedRect rrect = minAreaRect(fana.fancontours);
                    if (rrect.size.width > rrect.size.height) {
                        rot_angle = 90 + rrect.angle;
                        swap(rrect.size.width, rrect.size.height);
                    } else {
                        rot_angle = rrect.angle;
                    }
                    Point2f roi_center = Point2f(roi.cols / 2, roi.rows / 2);

                    // 旋转图形，使图片信息不丢失q
                    Mat rot = getRotationMatrix2D(roi_center, ang-90, 1);

                    Mat rot_g(3,3,CV_64F);
                    for(int i=0;i<2;++i){
                        for(int j=0;j<3;++j){
                            rot_g.at<double>(i,j)=rot.at<double>(i,j);
                        }
                    }
                    rot_g.at<double>(2,0)=0;
                    rot_g.at<double>(2,1)=0;
                    rot_g.at<double>(2,2)=1;
                    invert(rot_g,rot_g);
                    Rect2f bbox = RotatedRect(roi_center, roi.size(), rot_angle).boundingRect2f();
                    rot.at<double>(0, 2) += bbox.width / 2.0 - roi.cols / 2.0;
                    rot.at<double>(1, 2) += bbox.height / 2.0 - roi.rows / 2.0;
                    Mat rot_roi;
                    warpAffine(roi, rot_roi, rot, bbox.size());

                    // 扇叶中心旋转后的点
                    Mat rrect_center_mat = (Mat_<double>(3, 1) << rrect.center.x - rect.tl().x,
                            rrect.center.y - rect.tl().y,1);
                    Mat rot_mat = rot * rrect_center_mat;
                    Point2f rot_center = Point2f(rot_mat.at<double>(0, 0), rot_mat.at<double>(1, 0));

                    // 截取矫正的图形
                    Mat dst;
                    getRectSubPix(rot_roi, rrect.size, rot_center, dst);
                    circle(dst,rot_center,3,Scalar(255,255,255),2);


                    vector<Point2f> armor_pts;
                    double start=dst.rows/3;
                    double pixel_ratio=0;
                    while(pixel_ratio<0.3){
                        pixel_ratio=PixelCounter(dst,start++,1)/(1*dst.cols);
                    }
                    Point2f roi_target_center(dst.cols/2,(start+dst.rows)/2);
                    fana.fan_cols=dst.cols;
                    fana.fan_center=rrect.center;
//                        auto qwq=inverse_affine(rot_g,roi_target_center);
//                        auto awa=qwq+rrect.center;
//                        Point2f junction(rrect.center.x+cos(ang/180*CV_PI)*roi_target_center.y,rrect.center.y+cos(ang/180*CV_PI)*roi_target_center.y);
                    Point2f target_center(dst.cols/2,(start+dst.rows)/2-dst.rows/2);

//                        Point2f point_4_left_top    (rrect.center.x+cos(ang/180*CV_PI)*dst.cols,rrect.center.y+sin(ang/180*CV_PI)*dst.rows);
//                        Point2f point_4_right_top   (rrect.center.x+cos(ang/180*CV_PI)*0,       rrect.center.y+sin(ang/180*CV_PI)*dst.rows);
//                        Point2f point_4_right_bottom(rrect.center.x+cos(ang/180*CV_PI)*0,       rrect.center.y+sin(ang/180*CV_PI)*start);
//                        Point2f point_4_left_bottom (rrect.center.x+cos(ang/180*CV_PI)*dst.cols,rrect.center.y+sin(ang/180*CV_PI)*start);
                    Mat point_left_top =     (Mat_<double>(3, 1) << dst.cols,dst.rows,1);
                    Mat point_right_top =    (Mat_<double>(3, 1) << 0,dst.rows,1);
                    Mat point_right_bottom = (Mat_<double>(3, 1) << 0,start,1);
                    Mat point_left_bottom =  (Mat_<double>(3, 1) << dst.cols,dst.rows,1);
                    Mat yuantu_point_left_top= rot_g* point_left_top;
                    Mat yuantu_right_top= rot_g* point_right_top;
                    Mat yuantu_right_bottom= rot_g* point_right_bottom;
                    Mat yuantu_point_left_bottom= rot_g* point_left_bottom;
                    Point2f point_4_left_top = Point2f(yuantu_point_left_top.at<double>(0, 0), yuantu_point_left_top.at<double>(1, 0));
                    Point2f point_4_right_top = Point2f(yuantu_right_top.at<double>(0, 0), yuantu_right_top.at<double>(1, 0));
                    Point2f point_4_right_bottom = Point2f(yuantu_right_bottom.at<double>(0, 0), yuantu_right_bottom.at<double>(1, 0));
                    Point2f point_4_left_bottom = Point2f(yuantu_point_left_bottom.at<double>(0, 0), yuantu_point_left_bottom.at<double>(1, 0));
                    Point2f junction(rrect.center.x+cos(ang/180*CV_PI)*target_center.y,rrect.center.y+sin(ang/180*CV_PI)*target_center.y);
                    ////


                    fana.target_center=junction;
                    Mat debug=dst.clone();
                    cvtColor(dst,debug,COLOR_GRAY2BGR);
                    circle(debug,Point2f (dst.cols/2,start),2,Scalar(0,255,0),2);
                    circle(debug,roi_target_center,2,Scalar(0,255,0),2);
                    circle(rune_debug,rrect.center,10,Scalar(38,255,255),2);
                    circle(rune_debug,junction,8,Scalar(255,0,200),2);
                    circle(rune_debug,point_4_left_top+coo,8,Scalar(255,0,200),2);
                    circle(rune_debug,point_4_right_top+coo,8,Scalar(255,0,200),2);
                    circle(rune_debug,point_4_right_bottom+coo,8,Scalar(255,0,200),2);
                    circle(rune_debug,point_4_left_bottom+coo,8,Scalar(255,0,200),2);
//                        circle(rune_debug,point_4_left_top,4,Scalar(255,0,200),2);
//                        imshow("dstds", debug);
//                        imshow("rune_debug",rune_debug);
                    final_fan=fana;

                    return true;
                }else {
//                        fmt::print(fmt::fg(fmt::color::red), "no flow found!\n");
                    continue;
                }
            }else {

                continue;
            }
        }

//        imshow("roi",roi);
//            imshow("qwq",rune_debug);
    }
    fmt::print(fmt::fg(fmt::color::red), "no flow found!\n");
    return false;
}


Point2f getFitCircle(Point2f pt1, Point2f pt2, Point2f pt3, double &radius) {
    Point2f point;
    double x1 = pt1.x, x2 = pt2.x, x3 = pt3.x;
    double y1 = pt1.y, y2 = pt2.y, y3 = pt3.y;
    double a = x1 - x2;
    double b = y1 - y2;
    double c = x1 - x3;
    double d = y1 - y3;
    double e = ((x1 * x1 - x2 * x2) + (y1 * y1 - y2 * y2)) / 2.0;
    double f = ((x1 * x1 - x3 * x3) + (y1 * y1 - y3 * y3)) / 2.0;
    double det = b * c - a * d;
    if (fabs(det) < 1e-5) {
        radius = -1;
        return point;
    }

    double x0 = -(d * e - b * f) / det;
    double y0 = -(a * f - c * e) / det;
    radius = hypot(x1 - x0, y1 - y0);
    point.x = x0;
    point.y = y0;
    return point;
}

//444
bool RuneDetector::findCenter(Mat src) {
    auto time_q = std::chrono::steady_clock::now();

    if (imageProcess(src)) {
        if(fanSizer(fillContour())){

            cv::Mat mask = cv::Mat::zeros(binary.size(), CV_8UC1);
//        int radius = cv::norm(final_fan.fan_cols) * 2;

            auto r_coord=final_fan.fan_center+final_fan.towards*final_fan.fan_cols*1.3;
            cv::circle(mask, r_coord, 25, cv::Scalar(255), -1);



            cv::Mat R_roi = binary.mul(mask);
            //该函数使用OpenCV库，在二进制图像（binary）上执行与掩码（mask）的逐元素乘法，然后将结果存储到一个名为R_roi的Mat对象中。


            // Find the center
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(R_roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto &contour : contours) {
                cv::Rect rect = cv::boundingRect(contour);
                if (rune_param.min_r_area < rect.area() && rect.area() < rune_param.max_r_area) {
                    float ratio = (float) rect.height / (float) rect.width;
#ifdef SHOW_R_ROI
                    cv::putText(
                            R_roi,
                            "a:" + changeTo2f(rect.area()),
                            Point2f(rect.br()) + Point2f(0, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            Scalar(255), 1);

                    cv::putText(
                            R_roi,
                            "r:" + changeTo2f(ratio),
                            Point2f(rect.br()) + Point2f(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            Scalar(255), 1);


                imshow("R roi", R_roi);
#endif
//                    cv::putText(
//                            R_roi,
//                            "a:" + changeTo2f(rect.area()),
//                            Point2f(rect.br()) + Point2f(0, 0), cv::FONT_HERSHEY_SIMPLEX, 0.5,
//                            Scalar(255), 1);
//
//                    cv::putText(
//                            R_roi,
//                            "r:" + changeTo2f(ratio),
//                            Point2f(rect.br()) + Point2f(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
//                            Scalar(255), 1);
//
//
//                    imshow("R roi", R_roi);
                    if (rune_param.min_r_ratio < ratio && ratio < rune_param.max_r_ratio) {

                        r_center = (rect.br() + rect.tl()) * 0.5;//R标中心

#ifdef SHOW_RUNE_CENTER
                        circle(rune_detector_debug, r_center, 6, Scalar(0, 255, 0), -1);
                    // circle(rune_detector_debug, foot_point, 1, cv::Scalar(255, 255, 255), 10);
                    imshow("R roi",R_roi);
                    imshow("rune detector debug", rune_detector_debug);
#endif
                        auto time_cap = std::chrono::steady_clock::now();
                        auto time1 = (std::chrono::duration<double,std::milli>(time_cap - time_q).count());
                        return true;

                    }
                }
            }
            return true;
        }
        else
            return false;
    }
    return false;
}