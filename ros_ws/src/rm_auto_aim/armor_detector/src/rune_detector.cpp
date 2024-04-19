
#include "armor_detector/detector.hpp"

const static cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
const static cv::Mat kernel5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
const static cv::Mat kernel7 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
const static cv::Mat kernel11 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));

rm_auto_aim::RuneDetector::RuneDetector(const RuneParam & r) :
  rune_param(r)
{
}

bool rm_auto_aim::RuneDetector::imageProcess(cv::Mat & src)
{
  if (src.empty()) 
    return false;
  std::vector<cv::Mat> channels;
  rune_debug = src.clone();
  cv::split(src, channels);

  if (rune_color == self_BLUE)
    rm_auto_aim::RuneDetector::binary_img = channels.at(2);
  else
    binary_img = channels.at(0);

  threshold(binary_img, binary_img, rune_param.binary_threshold, 255, cv::THRESH_BINARY);
  morphologyEx(binary_img, binary_img, cv::MORPH_DILATE, kernel7);

  return true;
}

std::vector<rm_auto_aim::ShootFan> rm_auto_aim::RuneDetector::fillContour()
{
  ////创建轮廓填充图 填充轮廓
  filled_contour_img = cv::Mat::zeros(cv::Size(rune_debug.cols, rune_debug.rows), CV_8UC1);
  std::vector<std::vector<cv::Point>> contours;

  ////寻找主要轮廓并填充 需要调整area大小
  findContours(binary_img, filled_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  for (size_t i = 0; i < filled_contours.size(); i++) {
    double area = contourArea(filled_contours[i]);
    if (area < rune_param.min_filledContours_area) continue;
    drawContours(
      filled_contour_img, filled_contours, static_cast<int>(i), cv::Scalar(255, 255, 255),
      cv::FILLED);
  }

  std::vector<ShootFan> fans;

  ////膨胀轮廓使其连续
  //    dilate(filled_contour_img,filled_contour_img,kernel7);
  morphologyEx(filled_contour_img, filled_contour_img, cv::MORPH_CLOSE, kernel5);

  findContours(filled_contour_img, filled_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  for (size_t i = 0; i < filled_contours.size(); i++) {
    double area = contourArea(filled_contours[i]);
    drawContours(rune_debug, filled_contours, static_cast<int>(i), cv::Scalar(0, 255, 255), 2);
    auto rect = cv::minAreaRect(filled_contours[i]);
    // std::cout<<area<<std::endl;
    if (area < rune_param.min_contourArea || area > rune_param.max_contourArea) continue;
    float ratio = rect.size.width > rect.size.height ? rect.size.width / rect.size.height
                                                     : rect.size.height / rect.size.width;
      // std::cout << "area:" << rect.size.area() << std::endl;
      // std::cout <<"ratio" <<ratio<<std::endl;
      // std::cout<<"wqwq"<<rect.size.area() / area <<std::endl;
    if ((rect.size.area() > rune_param.min_fan_area && rect.size.area() < rune_param.max_fan_area) &&
      rect.size.area() / area > rune_param.min_area_ratio &&
      (ratio > rune_param.min_fan_ratio && ratio < rune_param.max_fan_ratio)) {
      //todo:: 需要添加更多的条件  放入扇叶候选队列
        // std::cout<<"qwqwqwq"<<std::endl;
        fans.emplace_back(filled_contours[i], rect);
    } else
        continue;
  }
  //todo::ifdef只是个画图
// #define DRAW
#ifdef DRAW
  for (const auto & fan : fans) {
    cv::Point2f vertices[4];
    fan.rrect.points(vertices);
    cv::putText(
      rune_debug, "fan_angle" + std::to_string(fan.rrect.angle), cv::Point2f(10, 30), 2, 2,
      cv::Scalar(135, 206, 235), 2);
    for (int i = 0; i < 4; i++) {
      line(
        rune_debug, vertices[i], vertices[(i + 1) % 4],
        cv::Scalar(0, 255, 255));  //四个角点连成线，最终形成旋转的矩形。
    }
  }
  // cv::imshow("1", rune_debug);
#endif
  //    imshow("ccc",contour_filled);
  return fans;
}

bool rm_auto_aim::RuneDetector::fanSizer(std::vector<rm_auto_aim::ShootFan> fans)
{
  if (fans.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "Fan is empty!");
    return false;
  }

  ////寻找长边中心点////
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
    fan.long_side = sorted_pts[0] - sorted_pts[1];
    cv::Point2f longcenter1, longcenter2;
    longcenter1 = (sorted_pts[0] + sorted_pts[1]) / 2;
    longcenter2 = (sorted_pts[2] + sorted_pts[3]) / 2;
    fan.longside_centers.emplace_back(longcenter1);
    fan.longside_centers.emplace_back(longcenter2);

#ifdef DRAW_CIRCLE
    //            line(frame,sorted_pts[0],sorted_pts[1],Scalar(255,255,0),3);
    //            line(frame,sorted_pts[2],sorted_pts[3],Scalar(255,255,0),3);
    circle(rune_debug, longcenter1, 3, Scalar(0, 255, 255));
    circle(rune_debug, longcenter2, 3, Scalar(0, 255, 255));
#endif
  }

  auto getROI = [&](
                  const std::vector<cv::Point> & roi_pts1,
                  const std::vector<cv::Point> & roi_pts2) -> cv::Mat {
    cv::Mat mask = cv::Mat::zeros(filled_contour_img.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> vpts = {roi_pts1, roi_pts2};
    cv::fillPoly(mask, vpts, cv::Scalar(255));
    return filled_contour_img & mask;
  };
  cv::Mat flow_roi;
  for (auto & fana : fans) {
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
    flow_roi = cv::Mat(getROI(lights_roi1_pts1, lights_roi1_pts2));

    std::vector<std::vector<cv::Point>> flow_roi_contours;
    cv::findContours(flow_roi, flow_roi_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto & contour : flow_roi_contours) {
      double area = contourArea(contour);
      auto rect = cv::minAreaRect(contour);
      if (
        rect.size.area() > rune_param.min_flow_area &&
        rect.size.area() < rune_param.max_flow_area) {
        float ratio = rect.size.width > rect.size.height ? rect.size.width / rect.size.height
                                                         : rect.size.height / rect.size.width;
        if (
          rune_param.min_ratio < ratio && ratio < rune_param.max_ratio &&//bug!!!!bug!!!!!!todo::bug!!!!!
          rect.size.area() / area > rune_param.min_flow_area_ratio) {
          cv::Point2f flow_pts[4];
          rect.points(flow_pts);
          if (cv::norm(flow_pts[0] - flow_pts[1]) > cv::norm(flow_pts[1] - flow_pts[2])) {
            // 0-1 is the long side
            sorted_pts = {flow_pts[0], flow_pts[1], flow_pts[2], flow_pts[3]};
          } else {
            // 1-2 is the long side
            sorted_pts = {flow_pts[1], flow_pts[2], flow_pts[3], flow_pts[0]};
          }
          line(rune_debug, sorted_pts[0], sorted_pts[1], cv::Scalar(230, 255, 60), 5);
      
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
          circle(rune_debug, sorted_pts[0], 5, cv::Scalar(134, 87, 223), 3);
          circle(rune_debug, sorted_pts[2], 5, cv::Scalar(134, 87, 223), 3);
          fana.flow_far_from_center = (sorted_pts[0] + sorted_pts[2]) / 2;
          fana.towards = (sorted_pts[1] - sorted_pts[0]) / norm(sorted_pts[0] - sorted_pts[1]);
          ////////////////找到灯条角度/////////////
          auto ang =
            cv::fastAtan2(sorted_pts[0].y - sorted_pts[1].y, sorted_pts[0].x - sorted_pts[1].x);
          fana.fan_angle = ang;
          // putText(
          //   rune_debug, "long_angle" + std::to_string(ang), cv::Point2f(10, 80), 2, 2, cv::Scalar(135, 206, 235),
          //   2);
          cv::Point2f vertices[4];
          fana.rrect.points(vertices);
          std::vector<cv::Point2f> roi_pts = {vertices, vertices + 4};
          //                        for(int i=0;i<4;i++){
          //                            line(rune_debug,vertices[i],vertices[(i+1)%4],Scalar(255,255,255),3);//四个角点连成线，最终形成旋转的矩形。
          //                        }

          cv::Rect rect = cv::boundingRect(fana.fan_contours);
          //                      if (!makeRectSafe(rect, image_process_.src_.size())) continue;
          // cv::Point2f coo = rect.tl();
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

          // 截取矫正的图形
          cv::Mat dst;
          getRectSubPix(rot_roi, rrect.size, rot_center, dst);
          circle(dst, rot_center, 3, cv::Scalar(255, 255, 255), 2);

          std::vector<cv::Point2f> armor_pts;
          double start = dst.rows / 3;
          double pixel_ratio = 0;
          while (pixel_ratio < 0.3) {
            pixel_ratio = PixelContour(dst, start++, 1) / (1 * dst.cols);
          }
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
          circle(debug, roi_target_center, 2, cv::Scalar(0, 255, 0), 2);
          circle(rune_debug, rrect.center, 10, cv::Scalar(38, 255, 255), 2);
          circle(rune_debug, junction, 8, cv::Scalar(255, 0, 200), 2);
          // circle(rune_debug, point_4_left_top + coo, 8, cv::Scalar(255, 0, 200), 2);
          // circle(rune_debug, point_4_right_top + coo, 8, cv::Scalar(255, 0, 200), 2);
          // circle(rune_debug, point_4_right_bottom + coo, 8, cv::Scalar(255, 0, 200), 2);
          // circle(rune_debug, point_4_left_bottom + coo, 8, cv::Scalar(255, 0, 200), 2);
          //                        circle(rune_debug,point_4_left_top,4,Scalar(255,0,200),2);
          //                        imshow("dstds", debug);
          //                        imshow("rune_debug",rune_debug);
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

    //        imshow("roi",roi);
    //            imshow("qwq",rune_debug);
  }
  RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "No flow found!");
  return false;
}

double rm_auto_aim::RuneDetector::PixelContour(cv::Mat & rot, double start, int height)
{
  double sum = 0;
  for (int i = start; i < start + height; i++) {
    auto * data = rot.ptr<uchar>(i);
    for (int j = 0; j < rot.cols; j++) {
      if (data[j] == 255) {
        sum += 1;
      }
    }
  }
  return sum;
}

bool rm_auto_aim::RuneDetector::findCenter(cv::Mat & src)
{
  // auto time_q = std::chrono::steady_clock::now();
  cv::waitKey(4);
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
#ifdef SHOW_R_ROI
          cv::putText(
            R_roi, "a:" + changeTo2f(rect.area()), cv::Point2f(rect.br()) + cv::Point2f(0, 0),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);

          cv::putText(
            R_roi, "r:" + changeTo2f(ratio), cv::Point2f(rect.br()) + cv::Point2f(0, 20),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1);

          // cv::imshow("R roi", R_roi);
#endif
          // std::cout<<ratio<<std::endl;
          if (rune_param.min_r_ratio < ratio && ratio < rune_param.max_r_ratio) {
            r_center = (rect.br() + rect.tl()) * 0.5;  //R标中心
            // std::cout<<r_center.x<<std::endl;
            // std::cout<<r_center.y<<std::endl;



          final_fan.fan_angle =  cv::fastAtan2(final_fan.flow_far_from_center.y - r_center.y, final_fan.flow_far_from_center.x - r_center.x);
          cv::circle(rune_debug, r_center, 3, cv::Scalar(255, 255, 255), 2);
#ifdef SHOW_RUNE_CENTER
            
            // circle(rune_detector_debug, foot_point, 1, cv::Scalar(255, 255, 255), 10);
            // imshow("R roi", R_roi);
            // imshow("rune detector debug", rune_detector_debug);
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