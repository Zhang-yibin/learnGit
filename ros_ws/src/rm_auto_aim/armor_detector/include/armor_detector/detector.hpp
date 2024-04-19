// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim
{

#define changeTo2f(x) std::to_string(int(x)) + "." + std::to_string(int(x * 100 + 0.5) % 100)

class Detector
{
public:
  struct LightParams
  {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
  };

  struct ArmorParams
  {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a);

  std::vector<Armor> detect(const cv::Mat & input);

  cv::Mat preprocessImage(const cv::Mat & input);
  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  // For debug usage
  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat & img);

  int binary_thres;
  int detect_color;
  LightParams l;
  ArmorParams a;

  std::unique_ptr<NumberClassifier> classifier;

  // Debug msgs
  cv::Mat binary_img;
  auto_aim_interfaces::msg::DebugLights debug_lights;
  auto_aim_interfaces::msg::DebugArmors debug_armors;

private:
  bool isLight(const Light & possible_light);
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
  ArmorType isArmor(const Light & light_1, const Light & light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

class ShootFan
{
public:
  ShootFan() = default;
  std::vector<cv::Point> fan_contours;
  cv::RotatedRect rrect;
  std::vector<cv::Point2f> longside_centers;
  cv::Point2f towards;  //右上角为原点，向右为x轴，向下为y轴
  cv::Point2f long_side;
  cv::Point2f target_center;
  int fan_cols;
  std::vector<cv::Point2f> armor_points;
  cv::Point2f fan_center;
  double fan_angle;
  ShootFan(std::vector<cv::Point> a, cv::RotatedRect r) : fan_contours(std::move(a)), rrect(r) {}
  cv::Point2f flow_far_from_center;
private:
};

class RuneDetector
{
public:
  struct RuneParam
  {
    double binary_threshold = 80;//why not 120?

    // Contour fill
    double min_filledContours_area = 300;
    double max_contourArea = 23000;
    double min_contourArea = 7000;

    //Select the fan
    double max_fan_area = 25000;
    double min_fan_area = 3000;
    double min_area_ratio = 1.5;
    double min_fan_ratio = 1.4;
    double max_fan_ratio = 2.5;

    //Find the flow
    double min_flow_area_ratio = 0.7;
    double max_flow_area = 3000;
    double min_flow_area = 500;
    double max_ratio = 5;
    double min_ratio = 2;

    // 60 300 120 240
    // size(5,5) 40 200
    double min_r_area = 200, max_r_area = 1000;
    // 0.4 3.2
    double min_r_ratio = 0.4, max_r_ratio = 5;
  };

public:
  
  cv::Point2f r_center;
  std::vector<cv::RotatedRect> armors;
  ShootFan final_fan;
  float fan_angle = 0;
  enum Color {
    self_BLUE,
    self_RED,
  } rune_color;
  bool findCenter(cv::Mat & src);

  RuneDetector(const RuneParam & r);
private:
  bool imageProcess(cv::Mat & src);
  std::vector<ShootFan> fillContour();
  bool fanSizer(std::vector<ShootFan> fans);
  static double PixelContour(cv::Mat & rot, double start, int height);

public:
  RuneParam rune_param;
  cv::Mat binary_img;
  cv::Mat filled_contour_img;
  std::vector<std::vector<cv::Point>> filled_contours;
  std::array<cv::Point2f, 4> sorted_pts;

  // Debug image
  cv::Mat rune_debug;
  cv::Mat rune_armor_lines;
  cv::Mat rune_detector_debug;
};

struct Rune
{
  cv::Point2f r_center;
  cv::Point2f pre_center;
  cv::RotatedRect rune_armor;
  
  double real_angle;
  float radian = 0;

  int cols;
  int rows;
 
};


}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_   
