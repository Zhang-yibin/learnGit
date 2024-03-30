//
// Created by eski on 6/29/22.
//

#ifndef SR_SDUST_DETECTOR_H
#define SR_SDUST_DETECTOR_H

#include "parameter.h"

struct PlatState
{
    float yaw_angle;
    float pitch_angle;
};

struct Pose {
//  Record
    Eigen::Vector3d pc;                                         // Position in Camera_coordinate
    Eigen::Vector3d pw;                                         // Position in World_coordinate
//    Eigen::Quaternion<double> quaternion_in_world;              // Making Quaterion in world
    Eigen::Matrix<double, 3, 3> rotation_Matrix;                // Calculated for center and yaw of rvec
    PlatState plat_state;                                       // Pitch angle and Yaw angle of gimbal  !!!!!
    double yaw;
//  Process
    Eigen::Matrix3d R_IW;
    Eigen::Matrix3d M;
//  Function
};

struct Light : public cv::RotatedRect
{
    Light() = default;
    explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
    {
        cv::Point2f p[4];
        box.points(p);
        std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
        top = (p[0] + p[1]) / 2;
        bottom = (p[2] + p[3]) / 2;

        length = cv::norm(top - bottom);
        width = cv::norm(p[0] - p[1]);

        tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
        tilt_angle = tilt_angle / CV_PI * 180;
    }

    int color;
    cv::Point2f top, bottom;
    double length;
    double width;
    float tilt_angle;
};

struct Armor
{
    Armor() = default;
    Armor(const Light & l1, const Light & l2)
    {
        if (l1.center.x < l2.center.x) {
            left_light = l1, right_light = l2;
        } else {
            left_light = l2, right_light = l1;
        }
        center = (left_light.center + right_light.center) / 2;
    }

    Light left_light, right_light;
    cv::Point2f center;
    cv::Mat number_img;
    Rect armor_rect;
    char number;
    float confidence;
    Eigen::Vector3d camera_pos;
    std::string classification_result;
    ArmorType armor_type;
    Point2f pt[4];
    double distance_to_image_center;
    Pose pose;
    Mat rvec_, tvec_;

};


class Detector {
public:
    Mat debug;
    Mat light_debug;
    Mat armor_debug;
    int detect_color;

    LightParams light_param;
    ArmorParams armor_param;
    Point2f relative_coord;

    Mat armorSrc;
    Rect track_rect;
    int track_flag = 0;

    Eigen::Vector3d camera_pos;



    int more_strict_classification = 1;

    cv::Mat preprocessImage(const cv::Mat & rbg_img);
    std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);
    std::vector<Armor> matchLights(const std::vector<Light> & lights);
    std::vector<Armor> trackArmor(Mat &img,vector<Armor> &armors);

private:
    bool isLight(const Light & light);
    bool containLight(
            const Light & light_1, const Light & light_2, const std::vector<Light> & lights);
    ArmorType isArmor(const Light & light_1, const Light & light_2) const;
    bool isTracking(const Mat &img);
};



#endif //SR_SDUST_DETECTOR_H
