//
// Created by eski on 6/30/22.
//

#ifndef SR_SDUST_PNPSOLVER_H
#define SR_SDUST_PNPSOLVER_H

#include "supervisor.h"
#include "detector.h"

struct Euler{
    float yaw;
    float pitch;
    float distance;
    float world_yaw;

    void toEuler(Point3f pt3f){
        yaw = static_cast<float >(std::atan2(pt3f.x, pt3f.z) / CV_PI * 180);
        pitch = static_cast<float >(std::atan2(pt3f.y, pt3f.z) / CV_PI * 180);
        distance = pt3f.z;
    }

    void calWorldAngle(float plat_yaw){
        float world_angle;
        world_angle = plat_yaw - (yaw - 90);
        if(world_angle > 360) world_angle -= 360.0;
        else if(world_angle < 0) world_angle += 360.0;
        world_yaw = world_angle;
    }
};

class PnPSolver
{
public:
    PnPSolver(
//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]
            const std::array<double, 9> & camera_matrix,
            const std::vector<double> & distortion_coefficients);

    // Get 3d position
    bool solvePnP(Armor & armor, Point3f & point,Mat &,Mat &);
    // Calculate the distance between armor center and image center
    float calculateDistanceToCenter(const cv::Point2f & image_point);
    Eigen::Vector3d tvec_eigen;//eigen_pnpsolver
    Eigen::Vector3d xyzz;
    Eigen::Matrix<double, 3, 3> R_WtoC;
    void getEulerYPR(Eigen::Matrix3d m_el,double &yaw,double &pitch,double &roll);
    double Asin(double x) {
        if(x < -1.0) x = -1.0;
        if(x > 1.0) x = 1.0;
        return asin(x);
    };

private:
//    Eigen::Matrix<double, 3, 3> R_WtoC;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // Unit: m
    static constexpr float SMALL_ARMOR_WIDTH = 135;
    static constexpr float SMALL_ARMOR_HEIGHT = 55;
    static constexpr float LARGE_ARMOR_WIDTH = 225;
    static constexpr float LARGE_ARMOR_HEIGHT = 55;

    // Four vertices of armor in 3d
    std::vector<cv::Point3f> small_armor_points_;
    std::vector<cv::Point3f> large_armor_points_;



};

#endif //SR_SDUST_PNPSOLVER_H
