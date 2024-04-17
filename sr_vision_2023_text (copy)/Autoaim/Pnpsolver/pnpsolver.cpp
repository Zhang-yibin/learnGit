//
// Created by eski on 6/30/22.
//

#include "pnpsolver.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <fmt/format.h>
#include <fmt/color.h>
PnPSolver::PnPSolver(
        const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
        : camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
          dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())

{
    // Unit: m
    constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
    constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
    constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

    // Start from bottom left in clockwise order
    // Model coordinate: x forward, y left, z up
    small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
    small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
    small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
    small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

    large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
    large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
    large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
    large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}

bool PnPSolver::solvePnP(Armor & armor, Point3f & point, Mat & rvec, Mat & tvec)
{

    R_WtoC<<0.0,    0.0,   1.0,
            -1.0,   0.0,   0.0,
            0.0,   -1.0,   0.0;
    std::vector<cv::Point2f> image_armor_points;
    // Fill in image points
    image_armor_points.emplace_back(armor.left_light.bottom);
    image_armor_points.emplace_back(armor.left_light.top);
    image_armor_points.emplace_back(armor.right_light.top);
    image_armor_points.emplace_back(armor.right_light.bottom);

    // Solve pnp

    auto object_points = armor.armor_type == SMALL ? small_armor_points_ : large_armor_points_;
    bool success = cv::solvePnP(
            object_points, image_armor_points, camera_matrix_, dist_coeffs_, rvec, tvec, false,
            cv::SOLVEPNP_IPPE);
    if (success) {
//        cout<<endl<<endl<<endl<<rvec<<endl;
        Mat zxc = Mat::zeros(1000,1000,CV_8UC3);
//        putText(zxc,"1",image_armor_points[0],1,1,Scalar(255,255,255));
//        putText(zxc,"2",image_armor_points[1],1,1,Scalar(255,255,255));
//        putText(zxc,"3",image_armor_points[2],1,1,Scalar(255,255,255));
//        putText(zxc,"4",image_armor_points[3],1,1,Scalar(255,255,255));

        putText(zxc,to_string(sqrt((image_armor_points[0].x*image_armor_points[0].x-image_armor_points[1].x*image_armor_points[1].x) +
        (image_armor_points[0].y*image_armor_points[0].y-image_armor_points[1].y*image_armor_points[1].y))),image_armor_points[0],1,1,Scalar(255,255,255));

        putText(zxc,to_string(sqrt((image_armor_points[3].x*image_armor_points[3].x-image_armor_points[2].x*image_armor_points[2].x) +
        (image_armor_points[3].y*image_armor_points[3].y-image_armor_points[2].y*image_armor_points[2].y))),image_armor_points[3]+Point2f(50,50),1,1,Scalar(255,255,255));


//        r_vec=rvec;
//        t_vec=tvec;
        armor.tvec_=tvec;
        armor.rvec_=rvec;
        // Convert to geometry_msgs::msg::Point
        point.x = tvec.at<double>(0) * 0.001;
        point.y = tvec.at<double>(1) * 0.001;
        point.z = tvec.at<double>(2) * 0.001;


//        cout<<xyzz<<endl;
        cv::cv2eigen(tvec, armor.camera_pos);
//        armor.camera_pos[0]
        xyzz[1]-=0.065;
        xyzz[2]-=0.09;
//        armor.camera_pos=RRR * armor.camera_pos;
//        Mat a=Mat(480,640,CV_8UC3,Scalar(0,0,0));
        Mat matrix_rvec;
        cv::Rodrigues(rvec, matrix_rvec);
        Eigen::Matrix<double, 3, 3> tmp_rotation_Matrix;
        cv::cv2eigen(matrix_rvec,tmp_rotation_Matrix);
//        armor.pose.plat_state.pitch_angle=0;
//        armor.pose.plat_state.yaw_angle=0;
        Eigen::Vector3d t_;
        cv::cv2eigen(tvec,t_);
        Eigen::Matrix3d R_,R_Z,R_Y,R_X;
        R_<<0,0,1,
            1,0,0,
            0,1,0;
        tmp_rotation_Matrix=R_*tmp_rotation_Matrix;

        t_=R_*t_;
        R_Y<<    cos(armor.pose.plat_state.pitch_angle* M_PI / 180.0),0,-sin(armor.pose.plat_state.pitch_angle* M_PI / 180.0),
                0,1,0,
                sin(armor.pose.plat_state.pitch_angle* M_PI / 180.0),0,cos(armor.pose.plat_state.pitch_angle* M_PI / 180.0);
        tmp_rotation_Matrix=R_Y*tmp_rotation_Matrix;
        t_=R_Y*t_;
        R_Z<< cos(armor.pose.plat_state.yaw_angle * M_PI / 180.0),sin(armor.pose.plat_state.yaw_angle * M_PI / 180.0), 0,
                -sin(armor.pose.plat_state.yaw_angle * M_PI / 180.0), cos(armor.pose.plat_state.yaw_angle * M_PI / 180.0), 0,
                0, 0, 1;
        tmp_rotation_Matrix=R_Z*tmp_rotation_Matrix;
        t_=R_Z*t_;

        R_X<<1,0,0,
            0,-1,0,
            0,0,-1;
        tmp_rotation_Matrix=R_X*tmp_rotation_Matrix;
        t_=R_X*t_;
//        tmp_rotation_Matrix = R_WtoC * tmp_rotation_Matrix;
//        armor.pose.plat_state.yaw_angle=50;
//        armor.pose.plat_state.pitch_angle=0;
//        Eigen::Vector3d eulerAngel(armor.pose.plat_state.yaw_angle*M_PI/180,armor.pose.plat_state.pitch_angle*M_PI/180,0);
//        Eigen::Matrix3d I2W;
//        I2W = Eigen::AngleAxisd(eulerAngel[0],Eigen::Vector3d::UnitZ()) *
//              Eigen::AngleAxisd(eulerAngel[1],Eigen::Vector3d::UnitY()) *
//              Eigen::AngleAxisd(eulerAngel[2],Eigen::Vector3d::UnitX());


//        Eigen::Vector3d t_;
//        cv::cv2eigen(tvec,t_);
//        t_ = R_WtoC * t_;
//        t_ = I2W * t_;
        double yaw, pitch, roll;
//        auto ini=I2W.inverse()*tmp_rotation_Matrix;
//        cout<<"ini"<<endl<<ini<<endl;
//        getEulerYPR(tmp_rotation_Matrix,yaw,pitch,roll);
//        cout<<yaw*180/CV_PI<<endl;
//        tmp_rotation_Matrix = tmp_rotation_Matrix * I2W;
//        cout<<"qwq"<<endl<<tmp_rotation_Matrix<<endl;

        getEulerYPR(tmp_rotation_Matrix,yaw,pitch,roll);
//        cout<<"yaw"<<yaw*180/CV_PI<<endl;
        Mat final = Mat::zeros(900, 900, CV_8UC3);
        auto a=t_(0);
        auto b=t_(1);
        auto c=t_(2);
        putText(final,'x'+to_string(a),Point2f(10,100),2,1,Scalar(0,255,255));
        putText(final,'y'+to_string(b),Point2f(10,300),2,1,Scalar(0,255,255));
        putText(final,'z'+to_string(c),Point2f(10,500),2,1,Scalar(0,255,255));
        putText(final,'x'+to_string(tvec.at<double>(0)),Point2f(500,100),2,2,Scalar(0,255,255));
        putText(final,'y'+to_string(tvec.at<double>(1)),Point2f(500,300),2,2,Scalar(0,255,255));
        putText(final,'z'+to_string(tvec.at<double>(2)),Point2f(500,500),2,2,Scalar(0,255,255));
        putText(final,"yaw"+to_string(yaw),Point2f(300,500),2,1,Scalar(0,255,255));
        //        imshow("zxc",zxc);
//        imshow("xyz",final);
        armor.distance_to_image_center = calculateDistanceToCenter(armor.center);
//        Eigen::Vector3d m_pc = std::move(armor.camera_pos);
        armor.pose.pw = t_;
        armor.pose.yaw = yaw;
        return true;
    } else {
        return false;
    }
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
    float cx = camera_matrix_.at<double>(0, 2);
    float cy = camera_matrix_.at<double>(1, 2);
    return cv::norm(image_point - cv::Point2f(cx, cy));
}

void PnPSolver::getEulerYPR(Eigen::Matrix3d m_el,double &yaw,double &pitch,double &roll){
    struct Euler{
        double yaw;
        double pitch;
        double roll;
    };
    Euler euler_out{};
    if(fabs(m_el(2,0))>=1){
        euler_out.yaw=0;
        double delta=atan2(m_el(2,1),m_el(2,2));
        if(m_el(2,0)<0){
            euler_out.pitch=M_PI/2;
            euler_out.roll=delta;
        }else{
            euler_out.pitch=-M_PI/2;
            euler_out.roll=delta;
        }
    }else{
        euler_out.pitch=-Asin(m_el(2,0));
        euler_out.roll=atan2(m_el(2,1)/cos(euler_out.pitch),m_el(2,2)/cos(euler_out.pitch));
        euler_out.yaw=atan2(m_el(1,0)/cos(euler_out.pitch),m_el(0,0)/cos(euler_out.pitch));
    }
    yaw=euler_out.yaw;
    pitch=euler_out.pitch;
    roll=euler_out.roll;
}
