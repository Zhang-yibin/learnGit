//
// Created by root on 23-3-2.
//

#ifndef SR_SDUST_EKF_PREDICTOR_H
#define SR_SDUST_EKF_PREDICTOR_H

#include "detector.h"
#include "classifier.h"
#include "pnpsolver.h"
#include "supervisor.h"
#include "parameter.h"
#include "AdaptiveEKF.hpp"
//#include "../Thread/thread.h"
#include <sys/time.h>


struct Predict {
    ///Uniform linear motion model
    template<class T>
    void operator()(const T x0[5], T x1[5]) {
        x1[0] = x0[0] + delta_t * x0[1];  //0.1
        x1[1] = x0[1];  //100
        x1[2] = x0[2] + delta_t * x0[3];  //0.1
        x1[3] = x0[3];  //100
        x1[4] = x0[4];  //0.01
    }
    double delta_t;
};

template<class T>
void xyz2pyd(T xyz[3], T pyd[3]) {
    /*
     * make xyz to pitch、yaw、distance
     */
    pyd[0] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]));  // pitch
    pyd[1] = ceres::atan2(xyz[1], xyz[0]);  // yaw
    pyd[2] = ceres::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);  // distance
}

struct Measure {
    /*
     * 工具函数
     */
    template<class T>
    void operator()(const T x[5], T y[3]) {
        T x_[3] = {x[0], x[2], x[4]};
        xyz2pyd(x_, y);
    }
};

class EKF_Predictor {

private:
    // 时间处理
    struct LastTimeHelper {
        LastTimeHelper(double current_time, double &ref_time) : c_time(current_time), r_time(ref_time) {};

        ~LastTimeHelper() { r_time = c_time; }

        double c_time;
        double &r_time;
    };

    /// get now->pre time
    static void Get_Time(double &time)
    {
        struct timeval tv;
        gettimeofday(&tv,NULL);
        time = tv.tv_sec * 1000 + tv.tv_usec / 1000; // 获取当前系统时间戳
        time /= 1000;
    }
    /// c:camera    w:world    i:imu
    // camera 2 world
    Eigen::Vector3d pc_to_pw(const Eigen::Vector3d & pc, const Eigen::Matrix3d & R_IW){
        auto R_WC = (R_CI * R_IW).transpose();
        return R_WC * pc;
    }
    // world 2 camera
    // odem 2 camera
    Eigen::Vector3d pw_to_pc(const Eigen::Vector3d & pw, double yaw_angle,double pitch_angle){
        Eigen::Matrix3d R_,R_Z,R_Y,R_X;
        R_X<<1,0,0,
                0,-1,0,
                0,0,-1;
        auto d=R_X.transpose()*pw;
        R_Y<<    cos(pitch_angle* M_PI / 180.0),0,-sin(pitch_angle* M_PI / 180.0),
                0,1,0,
                sin(pitch_angle* M_PI / 180.0),0,cos(pitch_angle* M_PI / 180.0);
        auto c=R_Y.transpose()*d;
        R_Z<< cos(yaw_angle * M_PI / 180.0),sin(yaw_angle * M_PI / 180.0), 0,
                -sin(yaw_angle * M_PI / 180.0), cos(yaw_angle * M_PI / 180.0), 0,
                0, 0, 1;
        auto b=R_Z.transpose()*c;
        R_<<    0, 0, 1,
                1, 0, 0,
                0, 1, 0;
        auto a=R_.transpose()*b;
        return a;
    }
    Eigen::Vector3d pc_to_pu(const Eigen::Vector3d & pc) {
        return cam_e_Matrix * pc / pc(2, 0);
    }
    // 将世界坐标系内一点，投影到图像中，并绘制该点
    /// image  output Mat pw  R_IW  color
    void re_project_point(cv::Mat &image, const Eigen::Vector3d &pw,
                          double yaw_angle,double pitch_angle, const cv::Scalar &color, double radius = 6) {
//            Eigen::Vector3d pc = pw_to_pc(pw, R_IW);
        Eigen::Matrix3d R_,R_Z,R_Y,R_X;
        R_X<<   1, 0, 0,
                0,-1, 0,
                0, 0,-1;
        auto d = R_X.transpose() * pw;
        R_Z<<   cos(yaw_angle * M_PI / 180.0), sin(yaw_angle * M_PI / 180.0), 0,
                -sin(yaw_angle * M_PI / 180.0), cos(yaw_angle * M_PI / 180.0), 0,
                0, 0, 1;
        auto b = R_Z.transpose() * d;
        R_Y<<   cos(pitch_angle* M_PI / 180.0), 0, -sin(pitch_angle* M_PI / 180.0),
                0, 1, 0,
                sin(pitch_angle* M_PI / 180.0), 0, cos(pitch_angle* M_PI / 180.0);
        auto c = R_Y.transpose() * b;


        R_<<    0, 0, 1,
                1, 0, 0,
                0, 1, 0;
        auto a = R_.transpose() * c;

        Eigen::Vector3d pu = pc_to_pu(a);
        cv::circle(image, {int(pu(0, 0)), int(pu(1, 0))}, radius, color, 2);
    }


//    Eigen::Matrix3d Set_eularAngles2RotationMatrix(const PlatState &plat_state_) {
//        Eigen::Matrix3d R_z, R_y, R_x, R;
//
//        R_x <<  1, 0, 0,
//                0, 1, 0,
//                0, 0, 1;
//
//        R_y <<  std::cos(plat_state_.pitch_angle), 0, std::sin(plat_state_.pitch_angle),
//                0, 1, 0,
//                -std::sin(plat_state_.pitch_angle), 0, std::cos(plat_state_.pitch_angle);
//
//        R_z <<  std::cos(plat_state_.yaw_angle), -std::sin(plat_state_.yaw_angle), 0,
//                std::sin(plat_state_.yaw_angle), std::cos(plat_state_.yaw_angle), 0,
//                0, 0, 1;
//
//        R = R_z * R_y * R_x;
//        return R;
//    }

    inline void Load_Param()
    {
        // 设置对角线的值
        // 预测过程协方差
        ekf_.Q(0, 0) = Q00;
        ekf_.Q(1, 1) = Q11;
        ekf_.Q(2, 2) = Q22;
        ekf_.Q(3, 3) = Q33;
        ekf_.Q(4, 4) = Q44;
        // 观测过程协方差
        ekf_.R(0, 0) = R00;
        ekf_.R(1, 1) = R11;
        ekf_.R(2, 2) = R22;

        Mat camera_Matrix = cv::Mat(3, 3, CV_64F, const_cast<double *>(cameraMatrix.data())).clone();
        cv::cv2eigen(camera_Matrix, cam_e_Matrix);
        T_CI(0, 0) = 0;
        T_CI(1, 0) = -0.09;
        T_CI(2, 0) = 0.055;
    }

public:

    using EKF = AdaptiveEKF<5, 3>;
    EKF ekf_;
    Eigen::Vector3d point_camera;
    bool need_init = true;
    double m_yaw, m_pitch;

    enum State {
        LOST = 0,
        DETECTING = 1,
        TRACKING = 2,
        TEMP_LOST = 3,
    };

    struct IDFilter{
        char last_id, cur_id;
        int lost_cnt = 0;
    }id_filter;


private:
    Eigen::Vector3d last_m_pw;
    double last_m_yaw, last_m_pitch;
    double last_time = 0.;
    double t = 0.;
    double shoot_delay = 0.03;  // shoot delay time  s
    Eigen::Matrix<double, 3, 3> R_CI; // imu 2 camera T matrix
public:
    PlatState plat_state;

    double distance = 0.; // distance (单位：m)
    double shoot_speed = 15.0; // speed (单位：m/s)
    Euler now_euler;
    Eigen::Matrix3d cam_e_Matrix; // 相机内参

    Euler predict_euler;
    State tracking_state;
    Eigen::Vector3d T_CI;

    EKF_Predictor();
    bool predict(const Armor & armor, Mat image, double delta_t);
    ~EKF_Predictor() = default;
};


#endif //SR_SDUST_EKF_PREDICTOR_H
