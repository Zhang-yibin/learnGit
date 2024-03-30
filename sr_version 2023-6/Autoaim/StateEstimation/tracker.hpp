//
// Created by root on 23-4-15.
//

#ifndef SR_SDUST_TRACKER_HPP
#define SR_SDUST_TRACKER_HPP
#include "pnpsolver.h"
#include "extended_kalman_filter.hpp"
#include "supervisor.h"
#include "parameter.h"
#include "detector.h"
#include "classifier.h"
#include <Eigen/Eigen>
#include <sys/time.h>


enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

typedef struct target_info_position{
    Eigen::Vector3d position;
    double yaw;
}target_info_position;

class StateTracker
{
public:
    StateTracker(double max_match_distance, int tracking_threshold, int lost_threshold,double max_match_yaw_diff_);

    void updateArmorsNum(const Armor & armor);
    void init(vector<Armor> armors);

    void update(vector<Armor> armors);

    /// c:camera    w:odem    i:imu
    // camera 2 world
    Eigen::Vector3d pc_to_pw(const Eigen::Vector3d & pc, const Eigen::Matrix3d & R_IW){
        auto R_WC = (R_CI * R_IW).transpose();
        return R_WC * pc;
    }
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

    Eigen::Matrix3d get_R_IW(PlatState ps) {
//            Eigen::Matrix3d R_IW_x, R_IW_y, R_IW_z;
//            R_IW_x<< 1, 0, 0,
//                    0, cos(ps.pitch_angle* M_PI / 180.0), sin(ps.pitch_angle* M_PI / 180.0),
//                    0, -sin(ps.pitch_angle* M_PI / 180.0), cos(ps.pitch_angle* M_PI / 180.0);
//        R_IW_y<< cos(plat_state.yaw_angle* M_PI / 180.0), 0, -sin(plat_state.yaw_angle* M_PI / 180.0),
//                 0, 1, 0,
//                 sin(plat_state.yaw_angle* M_PI / 180.0), 0, cos(plat_state.yaw_angle* M_PI / 180.0);
//            R_IW_z << cos(ps.yaw_angle * M_PI / 180.0), sin(ps.yaw_angle * M_PI / 180.0), 0,
//                    -sin(ps.yaw_angle * M_PI / 180.0), cos(ps.yaw_angle * M_PI / 180.0), 0,
//                    0, 0, 1;
//            cerr<<ps.pitch_angle* M_PI / 180.0<<endl;
//            auto R_IW_x_ = R_IW_x.inverse();
//            auto R_IW_y_ = R_IW_y.inverse();
//            auto R_IW_z_ = R_IW_z.inverse();
//
//        auto R_IW_ = R_IW_y_ * R_IW_x_;
//            cout<<"R_IW_Z::::::::::::"<<R_IW_z_<<endl;
//            cout<<"R_IW_x::::::::::::"<<R_IW_x_<<endl;
//            Eigen::Matrix3d tmp_R_IW = R_IW_z_ * R_IW_x;
        Eigen::Vector3d eulerAngel(ps.yaw_angle*M_PI/180,ps.pitch_angle*M_PI/180,0);
        Eigen::Matrix3d I2W;
        I2W=Eigen::AngleAxisd(eulerAngel[0],Eigen::Vector3d::UnitZ())*
            Eigen::AngleAxisd(eulerAngel[1],Eigen::Vector3d::UnitY())*
            Eigen::AngleAxisd(eulerAngel[2],Eigen::Vector3d::UnitX());
        return I2W;
    }

    void Draw_3D(Armor armor, Mat coordinate_debug) {
        vector<Point3f> RealPoint;
        RealPoint.emplace_back(Point3f(0.1,0.1,0.1));
        RealPoint.emplace_back(Point3f(0.1,-0.1,0.1));
        RealPoint.emplace_back(Point3f(-0.1,-0.1,0.1));
        RealPoint.emplace_back(Point3f(-0.1,0.1,0.1));
        RealPoint.emplace_back(Point3f(0.1,0.1,-0.1));
        RealPoint.emplace_back(Point3f(0.1,-0.1,-0.1));
        RealPoint.emplace_back(Point3f(-0.1,-0.1,-0.1));
        RealPoint.emplace_back(Point3f(-0.1,0.1,-0.1));
//        RealPoint.emplace_back(Point3f(xc,yc,zc));
        RealPoint.emplace_back(Point3f(0,0,0));
//        RealPoint.emplace_back(Point3f(0.1,0.1,-0.1));
//        RealPoint.emplace_back(Point3f(0.1,0.1,-0.1));
//        RealPoint.emplace_back(Point3f(0.1,0.1,-0.1));

        vector<Point2f> ImagePoint;
        Mat camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(cameraMatrix.data())).clone());
        Mat dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(distCoeffs.data())).clone());
        projectPoints(RealPoint, armor.rvec_, armor.tvec_, camera_matrix_, dist_coeffs_, ImagePoint);

        line(coordinate_debug,ImagePoint[0],ImagePoint[1],Scalar(0,78,255),3);
//        line(armor_info,armors[0].center,ImagePont[1],Scalar(0,255,0),3);
//        line(armor_info,armors[0].center,ImagePoint[2],Scalar(255,0,0),3);
        line(coordinate_debug,ImagePoint[1],ImagePoint[2],Scalar(0,78,255),3);
        line(coordinate_debug,ImagePoint[2],ImagePoint[3],Scalar(0,78,255),3);
        line(coordinate_debug,ImagePoint[3],ImagePoint[0],Scalar(0,78,255),3);
        line(coordinate_debug,ImagePoint[4],ImagePoint[5],Scalar(220,0,255),3);
        line(coordinate_debug,ImagePoint[5],ImagePoint[6],Scalar(220,0,255),3);
        line(coordinate_debug,ImagePoint[6],ImagePoint[7],Scalar(220,0,255),3);
        line(coordinate_debug,ImagePoint[7],ImagePoint[4],Scalar(220,0,255),3);
        line(coordinate_debug,ImagePoint[0],ImagePoint[4],Scalar(0,0,255),3);
        line(coordinate_debug,ImagePoint[1],ImagePoint[5],Scalar(0,0,255),3);
        line(coordinate_debug,ImagePoint[2],ImagePoint[6],Scalar(0,0,255),3);
        line(coordinate_debug,ImagePoint[3],ImagePoint[7],Scalar(0,0,255),3);
//        line(armor_info,ImagePoint[8],ImagePoint[9],Scalar(224,44,138),5);
//        line(armor_info,ImagePoint[1],ImagePoint[8],Scalar(224,44,138),2);
//        line(armor_info,ImagePoint[2],ImagePoint[8],Scalar(224,44,138),2);
//        line(armor_info,ImagePoint[7],ImagePoint[8],Scalar(224,44,138),2);

        circle(coordinate_debug,ImagePoint[0],2,Scalar(0,0,255),2);
        circle(coordinate_debug,ImagePoint[1],2,Scalar(0,0,255),2);
        circle(coordinate_debug,ImagePoint[2],2,Scalar(0,0,255),2);
        circle(coordinate_debug,ImagePoint[3],2,Scalar(0,0,255),2);
        circle(coordinate_debug,ImagePoint[4],2,Scalar(0,255,255),2);
        circle(coordinate_debug,ImagePoint[5],2,Scalar(0,255,255),2);
        circle(coordinate_debug,ImagePoint[6],2,Scalar(0,255,255),2);
        circle(coordinate_debug,ImagePoint[7],2,Scalar(0,255,255),2);
        circle(coordinate_debug,ImagePoint[8],3,Scalar(224,44,138),3);
//            imshow("coordinate_debug",coordinate_debug);
    }

    void Load_EKF() {
        // EKF
        // xa = x_armor, xc = x_robot_center
        // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
        // measurement: xa, ya, za, yaw
        // f - Process function
        auto f = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_;
            x_new(2) += x(3) * dt_;
            x_new(4) += x(5) * dt_;
            x_new(6) += x(7) * dt_;
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_f = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(9, 9);
            // clang-format off
            f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
                    0,   1,   0,   0,   0,   0,   0,   0,   0,
                    0,   0,   1,   dt_, 0,   0,   0,   0,   0,
                    0,   0,   0,   1,   0,   0,   0,   0,   0,
                    0,   0,   0,   0,   1,   dt_, 0,   0,   0,
                    0,   0,   0,   0,   0,   1,   0,   0,   0,
                    0,   0,   0,   0,   0,   0,   1,   dt_, 0,
                    0,   0,   0,   0,   0,   0,   0,   1,   0,
                    0,   0,   0,   0,   0,   0,   0,   0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto h = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_h = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 9);
            double yaw = x(6), r = x(8);
            // clang-format off
            //      xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
            h <<    1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
                    0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
                    0,   0,   0,   0,   1,   0,   0,          0,   0,
                    0,   0,   0,   0,   0,   0,   1,          0,   0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        s2qxyz_ = 0.05;
        s2qyaw_ = 1;
        s2qr_ = 8;
        auto u_q = [this]() {
            Eigen::MatrixXd q(9, 9);
            double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //      xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
            q <<    q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
                    q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
                    0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
                    0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
                    0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
                    0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
                    0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
                    0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
                    0,      0,      0,      0,      0,      0,      0,      0,      q_r;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        r_xyz_factor =4e-4;
        r_yaw = 5e-3;
        auto u_r = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 9> p0;
        p0.setIdentity();
        this->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
    }

    Eigen::Vector3d getTargetArmor(Eigen::VectorXd state);

    enum State {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } tracker_state;

    ExtendedKalmanFilter ekf;

    double dt_;
    double s2qxyz_, s2qyaw_, s2qr_;
    double r_xyz_factor, r_yaw;

    char tracked_id;
    Mat tracker_debug;
    Armor tracked_armor;
    ArmorsNum tracked_armors_num;

    target_info_position pre_position ;

    target_info_position pre_ArmorsOfState[4];

    Eigen::VectorXd target_state;
    Eigen::VectorXd measurement;
    Eigen::VectorXd pre_state;
    Eigen::Matrix3d R_IW;
    Eigen::Matrix3d cam_e_Matrix; // 相机内参
    Eigen::Matrix<double, 3, 3> R_CI; // imu 2 camera T matrix

    // To store another pair of armors
    double dz, another_r;

    double shoot_speed = 17, shoot_delay = 0.11;

    int lost_threshold_;
    int tracking_threshold_;
    double max_match_distance_;
    double max_match_yaw_diff_;


private:
    void initEKF(const Armor & a);

    void handleArmorJump(const Armor & a);

    double RotationToYaw(double yaw);

    Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

    int lost_count_;
    int detect_count_;

    double last_yaw_;
};



#endif //SR_SDUST_TRACKER_HPP
