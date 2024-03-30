//
// Created by root on 23-2-18.
//

#ifndef SR_SDUST_ADAPTIVEEKF_HPP
#define SR_SDUST_ADAPTIVEEKF_HPP
#include "supervisor.h"
#include <ceres/jet.h>

template<int X_n, int Y_n>// normal 5 or 3
class  AdaptiveEKF {
#define ChiSquaredTest_EKF_THRESH 5
public:
    using MatrixXX = Eigen::Matrix<double, X_n, X_n>;
    using MatrixXY = Eigen::Matrix<double, X_n, Y_n>;
    using MatrixYX = Eigen::Matrix<double, Y_n, X_n>;
    using MatrixYY = Eigen::Matrix<double, Y_n, Y_n>;
    using VectorX = Eigen::Matrix<double, X_n, 1>;
    using VectorY = Eigen::Matrix<double, Y_n, 1>;

    VectorX Xe;     // 估计状态变量
    VectorX Xp;     // 预测状态变量
    MatrixXX F;     // 预测雅克比
    MatrixYX H;     // 观测雅克比
    MatrixXX P;     // 状态协方差
    MatrixXX Q;     // 预测过程协方差
    MatrixYY R;     // 观测过程协方差
    MatrixXY K;     // 卡尔曼增益
    VectorY Yp;     // 预测观测量

    MatrixYY S;     // 残差矩阵 S
    MatrixYY M_matrix; // 矩阵M N 计算lambda
    MatrixYY N_matrix;
    VectorY Res; //残差向量

public:
    explicit AdaptiveEKF(const VectorX &X0 = VectorX::Zero()) : Xe(X0), P(MatrixXX::Identity()), Q(MatrixXX::Identity()), R(MatrixYY::Identity()) {}
    void init(const VectorX &X0 = VectorX::Zero()) { Xe = X0; }


    template<class Func>
    VectorX predict(Func &&func) {

        ceres::Jet<double, X_n> Xe_auto_jet[X_n];
        for (int i = 0; i < X_n; i++) {
            Xe_auto_jet[i].a = Xe[i];
            Xe_auto_jet[i].v[i] = 1;
        }

        ceres::Jet<double, X_n> Xp_auto_jet[X_n];
        func(Xe_auto_jet, Xp_auto_jet);
        for (int i = 0; i < X_n; i++) {
            Xp[i] = Xp_auto_jet[i].a;
            F.block(i, 0, 1, X_n) = Xp_auto_jet[i].v.transpose();
        }

        P = F * P * F.transpose() + Q;

        return Xp;
    }


    template<class Func>
    Eigen::Matrix<double, X_n, 1> update(Func &&func, const AdaptiveEKF::VectorY &Y) {

        ceres::Jet<double, X_n> Xp_auto_jet[X_n];
        for (int i = 0; i < X_n; i++) {
            Xp_auto_jet[i].a = Xp[i];
            Xp_auto_jet[i].v[i] = 1;
        }

        ceres::Jet<double, X_n> Yp_auto_jet[Y_n];
        func(Xp_auto_jet, Yp_auto_jet);
        for (int i = 0; i < Y_n; i++) {
            Yp[i] = Yp_auto_jet[i].a;
            H.block(i, 0, 1, X_n) = Yp_auto_jet[i].v.transpose();
        }

        K = P * H.transpose() * (H * P * H.transpose() + R).inverse(); //  calculate kalman gain

        Xe = Xp + K * (Y - Yp);

        P = (MatrixXX::Identity() - K * H) * P;

        return Xe;
    }

};




#endif //SR_SDUST_ADAPTIVEEKF_HPP
