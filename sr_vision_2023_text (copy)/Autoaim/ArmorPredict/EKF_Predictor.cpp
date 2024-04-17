//
// Created by root on 23-3-2.
//

#include "EKF_Predictor.h"
EKF_Predictor::EKF_Predictor() {
    R_CI << -1.0, 0, 0,
            0, 0, 1.0,
            0, 1.0, 0;
    Load_Param();
    cout << "Init EKF successfully !" << endl;
}
Eigen::Matrix<double, 5, 1> last_Xp;

bool EKF_Predictor::predict(const Armor & armor, Mat image, double t) {

/// 修改时间
//    Get_Time(t);
    LastTimeHelper helper(t, last_time);  // 自动更新上次时间
    double delta_t = t - last_time;

    if(id_filter.last_id != id_filter.cur_id || delta_t >= 0.07)
        need_init = true;

    Predict func; /// 前者 -> 后者
    Measure measure; /// xyz -> pyd
    if(tracking_state==TEMP_LOST){

    }
    if(!need_init) {
        Eigen::Vector3d m_pc = std::move(point_camera); //在相机坐标系下的真实坐标

        Eigen::Vector3d m_pw = armor.pose.pw;    //在世界坐标系下的坐标

        m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0));     // yaw的测量值，单位弧度
        m_pitch = std::atan2(m_pw(2, 0), sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0)));

//        putText(image,"m_yaw"+to_string(m_yaw),Point(500,260), 1, 1, Scalar(13, 245, 255), 1);
//        putText(image,"m_pitch"+to_string(m_pitch),Point(500,280), 1, 1, Scalar(13, 245, 255), 1);

        Eigen::Matrix<double, 5, 1> Xr;
        Xr << m_pw(0, 0), 0, m_pw(1, 0), 0, m_pw(2, 0);
        Eigen::Matrix<double, 3, 1> Yr;
        measure(Xr.data(), Yr.data());  // 转化成相机求坐标系 Yr
        func.delta_t = delta_t;         // 设置距离上次预测的时间
        ekf_.predict(func);             // 更新预测器，此时是预测值
        Eigen::Matrix<double, 5, 1> Xe = ekf_.update(measure, Yr);   // 更新滤波器，输入真实的球面坐标 Yr

        double predict_time = m_pw.norm() / 27 + shoot_delay;
        func.delta_t = predict_time; // set pre time

        Eigen::Matrix<double, 5, 1> Xp;
        func(Xe.data(), Xp.data());      // 使用匀速直线模型直接预测 Xp
        last_Xp=Xp;
        Eigen::Vector3d c_pw{Xe(0, 0), Xe(2, 0), Xe(4, 0)};
        Eigen::Vector3d p_pw{Xp(0, 0), Xp(2, 0), Xp(4, 0)};

        // 抬枪补偿
        distance = m_pw.norm();
        Eigen::Vector3d p_pc = pw_to_pc(p_pw, plat_state.yaw_angle, plat_state.pitch_angle);
        // Paint

        re_project_point(image, m_pw, plat_state.yaw_angle,plat_state.pitch_angle, {0, 255, 255});
//    re_project_point(image, c_pw, R_IW, {0, 255, 0});
        re_project_point(image, p_pw, plat_state.yaw_angle,plat_state.pitch_angle, {255, 255, 0});
        cv::circle(image, {image.cols / 2, image.rows / 2}, 3, {0, 255, 0});
//        imshow("image", image);

        last_m_pw = m_pw;
        last_m_yaw = m_yaw;
        last_m_pitch = m_pitch;

//        predict_euler.yaw = atan(p_pc(0, 0) / p_pc(2, 0)) * 180. / M_PI;
//        predict_euler.pitch = atan(p_pc(1, 0) / p_pc(2, 0)) * 180. / M_PI;
        predict_euler.distance = distance;

        predict_euler.yaw = atan2(p_pw(1, 0),p_pw(0,0)) * 180. / M_PI;
        predict_euler.pitch = -std::atan2(p_pw(2, 0), sqrt(p_pw(0, 0) * p_pw(0, 0) + p_pw(1, 0) * p_pw(1, 0))) * 180. / M_PI;


        putText(image,"vx:"+to_string(Xe(1)),Point(500,160), 1, 1, Scalar(13, 245, 255), 1);
        putText(image,"vy:"+to_string(Xe(3)),Point(500,180), 1, 1, Scalar(13, 245, 255), 1);
        putText(image,"z:"+to_string(Xe(4)),Point(500,200), 1, 1, Scalar(13, 245, 255), 1);

        putText(image,"yaw"+to_string(predict_euler.yaw),Point(500,100), 1, 1, Scalar(13, 245, 255), 1);
        putText(image,"pitch"+to_string(predict_euler.pitch),Point(500,120), 1, 1, Scalar(13, 245, 255), 1);
        putText(image,"distance"+to_string(predict_euler.distance),Point(500,140), 1, 1, Scalar(13, 245, 255), 1);

//        putText(image,"x"+to_string(s_pc(0)),Point(500,200), 1, 1, Scalar(13, 245, 255), 1);
//        putText(image,"y"+to_string(s_pc(1)),Point(500,220), 1, 1, Scalar(13, 245, 255), 1);
//        putText(image,"z"+to_string(s_pc(2)),Point(500,240), 1, 1, Scalar(13, 245, 255), 1);

    }

    if(need_init) {
        Eigen::Vector3d m_pc = std::move(point_camera); //在相机坐标系下的真实坐标
        Eigen::Vector3d m_pw = armor.pose.pw;    //在世界坐标系下的坐标

        m_yaw = std::atan2(m_pw(1, 0), m_pw(0, 0));     // yaw的测量值，单位弧度
        m_pitch = std::atan2(m_pw(2, 0), sqrt(m_pw(0, 0) * m_pw(0, 0) + m_pw(1, 0) * m_pw(1, 0)));

        Eigen::Matrix<double, 5, 1> Xr;
        Xr << m_pw(0, 0), 0, m_pw(1, 0), 0, m_pw(2, 0);

        ekf_.init(Xr);

        Eigen::Matrix<double, 3, 1> Yr;
        measure(Xr.data(), Yr.data());  // 转化成相机求坐标系 Yr
        func.delta_t = delta_t;         // 设置距离上次预测的时间
        ekf_.predict(func);             // 更新预测器，此时是预测值
        Eigen::Matrix<double, 5, 1> Xe = ekf_.update(measure, Yr);   // 更新滤波器，输入真实的球面坐标 Yr

        double predict_time = m_pw.norm() / shoot_speed + shoot_delay;
        func.delta_t = predict_time; // set pre time

        Eigen::Matrix<double, 5, 1> Xp;
        func(Xe.data(), Xp.data());      // 使用匀速直线模型直接预测 Xp
        Eigen::Vector3d c_pw{Xe(0, 0), Xe(2, 0), Xe(4, 0)};
        Eigen::Vector3d p_pw{Xp(0, 0), Xp(2, 0), Xp(4, 0)};

        // 抬枪补偿
        distance = m_pw.norm();
        Eigen::Vector3d p_pc = pw_to_pc(p_pw, plat_state.yaw_angle, plat_state.pitch_angle);


        re_project_point(image, m_pw, plat_state.yaw_angle,plat_state.pitch_angle, {0, 255, 255});
//    re_project_point(image, c_pw, R_IW, {0, 255, 0});
//        re_project_point(image, p_pw, R_IW, {255, 0, 0});
        cv::circle(image, {image.cols / 2, image.rows / 2}, 3, {0, 255, 0});
//        imshow("image", image);

        need_init = false;

        predict_euler.yaw = atan2(p_pw(1, 0),p_pw(0,0)) * 180. / M_PI;
        predict_euler.pitch = atan2(p_pw(0, 0),p_pw(2,0)) * 180. / M_PI;

//        predict_euler.yaw = atan(p_pc(0, 0) / p_pc(2, 0)) * 180. / M_PI;
//        predict_euler.pitch = atan(p_pc(1, 0) / p_pc(2, 0)) * 180. / M_PI;
        predict_euler.distance = distance;
    }

////// 输出信息
//    cout << "distance : " << distance << endl;
//    cout << "height : " << height << endl;
//    cout << "ekf_.Q" << ekf_.Q << endl;
//    cout << "ekf_.R" << ekf_.R << endl;
//    cout  << "Xr : " << Xr << endl;
//    cout << "Yr(pyd) : " << Yr << endl;
//    cout << "predict time : " << predict_time << endl;
//    cout << "real point in camera coordinate system : " << endl << point_camera << endl;
//    cout << "m_pw : " << m_pw << endl;
//    cout << "p_pw : " << p_pw << endl;
//    cout << "s_pw : " << s_pw << endl;
//    cout << "c_pw : " << c_pw << endl << endl;
//    cout << "t : " << t << "   last_time : " << last_time << endl;
//    cout << "delta_t : " << func.delta_t << endl << endl;

    return true;
}


