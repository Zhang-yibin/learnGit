//
// Created by root on 23-4-15.
//

#include "tracker.hpp"
#include <fmt/format.h>
#include <fmt/color.h>

StateTracker::StateTracker(double max_match_distance, int tracking_threshold, int lost_threshold, double max_match_yaw_diff)
        : tracker_state(LOST),
          tracked_id(' '),
          measurement(Eigen::VectorXd::Zero(4)),
          target_state(Eigen::VectorXd::Zero(9)),
          max_match_distance_(max_match_distance),
          tracking_threshold_(tracking_threshold),
          max_match_yaw_diff_(max_match_yaw_diff)
{
    R_CI << 0.0000000,   -1.0000000,   0.0000000,
            0.0000000,  0.0000000,   -1.0000000,
            1.0000000,   0.0000000,  0.0000000;
}

void StateTracker::init(vector<Armor> armors) {
    if(armors.empty())
        return;

    PnPSolver solver(cameraMatrix,distCoeffs);

    // Simply choose the armor that is closest to image center
    auto min_distance = DBL_MAX;
    tracked_armor = armors[0];

    // Find the closest armor becoming the chosen_armor
    for (const auto & armor : armors) {
        if (armor.distance_to_image_center < min_distance) {
            min_distance = armor.distance_to_image_center;
            tracked_armor = armor;
        }
    }
    // Init EKF with chosen_armor
    initEKF(tracked_armor);
    // Change tracked ID and State
    tracked_id = tracked_armor.number;
    tracker_state = DETECTING;
    // Update the number of armors
    updateArmorsNum(tracked_armor);
}

void StateTracker::update(vector<Armor> armors) {

    // Draw the 3D cube.0-------87
    Mat co = tracker_debug;
    Mat horizon =  Mat::zeros(900, 900, CV_8UC3);
//    Draw_3D(tracked_armor, co);

    // KF predict
    Eigen::VectorXd ekf_prediction = ekf.predict(); // Priori state
    bool matched = false;

    // Use KF prediction as default target state if no matched armor is found
    target_state = ekf_prediction;
    if (!armors.empty()) {
        Armor same_id_armor;
        int same_id_armors_count = 0;
        auto predicted_position = getArmorPositionFromState(ekf_prediction);
        double min_position_diff = DBL_MAX;
        double yaw_diff = DBL_MAX;
        for (const auto & armor : armors) {
//            auto p = armor.pose.pw;
//            Eigen::Vector3d position_vec(p[0], p[1], p[2]);
//            // Difference of the current armor position and tracked armor's predicted position
//            double position_diff = (predicted_position - position_vec).norm();
//            if (position_diff < min_position_diff) {
//                min_position_diff = position_diff;
//                tracked_armor = armor;
//            }
            // Only consider armors with the same id
            if (armor.number == tracked_id) {
                same_id_armor = armor;
                same_id_armors_count++;
                // Calculate the difference between the predicted position and the current armor position
                auto p = armor.pose.pw;
                Eigen::Vector3d position_vec(p[0], p[1], p[2]);
                double position_diff = (predicted_position - position_vec).norm();
                if (position_diff < min_position_diff) {
                    // Find the closest armor
                    min_position_diff = position_diff;
                    yaw_diff = abs(RotationToYaw(armor.pose.yaw) - ekf_prediction(6));
//                    fmt::print(fmt::fg(fmt::color::yellow), "armor_jump {}!!!\n", yaw_diff);
                    tracked_armor = armor;
                }
            }
        }

        if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_)  {
            // Matching armor found
            matched = true;
            auto p = tracked_armor.pose.pw;
            // Update EKF
            double measured_yaw = RotationToYaw(tracked_armor.pose.yaw);

//            ofstream location_out;
//            location_out.open("/home/j11218cpu/Downloads/345678/5_21/Files/yaw_.txt", std::ios::out | std::ios::app);
//            location_out << to_string(measured_yaw) << endl;
//            location_out.close();
//
//            location_out.open("/home/j11218cpu/Downloads/345678/5_21/Files/x.txt", std::ios::out | std::ios::app);
//            location_out << to_string(p[0]) << endl;
//            location_out.close();

//            double measured_yaw =tracked_armor.pose.yaw;
            measurement = Eigen::Vector4d(p[0], p[1], p[2], measured_yaw);
            target_state = ekf.update(measurement); // Posteriori state
            fmt::print(fmt::fg(fmt::color::gray), "EKF update!!\n");
        } else if (same_id_armors_count == 1 && yaw_diff > max_match_yaw_diff_) {
            // Matched armor not found, but there is only one armor with the same id
            // and yaw has jumped, take this case as the target is spinning and armor jumped
            handleArmorJump(same_id_armor);
            fmt::print(fmt::fg(fmt::color::yellow), "armor_jump!!!\n");
        } else {
            // No matched armor found
            fmt::print(fmt::fg(fmt::color::green_yellow), "No matched armor found!");
        }
    }
//     Prevent radius from spreading
    if (target_state(8) < 0.12) {
        target_state(8) = 0.12;
        ekf.setState(target_state);
    } else if (target_state(8) > 0.4) {
        target_state(8) = 0.40;
        ekf.setState(target_state);
    }
    // Tracking state machine
    if (tracker_state == DETECTING) {
        if (matched) {
            detect_count_++;
            if (detect_count_ > tracking_threshold_) {
                detect_count_ = 0;
                tracker_state = TRACKING;
            }
        } else {
            detect_count_ = 0;
            tracker_state = LOST;
        }
    } else if (tracker_state == TRACKING) {
        if (!matched) {
            tracker_state = TEMP_LOST;
            lost_count_++;
        }
    } else if (tracker_state == TEMP_LOST) {
        if (!matched) {
            lost_count_++;
            if (lost_count_ > lost_threshold_) {
                lost_count_ = 0;
                tracker_state = LOST;
            }
        } else {
            tracker_state = TRACKING;
            lost_count_ = 0;
        }
    }

    // Calculate predict position.
    pre_state = target_state;
    Eigen::Vector3d pre_armor = getArmorPositionFromState(pre_state);
    double predict_time = pre_armor.norm() / 17 + 0.05;
    pre_state(0) += pre_state(1) * predict_time;
    pre_state(2) += pre_state(3) * predict_time;
    pre_state(4) += pre_state(5) * predict_time;
    pre_state(6) += pre_state(7) * predict_time;
    pre_position.position = getArmorPositionFromState(pre_state);
    pre_position.yaw=pre_state(6);
//    cout<<"weww"<<pre_position.yaw<<endl;
//    auto tmp_state = pre_state;
//    for(int i = 0; i < 4; i++) {
//        tmp_state(6) += i * (2 * M_PI / 4);
//        pre_ArmorsOfState[i].position = getArmorPositionFromState(tmp_state);
//        pre_ArmorsOfState[i].yaw=tmp_state(6);
////        cout << i << "          " << pre_ArmorsOfState[i] <<  endl;
//    }
    // Draw the target state
#ifdef SHOW_TARGET_STATE
    bool is_current_pair = true;
    auto a_n = static_cast<int>(tracked_armors_num);
    Eigen::Vector3d p_a;
    double r = 0;
    double yaw = target_state(6), r1 = target_state(8), r2 = another_r;
    double xc = target_state(0), yc = target_state(2), za = target_state(4);
    double vx = target_state(1), vy = target_state(3), vz = target_state(5);
    for (size_t i = 0; i < 4; i++) {
        double tmp_yaw = yaw + i * (2 * M_PI / a_n);
        // Only 4 armors has 2 radius and height
        if (a_n == 4) {
            r = is_current_pair ? r1 : r2;
            p_a[2] = za + (is_current_pair ? 0 : dz);
            is_current_pair = !is_current_pair;
        } else {
            r = r1;
            p_a[2] = za;
        }
        p_a[0] = xc - r * cos(tmp_yaw);
        p_a[1] = yc - r * sin(tmp_yaw);

#ifdef SHOW_HORIZON
        cv::circle(horizon, {900 - static_cast<int>(p_a.y() * 100 + 450), 900 - static_cast<int>(p_a.x() * 100 + 450)}, 5, Scalar(0,(75 * (i+1)) % 255,(50 * (i+1)) % 255));
        putText(horizon,to_string(i) ,{900 - static_cast<int>(p_a.y() * 100 + 450), 900 - static_cast<int>(p_a.x() * 100 + 450)},1,1,Scalar(0,(75 * (i+1)) % 255,255),1);
#endif
        re_project_point(tracker_debug, p_a, armors[0].pose.plat_state.yaw_angle,armors[0].pose.plat_state.pitch_angle, {124, 125, 125});
        is_current_pair = !is_current_pair;
    }
    Eigen::Vector3d center_pw = {xc, yc, za};
    re_project_point(tracker_debug, center_pw, armors[0].pose.plat_state.yaw_angle,armors[0].pose.plat_state.pitch_angle, {0, 0, 255});
#endif

//    re_project_point(tracker_debug, p_a, armors[0].pose.plat_state.yaw_angle,armors[0].pose.plat_state.pitch_angle, {0, 225, 125});

#ifdef SHOW_HORIZON
    circle(horizon, {900 - static_cast<int>(target_state(2) * 100 + 450), 900 - static_cast<int>(target_state(0) * 100 + 450)}, 5, Scalar(0, 255, 0));
//    imshow("horizon", horizon);
#endif

//    cout << target_state << endl;

//    re_project_point(tracker_debug, p_pw, armors[0].pose.plat_state.yaw_angle, armors[0].pose.plat_state.pitch_angle,{155, 155, 0}, 8);
#ifdef SHOW_TRACKER_DEBUG
    re_project_point(tracker_debug, tracked_armor.pose.pw, armors[0].pose.plat_state.yaw_angle,armors[0].pose.plat_state.pitch_angle, {0, 255, 255});
    cv::circle(tracker_debug, {tracker_debug.cols / 2, tracker_debug.rows / 2}, 3, {0, 255, 0});
//    imshow("tracker_debug", tracker_debug);
#endif
}

void StateTracker::initEKF(const Armor &a) {
    cout<<a.pose.pw;
    double xa = a.pose.pw[0];
    double ya = a.pose.pw[1];
    double za = a.pose.pw[2];
    last_yaw_ = 0;
    double yaw = RotationToYaw(a.pose.yaw);

    // Set initial position at 0.2m behind the target
    target_state = Eigen::VectorXd::Zero(9);
    double r = 0.26;
    double xc = xa + r * cos(yaw);
    double yc = ya + r * sin(yaw);
    dz = 0, another_r = r;
    target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

    ekf.setState(target_state);
}

void StateTracker::updateArmorsNum(const Armor & armor)
{
    if (armor.armor_type == LARGE && (tracked_id == '3' || tracked_id == '4' || tracked_id == '5')) {
        tracked_armors_num = ArmorsNum::BALANCE_2;
    } else if (tracked_id == 'O') {
        tracked_armors_num = ArmorsNum::OUTPOST_3;
    } else {
        tracked_armors_num = ArmorsNum::NORMAL_4;
    }
}


void StateTracker::handleArmorJump(const Armor &current_armor) {

    double yaw = RotationToYaw(current_armor.pose.yaw);
    target_state(6) = yaw;
    updateArmorsNum(current_armor);

//    cout << "current_p             " << current_p << endl;
//    cout << "infer_p               " << infer_p << endl;
//
//    Mat pp_debug = Mat::zeros(1080, 960, CV_8UC3);
//    putText(pp_debug,"cpx :"+ to_string(current_p.x()) ,Point(100,100),2,3,Scalar(0,255,255),2);
//    putText(pp_debug,"cpy  :"+ to_string(current_p.y()),Point(100,200),2,3,Scalar(0,255,255),2);
//    putText(pp_debug,"cpz  :"+ to_string(current_p.z()),Point(100,300),2,3,Scalar(0,255,255),2);
//    putText(pp_debug,"inx  :"+ to_string(infer_p.x()),Point(100,400),2,3,Scalar(0,255,255),2);
//    putText(pp_debug,"iny  :"+ to_string(infer_p.y()),Point(100,500),2,3,Scalar(0,255,255),2);
//    putText(pp_debug,"inz  :"+ to_string(infer_p.z()),Point(100,600),2,3,Scalar(0,255,255),2);
//    imshow("pp_debug", pp_debug);

    // Only 4 armors has 2 radius and height
    if (tracked_armors_num == ArmorsNum::NORMAL_4) {
        dz = target_state(4) - current_armor.pose.pw[2];
        target_state(4) = current_armor.pose.pw[2];
        std::swap(target_state(8), another_r);
    }
    fmt::print(fmt::fg(fmt::color::yellow), "armor_jump!!!\n");

    // If position difference is larger than max_match_distance_,
    // take this case as the ekf diverged, reset the state
    auto p = current_armor.pose.pw;
    Eigen::Vector3d current_p(p[0], p[1], p[2]);
    Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
    if ((current_p - infer_p).norm() > max_match_distance_) {
        double r = target_state(8);
        target_state(0) = p[0] + r * cos(yaw);  // xc
        target_state(1) = 0;                   // vxc
        target_state(2) = p[1] + r * sin(yaw);  // yc
        target_state(3) = 0;                   // vyc
        target_state(4) = p[2];                 // za
        target_state(5) = 0;                   // vza
        fmt::print(fmt::fg(fmt::color::yellow), "Reset State!!!\n");
    }

    ekf.setState(target_state);
}

Eigen::Vector3d StateTracker::getArmorPositionFromState(const Eigen::VectorXd &x) {
    // Calculate predicted position of the current armor
    double xc = x(0), yc = x(2), za = x(4);
    double yaw = x(6), r = x(8);
    double xa = xc - r * cos(yaw);
    double ya = yc - r * sin(yaw);
    Eigen::Vector3d armor_position = Eigen::Vector3d(xa, ya, za);
    return armor_position;
}

double shortest_angular_distance(double from, double to) {
    double yaw = to - from;
    const double result = fmod(yaw + M_PI, 2.0 * M_PI);
    if(result <= 0.0) return result + M_PI;
    return result - M_PI;
}

double StateTracker::RotationToYaw(double yaw) {
    // Make yaw change continuous
    yaw = last_yaw_ + shortest_angular_distance(last_yaw_, yaw);
    last_yaw_ = yaw;
    return yaw;
}
