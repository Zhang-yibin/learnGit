//
// Created by root on 23-4-15.
//

#include "state_predict.h"
#include <fmt/format.h>
#include <fmt/color.h>
StatePredict::StatePredict(double max_match_distance, int tracking_threshold, int lost_threshold,double max_match_yaw_diff)
: last_time_(0.0)
{
    state_tracker_ = std::make_unique<StateTracker>(max_match_distance, tracking_threshold, lost_threshold,max_match_yaw_diff);
    lost_time_thres_ = 0.6;

    // EKF
    // xa = x_armor, xc = x_robot_center
    // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
    // measurement: xa, ya, za, yaw
    // f - Process function
    state_tracker_->Load_EKF();

    Mat camera_Matrix = cv::Mat(3, 3, CV_64F, const_cast<double *>(cameraMatrix.data())).clone();
    cv::cv2eigen(camera_Matrix, state_tracker_->cam_e_Matrix);
}

void StatePredict::predict(vector<Armor> & Armors, double dt, Mat src) {
    // Filter abnormal armors
    Armors.erase(
            std::remove_if(
                    Armors.begin(), Armors.end(),
                    [this](const Armor & armor) {
                        return abs(armor.pose.pw[2]) > 1.2 ||
                                Eigen::Vector2d(armor.pose.pw[0], armor.pose.pw[1]).norm() >
                                10.0;
                    }),
            Armors.end());


    // Calculate delta_t as dt
//    LastTimeHelper Keeper(time, last_time_);

    R_IW = state_tracker_->get_R_IW(plat_state);
    state_tracker_->R_IW = R_IW;

    state_tracker_->tracker_debug = src.clone();

    // Change the state.
    // Make tracker predict.
    if (state_tracker_->tracker_state == StateTracker::LOST) {
        state_tracker_->init(Armors);
        fmt::print(fmt::fg(fmt::color::green), "init!!\n");

        this->target.tracking = false;
    } else {
//        state_tracker_->dt_ = time - last_time_;
        state_tracker_->dt_ = dt;
        state_tracker_->lost_threshold_ = static_cast<int>(lost_time_thres_ / state_tracker_->dt_);
        state_tracker_->update(Armors);

        measure_state = state_tracker_->measurement;

        if (state_tracker_->tracker_state ==StateTracker::DETECTING) {
            this->target.tracking = false;
        } else if (
                state_tracker_->tracker_state ==StateTracker::TRACKING ||
                        state_tracker_->tracker_state ==StateTracker::TEMP_LOST) {
            this->target.tracking = true;
            this->target.id = state_tracker_->tracked_id;
        }
    }

//    last_time_ = time;

}

