//
// Created by root on 23-4-15.
//

#ifndef SR_SDUST_ARMOR_PREDICT_H
#define SR_SDUST_PREDICT_H
// STD
#include <memory>
#include <string>
#include <vector>

#include "parameter.h"
#include "supervisor.h"

#include "tracker.hpp"

class StatePredict
{
public:
    StatePredict(double max_match_distance, int tracking_state, int lost_threshold,double max_match_yaw_diff);
    void predict(vector<Armor> & Armors, double time, Mat src);

    struct Target{
        bool tracking;
        char id;
        double pitch, yaw;
    };

    // Armor tracker
    double lost_time_thres_;

    double last_time_;

    std::unique_ptr<StateTracker> state_tracker_;

    Target target;
    Eigen::Vector4d measure_state;
    PlatState plat_state;
    Euler predict_euler;

    Eigen::Matrix<double, 3, 3> R_IW;

    Mat predict_debug;

};

// 时间处理
struct LastTimeHelper {
    LastTimeHelper(double current_time, double &ref_time) : c_time(current_time), r_time(ref_time) {};
    ~LastTimeHelper() { r_time = c_time; }
    double c_time;
    double &r_time;
};



#endif //SR_SDUST_ARMOR_PREDICT_H
