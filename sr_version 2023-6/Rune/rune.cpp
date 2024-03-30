
#include "../Thread/thread.h"
//#include "rune_pnp.h"
// TODO: ------------------------------ rune part ------------------------------
//double last_time=0;
int last_mode=1;
int cnt=0;
int over_time_sum;
void Aim::rune(Aimpack src) {
    if (src.aim_mode!= last_mode)
    {
        last_mode = src.aim_mode;
        rune_predictor.history_info.clear();
        rune_predictor.pf.initParam(rune_predictor.pf_param_loader);
        rune_predictor.is_params_confirmed = false;
    }
    rune_detector.detect_color=src.enemy_color==0?RuneDetector::self_BLUE:RuneDetector::self_RED;
    predictor.plat_state.yaw_angle=src.yaw_angle;
    predictor.plat_state.pitch_angle=src.pitch_angle;
    bullet_speed = state_predictor.state_tracker_->shoot_speed = src.bullet_speed;
    if (rune_detector.findCenter(src.img)) {
        // rune.center -> aim.center
        r_center = rune_detector.r_center;









#ifdef SHOW_RUNE_CENTERS
        circle(rune_info, rune_armor.center, 2, Scalar(0, 255, 255), 2);
        circle(rune_info, r_center, 6, Scalar(0, 157, 255), -1);
#endif

//        radian = atan2((rune_armor.center.y - r_center.y), (rune_armor.center.x - r_center.x));
//        /*float angle = radian * 180 / 3.1415926;*/
//
//        real_angle = float(int((radian * 180 / CV_PI + 450) * 100 + 0.5) % 36000) / 100.0;
//        cout<<"angle     "<<rune_detector.final_fan.fan_angle<<endl;
        real_angle=rune_detector.final_fan.fan_angle;
        super_angle = real_angle;
        radian=real_angle/180*M_PI;
//        dt=src.timestamp-last_time;
//        last_time=src.timestamp;
        if (dt < 1000)
            time_sum += dt;
        timeSum += dt;
//        cout<<dt<<endl;
        // Judge the mode.
//        aim_mode=3;
        spin_speed = (radian - last_radian) / (dt / 1000.0);
        if(rotation_flag == 0)
            if(rotation_judge.size()<10){
                if(spin_speed > -3 || spin_speed < 3){
                    rotation_judge.push(spin_speed);
                    if(spin_speed>0) speed_positive++;
                    else if(spin_speed<0) speed_negative++;
                }
            }else{
                if(speed_positive>speed_negative) rotation_flag=1;
                else if(speed_negative>speed_positive) rotation_flag=-1;


            }
//        src.aim_mode=BIG_BUFF;
        if(src.aim_mode==SMALL_BUFF){
            real_angle += 20*rotation_flag;
        }else if(src.aim_mode==BIG_BUFF){
            // spining speed.(rad/s).
            //timeSum (ms)
//            rune_predictor.predict(spin_speed, r2c_distance, timeSum, pre_angle);
            if(!rune_predictor.predict(spin_speed, r2c_distance, timeSum, pre_angle)) pre_angle=0.18*rotation_flag;
//            cout<<"                         "<<pre_angle<<endl;
            real_angle += pre_angle * 180 / CV_PI;
        }

        if(real_angle >= 360) real_angle -= 360;

        double r_dis = sqrt((rune_detector.final_fan.target_center.x - r_center.x) * (rune_detector.final_fan.target_center.x - r_center.x)
                            + (rune_detector.final_fan.target_center.y - r_center.y) * (rune_detector.final_fan.target_center.y - r_center.y));

        rune_distance=4200/r_dis/0.00345;

        cout<<"r_dis       "<<r_dis<<endl;
        cout<<"rune_distance:  "<<rune_distance<<endl;

        pre_center.x = r_center.x + sin(((real_angle+90) * CV_PI / 180)) * r_dis;
        pre_center.y = r_center.y - cos(((real_angle+90) * CV_PI / 180)) * r_dis;

        float dx = pre_center.x - src.img.cols/2;
        double p_yaw = atan(dx / 1739) * 180 / CV_PI;
        float dy = pre_center.y - src.img.rows/2;
        double p_pitch = atan(dy / 1739) * 180 / CV_PI;
        putText(rune_info,to_string(real_angle),Point2f(700,700),2,2,Scalar(0,255,255),1);
        circle(rune_info, r_center, 4, Scalar(0, 255, 0), -1);
//        circle(rune_info, pre_center, 4, Scalar(0, 255, 255), 2);
        circle(rune_info, pre_center, 4, Scalar(0, 255, 255), 2);
        putText(rune_info,to_string(p_yaw),Point2f(500,500),2,2,Scalar(0,255,255),1);
        putText(rune_info,to_string(p_pitch),Point2f(500,600),2,2,Scalar(0,255,255),1);
        circle(rune_info, Point2f(rune_info.size().width / 2, rune_info.size().height / 2), 4, Scalar(255, 255, 255),
               -1);

            last_radian = radian;


//        rune_pnp.solvePnP(rune_detector.final_fan.armor_points)

#ifdef SHOW_CERES_DEBUG
        if (timeSum / 40 > ceres_debug.cols) {
            timeSum = 0;
            ceres_debug = Mat::zeros(200, 900, CV_8UC3);
        }
//        cout<<spin_speed<<endl;
        circle(ceres_debug, Point2f(timeSum / 40, 150 + rune_predictor.c_function * 24), 1, Scalar(255, 255, 0), 1);
        circle(ceres_debug, Point2f(timeSum / 40, 150 + rune_predictor.t_speed * 24), 1, Scalar(0, 0, 255), 1);
//        circle(ceres_debug, Point2f(timeSum / 40, 250 + spin_speed * 24), 1, Scalar(0, 255, 255), 1);
        circle(ceres_debug, Point2f(timeSum / 40, 150 + pre_angle * 24), 1, Scalar(255, 0, 255), 1);
//        circle(ceres_debug, Point2f(timeSum / 40, 250 + predictor.plat_state.pitch_angle * 8), 1, Scalar(255, 0, 255), 1);

        imshow("ceres_debug", ceres_debug);

#endif
        float bullet_comp = 0;
        // 90 0 90 0
//         int temp = abs(int(abs(int(real_angle) - 360*(real_angle/181))) % 90 -90);
        // 180 90 0 90
//        int temp = abs(int(abs(int(real_angle) - 360 * (real_angle / 181))) - 180);
//        bullet_comp = temp / 90;
//        p_pitch -= bulletDroppingCompensation(r2c_distance) - bullet_comp+3.5;

        send_data.yaw_angle = src.yaw_angle - p_yaw +0.2;

        if (send_data.yaw_angle > 360) {
            send_data.yaw_angle -= 360;
        } else if (send_data.yaw_angle < 0) {
            send_data.yaw_angle += 360;
        }

        //send_data.pitch_angle = p_pitch;
        send_data.pitch_angle = src.pitch_angle + p_pitch + 90-4.4;
        send_data.distance = r2c_distance * 1000;

        send_data.detect_flag = 1;
        if (send_data.yaw_angle > 360 || send_data.yaw_angle < 0) {
            return;
        }
#ifdef OPEN_SERIAL
        send();
#endif
    } else {
        cerr<<"zonghaoshabi"<<endl;
        send_data.yaw_angle = 0;
        send_data.pitch_angle = 0;
        send_data.distance = 0;
        send_data.detect_flag = 0;
#ifdef OPEN_SERIAL
        send();
#endif
    }
}