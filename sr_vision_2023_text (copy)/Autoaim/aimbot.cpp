//
// Created by root on 2022/7/4.
//
#include "../Thread/thread.h"


int lost_cnt = 0;
int detect_cnt = 0;
double dtt = 0;
void getEulerYPR(Eigen::Matrix3d m_el,double &yaw,double &pitch,double &roll);
void Aim::armor(Aimpack src) {
    //fill the serial pack
    detector.detect_color=src.enemy_color;
    predictor.plat_state.yaw_angle=src.yaw_angle;
    predictor.plat_state.pitch_angle=src.pitch_angle;
    bullet_speed = state_predictor.state_tracker_->shoot_speed = src.bullet_speed;

//    cout<<"bullet"<<src.bullet_speed<<endl;
//?
    if (src.img.empty())
        return;

    detector.trackArmor(src.img, armors);

    if (armors.empty()) {
        detect_cnt = 0;
        send_data.clear();
        // Tracking state update.
        if (lost_cnt < 6) {
            lost_cnt++;
            predictor.tracking_state = EKF_Predictor::TEMP_LOST;//temporary lost of frame
        } else {
            predictor.tracking_state = EKF_Predictor::LOST;
        }
#ifdef OPEN_SERIAL
        send_data.clear();
        send();
#endif
        return;
    } else {
        lost_cnt = 0;
        // Tracking state update.
        if(detect_cnt <= 3){
            detect_cnt++;
            predictor.tracking_state = EKF_Predictor::DETECTING;
        }else{
            predictor.tracking_state = EKF_Predictor::TRACKING;
        }

		// Set target priority.
        targetFollowing();
  }
//    Mat coordinate_debug=armor_info.clone();
//    int cnt=0;
        for (auto & armor : armors) {
            armor.pose.plat_state.pitch_angle=src.pitch_angle;
            armor.pose.plat_state.yaw_angle=src.yaw_angle;
            Point3f pt3f;
            solver.solvePnP(armor,pt3f,armor.rvec_,armor.tvec_);
//            vector<Point3f> RealPoint;
//            RealPoint.emplace_back(Point3f(0.1,0.1,0.1));
//            RealPoint.emplace_back(Point3f(0.1,-0.1,0.1));
//            RealPoint.emplace_back(Point3f(-0.1,-0.1,0.1));
//            RealPoint.emplace_back(Point3f(-0.1,0.1,0.1));
//            RealPoint.emplace_back(Point3f(0.1,0.1,-0.1));
//            RealPoint.emplace_back(Point3f(0.1,-0.1,-0.1));
//            RealPoint.emplace_back(Point3f(-0.1,-0.1,-0.1));
//            RealPoint.emplace_back(Point3f(-0.1,0.1,-0.1));
////        RealPoint.emplace_back(Point3f(xc,yc,zc));
//            RealPoint.emplace_back(Point3f(0,0,0));
////        RealPoint.emplace_back(Point3f(0.1,0.1,-0.1));
////        RealPoint.emplace_back(Point3f(0.1,0.1,-0.1));
////        RealPoint.emplace_back(Point3f(0.1,0.1,-0.1));
//
//            vector<Point2f> ImagePoint;
//            Mat camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(cameraMatrix.data())).clone());
//            Mat dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(distCoeffs.data())).clone());
//            projectPoints(RealPoint, armor.rvec_, armor.tvec_, camera_matrix_, dist_coeffs_, ImagePoint);
//
//            line(coordinate_debug,ImagePoint[0],ImagePoint[1],Scalar(0,255,60),4);
////        line(armor_info,armors[0].center,ImagePont[1],Scalar(0,255,0),3);
////        line(armor_info,armors[0].center,ImagePoint[2],Scalar(255,0,0),3);
//            line(coordinate_debug,ImagePoint[1],ImagePoint[2],Scalar(0,78,255),4);
//            line(coordinate_debug,ImagePoint[2],ImagePoint[3],Scalar(0,78,255),4);
//            line(coordinate_debug,ImagePoint[3],ImagePoint[0],Scalar(0,78,255),3);
//            line(coordinate_debug,ImagePoint[4],ImagePoint[5],Scalar(220,0,255),3);
//            line(coordinate_debug,ImagePoint[5],ImagePoint[6],Scalar(220,0,255),3);
//            line(coordinate_debug,ImagePoint[6],ImagePoint[7],Scalar(220,0,255),3);
//            line(coordinate_debug,ImagePoint[7],ImagePoint[4],Scalar(220,0,255),3);
//            line(coordinate_debug,ImagePoint[0],ImagePoint[4],Scalar(0,0,255),6);
//            line(coordinate_debug,ImagePoint[1],ImagePoint[5],Scalar(0,0,255),6);
//            line(coordinate_debug,ImagePoint[2],ImagePoint[6],Scalar(0,0,255),6);
//            line(coordinate_debug,ImagePoint[3],ImagePoint[7],Scalar(0,0,255),6);
//            line(coordinate_debug,ImagePoint[0],ImagePoint[5],Scalar(0,0,255),6);
////        line(armor_info,ImagePoint[8],ImagePoint[9],Scalar(224,44,138),5);
////        line(armor_info,ImagePoint[1],ImagePoint[8],Scalar(224,44,138),2);
////        line(armor_info,ImagePoint[2],ImagePoint[8],Scalar(224,44,138),2);
////        line(armor_info,ImagePoint[7],ImagePoint[8],Scalar(224,44,138),2);
//
//            circle(coordinate_debug,ImagePoint[0],2,Scalar(0,0,255),2);
//            circle(coordinate_debug,ImagePoint[1],2,Scalar(0,0,255),2);
//            circle(coordinate_debug,ImagePoint[2],2,Scalar(0,0,255),2);
//            circle(coordinate_debug,ImagePoint[3],2,Scalar(0,0,255),2);
//            circle(coordinate_debug,ImagePoint[4],2,Scalar(0,255,255),2);
//            circle(coordinate_debug,ImagePoint[5],2,Scalar(0,255,255),2);
//            circle(coordinate_debug,ImagePoint[6],2,Scalar(0,255,255),2);
//            circle(coordinate_debug,ImagePoint[7],2,Scalar(0,255,255),2);
//
//            putText(coordinate_debug,to_string(armor.pose.yaw/CV_PI*180),Point(800+cnt*50,800+cnt*50),2,2,Scalar(156,178,211),1);
//
//
//            cnt++;
//            imshow("coordinate_debug",coordinate_debug);
         }

        // Camera coord to Euler angle.
        predictor.now_euler.toEuler(pt3f);


#ifdef USE_PREDICTOR
        // Clone the armor_info to img_show.
        // Get now_time.
        cv::Mat img_show = armor_info;

        dtt = (src.timestamp - last_time) / 1000;
        src.aim_mode = NORMAL_AIM;
        if(src.aim_mode == NORMAL_AIM) {
            if(predictor.tracking_state == EKF_Predictor::TRACKING) {
                bool ok = predictor.predict(armors[0], img_show, src.timestamp/1000);
                if(ok)
                    cout << "Predict successfully" << endl;
                else
                    cout << "Error" << endl;
            }
        }
        else if(src.aim_mode == ANTI_TOP) {
            Mat target_debug = armor_info.clone();
            state_predictor.predict(armors, dtt, img_show);

            Eigen::VectorXd target_state = state_predictor.state_tracker_->target_state;
            Armor a = state_predictor.state_tracker_->tracked_armor;
            Eigen::VectorXd pre_state = state_predictor.state_tracker_->pre_state;
            Eigen::Vector4d tar_position[4];
            double yaw = predictor.now_euler.yaw;
 //
//            double target_g_yaw, tmp_g_yaw;
//            double gimbal_yaw = predictor.now_euler.yaw;
//            auto target_pre_info = state_predictor.state_tracker_->pre_ArmorsOfState;
//            Eigen::Vector3d p_pw, tmp_p_pw;
//            p_pw = tmp_p_pw = state_predictor.state_tracker_->pre_position;

            // Choose the target armor from calculating the closet yaw.


//            target_g_yaw = ;
//            double min_yaw_diff = fabs(target_pre_info[0].yaw - gimbal_yaw);
//            for(int i = 1; i < 4; i++) {
//                if(fabs(target_pre_info[i].yaw- gimbal_yaw) < min_yaw_diff) {
//                    min_yaw_diff = fabs(target_pre_info[i].yaw- gimbal_yaw) - gimbal_yaw;
//                    target_g_yaw = target_pre_info[i].yaw;
//                    p_pw = target_pre_info[i].position;
//                }
//            }

            Eigen::Vector3d p_pw;
            double tar_yaw = target_state(6) + dtt * target_state(7);
            double r1 = target_state(8), r2 = state_predictor.state_tracker_->another_r;
            double xc = target_state(0), yc = target_state(2), za = target_state(4);
            double vx = target_state(1), vy = target_state(3), vz = target_state(5);

            bool is_current_pair = true;
            auto a_n = static_cast<int>(state_predictor.state_tracker_->tracked_armors_num);
            for(int i = 0; i < 4; i++) {
                double tmp_yaw = tar_yaw + i * (2 * M_PI / a_n);
                double r = is_current_pair ? r1 : r2;
                tar_position[i].x() = xc - r * cos(tmp_yaw);
                tar_position[i].y() = yc - r * sin(tmp_yaw);
                tar_position[i].z() = za + (is_current_pair ? 0 : state_predictor.state_tracker_->dz);
                tar_position[i].w() = tmp_yaw;
                is_current_pair = !is_current_pair;
            }
            int idx = 0;
            double yaw_diff_min = abs(yaw - tar_position[0].w());
            for(int i = 0; i < 4; i++) {
                double temp_yaw_diff = abs(yaw - tar_position[i].w());
                if(temp_yaw_diff < yaw_diff_min) {
                    yaw_diff_min = temp_yaw_diff;
                    idx = i;
                }
            }
            p_pw = {tar_position[idx].x() + dtt * vx, tar_position[idx].y() + dtt * vy, tar_position[idx].z() + dtt * vz};

#ifdef SHOW_TARGET_DEBUG
            state_predictor.state_tracker_->re_project_point(target_debug, p_pw, a.pose.plat_state.yaw_angle,a.pose.plat_state.pitch_angle, {255, 255, 125});
#endif

            predictor.predict_euler.yaw = atan2(p_pw(1, 0),p_pw(0,0)) * 180. / M_PI;
            predictor.predict_euler.pitch = atan2(p_pw(0, 0),p_pw(2,0)) * 180. / M_PI;
            predictor.distance = p_pw.norm();

#ifdef PRINT_TARGET_INFO
            // Print the target_state.
            cv::putText(target_debug,"Xc: "+to_string(target_state(0)),Point2f(10,10),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"Yc: "+to_string(target_state(2)),Point2f(10,30),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"Zc: "+to_string(target_state(4)),Point2f(10,50),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"yaw:"+to_string(target_state(6)),Point2f(10,70),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"Vxc:"+to_string(target_state(1)),Point2f(10,90),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"Vyc:"+to_string(target_state(3)),Point2f(10,110),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"Vzc:"+to_string(target_state(5)),Point2f(10,130),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"Vyaw:"+to_string(target_state(7)),Point2f(10,150),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"r   :"+to_string(target_state(8)),Point2f(10,170),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"a_r   :"+to_string(state_predictor.state_tracker_->another_r),Point2f(10,190),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"dz   :"+to_string(state_predictor.state_tracker_->dz),Point2f(10,210),1,1,Scalar(20,48,200),1);
            // Print the gimbal_euler.
            cv::putText(target_debug,"g_yaw   :"+to_string(predictor.predict_euler.yaw),Point2f(200,10),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"g_pitch   :"+to_string(predictor.predict_euler.pitch),Point2f(200,30),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"distance   :"+to_string(predictor.distance),Point2f(200,50),1,1,Scalar(200,48,200),1);
            // Print the armor position in world coordinate.
            cv::putText(target_debug,"x:   :"+to_string(a.pose.pw[0]),Point2f(400,200),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"y:   :"+to_string(a.pose.pw[1]),Point2f(400,220),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"z:   :"+to_string(a.pose.pw[2]),Point2f(400,240),1,1,Scalar(20,48,200),1);
            cv::putText(target_debug,"ft::::"+to_string(dtt),Point2f(400,270),1,1,Scalar(20,48,200),1);
#ifdef SHOW_TARGET_DEBUG
            resize(target_debug,target_debug,Size(720,540),INTER_NEAREST);
            imshow("target_debug", target_debug);
#endif

#endif
        }
        last_time = src.timestamp;
#endif
		// Compensation of bullet dropping.
//        float comp = bulletDroppingCompensation(predictor.now_euler.distance);
        auto pitchoffset = dynamicCalcPitchOffset(armors[0].pose.pw);
//        cout<<"pitch:::::::::::::                   "<<pitchoffset<<endl;


#ifdef OPEN_SERIAL
		// New plan - turn the deviation angles to absolute angles.(Infant ry used).
        send_data.yaw_angle = predictor.predict_euler.yaw +0.3;
        if(send_data.yaw_angle>360)
            send_data.yaw_angle-=360;
        // send_data.yaw_angle=predictor.predict_euler.yaw ;
		// Judge whether the angles are crossing the border.
        if (send_data.yaw_angle > 360) {
            send_data.yaw_angle -= 360;
        } else if (send_data.yaw_angle < 0) {
            send_data.yaw_angle += 360;
        }

//        send_data.pitch_angle = predictor.plat_state.pitch_angle + predictor.predict_euler.pitch + 90 ;
        send_data.pitch_angle =predictor.predict_euler.pitch - pitchoffset+90+0.3;
//        send_data.pitch_angle=90;
        cout<<"send_data.pitch_angle:   "<<send_data.pitch_angle<<endl;
//        cout<<"predictor.plat_state.pitch_angle:   "<<predictor.plat_state.pitch_angle<<endl;
//        cout<<"predictor.now_euler.pitch:   "<<predictor.now_euler.pitch<<endl;
        send_data.distance = predictor.predict_euler.distance;

		// Detect flag update.
        if(predictor.tracking_state>0)
            send_data.detect_flag = 1;
        else
            send_data.detect_flag = 0;
        send();
#endif

#ifdef SHOW_CENTERS
        Point2f img_center = Point2f(armor_info.size().width / 2, armor_info.size().height / 2);
        Point2f armor_center = armors[0].center;

        int delta_x = tan(((predictor.predict_euler.yaw - 90) / 180.0) * CV_PI) * 850 + 320;
        int delta_y = tan(((send_data.pitch_angle - 50) / 180.0) * CV_PI) * 800 + 240;

        Point2f pre_center = Point2f(delta_x, delta_y);

        circle(armor_info, img_center, 4, Scalar(10, 255, 10), 3);
        circle(armor_info, armor_center, 4, Scalar(0, 255, 255), 3);
        circle(armor_info, pre_center, 6, Scalar(0, 255, 0), -1);

        for (int j = 0; j < 4; j++) {
            line(armor_info, armors[0].pt[j] + detector.relative_coord,
                 armors[0].pt[(j + 1) % 4] + detector.relative_coord, Scalar(18, 226, 252), 2);
        }
        imshow("centers",armor_info);
#endif


#ifdef SHOW_YAW_SCATTER_DIAGRAM
        timeSum += dt;

        if (timeSum / 100 > filter_debug.cols) {
            timeSum = 0;
            filter_debug = Mat::zeros(500, 1500, CV_8UC3);
        }
        circle(filter_debug, Point2f(timeSum / 100, predictor.now_euler.yaw * 8 - 480), 1, Scalar(0, 255, 255), -1);
        circle(filter_debug, Point2f(timeSum / 100, predictor.predict_euler.yaw * 8 - 480), 1, Scalar(0, 255, 0), -1);
        imshow("filter debug", filter_debug);
#endif

    }




int Aim::targetFollowing() {

    predictor.id_filter.cur_id = armors[0].number;

    if (armors.size() > 1) {
        bool find_same_as_last = false;
        if (predictor.id_filter.cur_id != predictor.id_filter.last_id && predictor.id_filter.last_id != -1) {
            for (int i = 1; i < armors.size(); i++) {
                if (armors[i].number == predictor.id_filter.last_id && armors[i].number != '2') {
                    swap(armors[0], armors[i]);
                    find_same_as_last = true;
                    predictor.tracking_state = EKF_Predictor::TRACKING;
                    predictor.id_filter.lost_cnt = 0;
                }
            }
            if (find_same_as_last == 0) {
                if(predictor.id_filter.lost_cnt < 12) {
                    predictor.id_filter.lost_cnt++;
                    predictor.tracking_state = EKF_Predictor::TEMP_LOST;
                }
                else{
                    predictor.tracking_state = EKF_Predictor::LOST;
                    predictor.id_filter.last_id = -1;
                }
            }
        } else if (predictor.id_filter.cur_id == predictor.id_filter.last_id) {
            find_same_as_last = true;
        }
    }

    predictor.id_filter.last_id = armors[0].number;

    return predictor.tracking_state;
}





// TODO: ------------------------------ serial part ------------------------------


bool Aim::makeSendData() {
    send_data.yaw_angle =predictor.predict_euler.yaw - 1.6;
    send_data.pitch_angle = predictor.predict_euler.pitch;
    send_data.distance = predictor.predict_euler.distance;
    if(predictor.tracking_state>0) send_data.detect_flag = 1;
    else send_data.detect_flag = 0;
    return true;
}

bool Aim::send(){
#ifdef PRINT_SEND_MESSAGE
    cout<<"send yaw:      "<<send_data.yaw_angle<<endl;
    cout<<"send pitch:    "<<send_data.pitch_angle<<endl;
    cout<<"send distance: "<<send_data.distance<<endl;
#endif
    if(!serial_port.sendData(S_NUM,send_data,0xAA,0xAF)){
        int cnt_send=0;
        while(!serial_port.initSerial("/dev/ttyRM", 115200, 'N', 8, 1)){
            if(cnt_send>500){
                exit(0);
            }
            cnt_send++;
        }
    }
    return true;
}

