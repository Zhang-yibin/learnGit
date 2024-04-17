#define byvideo
//#define byhik
#ifdef byhik

#include "thread.h"
#include <fstream>
#define SAVE_VIDEO
#define USING_VIDEO
#define USING_HIK


/**
 * @brief 生产者线程
 * @param factory 工厂类
**/
[[noreturn]] bool producer(Factory<Aimpack> &factory, std::chrono::_V2::steady_clock::time_point time_start)
{
    int last_mode=0;
    int last_timestamp;
    Serial_receive receive_port;
#ifdef USING_HIK
    {
        start_get_img:
        HaiKangCamera HaiKang;
        HaiKang.StartDevice(0);
        HaiKang.SetResolution(640, 480);// 设置分辨率
        HaiKang.SetFPS(400);
        HaiKang.UpdateTimestampOffset(time_start);//更新时间戳，设置时间戳偏移量
        HaiKang.SetStreamOn();// 开始采集帧
        HaiKang.SetExposureTime(2400);// 设置曝光
        // 设置1
        HaiKang.SetGAIN(0, 12);
        HaiKang.GetImageParam(HaiKang.m_param);
        Aimpack src;
        while(1)
        {
            if(HaiKang.GetMat(src.img))
            {
                src.timestamp = (int)(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now() - time_start).count());
                if(last_mode!=src.aim_mode||last_timestamp!=src.timestamp)
                {
                    last_mode=src.aim_mode;
                    last_timestamp=src.timestamp;
                    factory.produce(src);
                }
            }
        }
        auto HaiKang_stauts = HaiKang.GetMat(src.img);
        auto time_cap = std::chrono::steady_clock::now();


        auto time_capppp = std::chrono::steady_clock::now();

        auto time = (std::chrono::duration<double,std::milli>(time_capppp - time_cap).count());
//        fmt::print(fmt::fg(fmt::color::yellow), "[producer latency]--{}\n",time);

        if (!HaiKang_stauts) {
            fmt::print(fmt::fg(fmt::color::red), "[CAMERA] GetMat false return\n");
            goto start_get_img;
        }
            src.timestamp = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count());


            last_mode=src.aim_mode;
            last_timestamp=src.timestamp;
            factory.produce(src);
#ifdef SHOW_PRODUCER_LATENCY


    }
    cout<<"[CAMERA] Set param finished"<<endl;
    auto lasttime=std::chrono::steady_clock::now();
    while(1)
    {

      #endif
#ifdef OPEN_SERIAL
        if(!receive_port.receive(src)){
            int cnts=0;
            while(!receive_port.openSerial()){
                if(cnts>500){
                    exit(0);
                }
                cnts++;
            }
        }

#endif
//        src.aim_mode=BIG_BUFF;
//        if(last_mode!=src.aim_mode){
//            if(src.aim_mode>1){
//                HaiKang.SetResolution(1440,1080);
//            }else{
//                HaiKang.SetResolution(1440,1080);// 设置曝光
//            }
//        }

#endif //USING_HIK


#ifdef USING_VIDEO
        cap >> src.img;
        auto nowtime=std::chrono::steady_clock::now();
        src.timestamp = (double)(std::chrono::duration<double,std::milli>(time_cap - time_start).count());
        auto fku_qwq = (double)(std::chrono::duration<double,std::milli>(lasttime - nowtime).count());
        cout<<fku_qwq
        <<"lasttime - nowtime"
        <<endl;
        lasttime=nowtime;
        //dont know whether is causeof thisor else but speed return normal after cout qwq......wth.
        imshow("76",src.img);
        //why is cause of this..
        //wtf
        // sleep(0.02);
//        waitKey(1);
#endif //USING_VIDEO
//        if (src.img.empty()){
//            continue;
//        }


    }
}


// Construct function "Aim".
Aim::Aim(double & max_match_distance,int & tracking_threshold,int & lost_threshold,double & max_match_yaw_diff,
         std::array<double, 9> & camera_matrix,vector<double> & dist_coeffs)://Member object initialization
        state_predictor(max_match_distance, tracking_threshold, lost_threshold,max_match_yaw_diff),solver(camera_matrix,dist_coeffs){}

bool consumer(Factory<Aimpack> &task_factory)
{
    Serial_receive send_port;
    Aimpack dst;
    Aim aim(max_Matchdistance, trackingThreshold, lostThreshold, maxMatchYawdiff,cameraMatrix, distCoeffs);
    auto time_start = std::chrono::steady_clock::now();
    while(1)
    {
        task_factory.consume(aim.src);
        auto time_end=std::chrono::steady_clock::now();
        aim.end_time = (std::chrono::duration<double,std::milli>(time_end - time_start).count());
        aim.dt=aim.end_time-aim.last_end_time;
        aim.run(aim.src);
        aim.last_end_time = aim.end_time;
//        if (waitKey(100) == 'q')
          if(waitKey(360)=='q')
            break;
    }
    return true;
}
void Aim::run(Aimpack src) {
   src.aim_mode=3;
    if(src.aim_mode > 1){
        rune_info = src.img.clone();

        // 0 small buff, 1 big buff.
        rune_predictor.mode = aim_mode - 2;
        rune(src);
        showRuneInfo();
        waitKey(500);
        cout<<"rune_pmod:"<<rune_predictor.mode<<endl;
        //?? dog toll
    }
    else{
        // Clear the params of buff.
        if(rune_predictor.params[0]!=0){
            for(int i=0;i<4;i++){
                rune_predictor.params[0]=0;
                rune_predictor.params[1]=0;
                rune_predictor.params[2]=0;
                rune_predictor.params[3]=0;
            }
        }
        armor_info = src.img.clone();
        // 0 loose, 1 strict.
        detector.more_strict_classification =1;
        armor(src);
        showArmorInfo();
    }
}



// TODO: ------------------------------ tool part ------------------------------// TODO: tool part
void Aim::showArmorInfo() {

    putText(armor_info, "mode: " + to_string(aim_mode),
            Point2f(0, 60), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(armor_info, "strict: " + to_string(detector.more_strict_classification),
            Point2f(0, 80), 5, 0.6, Scalar(0, 208, 255), 1);
    putText(armor_info, "speed: " + to_string(bullet_speed),
            Point2f(0, 300), 5, 0.6, Scalar(0, 208, 255), 1);


    // state
    if (!armors.empty()) {
        putText(armor_info, "ARMOR SEEN!", Point2f(0, 20),
                7, 0.8, Scalar(5, 57, 245), 2);

        putText(armor_info, armors[0].classification_result,
                Point2f(0, 40), 5, 0.6, Scalar(255, 233, 15), 1);

        putText(armor_info, "armor number: " + to_string(armors.size()),
                Point2f(80, 40), 5, 0.6, Scalar(255, 233, 15), 1);


#ifdef CV_INIT
//        putText(armor_info, "now yaw: " + to_string(predictor.now_euler.yaw),
//                Point2f(0, 180), 5, 0.6, Scalar(133, 245, 5), 1);
//
//        putText(armor_info, "now pitch: " + to_string(predictor.now_euler.pitch),
//                Point2f(0, 200), 5, 0.6, Scalar(133, 245, 5), 1);
//
        putText(armor_info, "distance: " + to_string(predictor.now_euler.distance),
                Point2f(0, 250), 5, 0.6, Scalar(133, 245, 5), 1);
//
//        putText(armor_info, "predict yaw: " + to_string(predictor.predict_euler.yaw),
//                Point2f(0, 240), 5, 0.6, Scalar(133, 245, 5), 1);
//
//        putText(armor_info, "predict pitch: " + to_string(predictor.predict_euler.pitch),
//                Point2f(0, 260), 5, 0.6, Scalar(133, 245, 5), 1);
#endif

    }
#ifdef OPEN_SERIAL

    // receive data
    putText(armor_info, "receive yaw: " + to_string(predictor.plat_state.yaw_angle),
            Point2f(0, 200), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(armor_info, "receive pitch: " + to_string(predictor.plat_state.pitch_angle),
            Point2f(0, 220), 5, 0.6, Scalar(201, 116, 201), 1);

    // send data
    putText(armor_info, "send yaw: " + to_string(send_data.yaw_angle),
            Point2f(400, 20), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(armor_info, "send pitch: " + to_string(send_data.pitch_angle),
            Point2f(400, 40), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(armor_info, "send distance: " + to_string(send_data.distance),
            Point2f(400, 60), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(armor_info, "detect flag: " + to_string(send_data.detect_flag),
            Point2f(400, 80), 5, 0.6, Scalar(0, 208, 255), 1);
#endif
    resize(armor_info,armor_info,Size(720,540),INTER_NEAREST);
    namedWindow("armor info",2);
    imshow("armor info", armor_info);

}


void Aim::showRuneInfo() {

    putText(rune_info, "rune mode: " + to_string(rune_predictor.mode),
            Point2f(0, 40), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(rune_info, "detect flag: " + to_string(1),
            Point2f(0, 160), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(rune_info, "r2c distance: " + to_string(send_data.distance),
            Point2f(0, 140), 5, 0.6, Scalar(0, 208, 255), 1);

    if (!rune_detector.armors.empty()) {
        // state
        putText(rune_info, "RUNE SEEN!", Point2f(0, 20),
                7, 0.8, Scalar(5, 57, 245), 2);

        putText(rune_info, "predict yaw: " + to_string(send_data.yaw_angle),
                Point2f(0, 100), 5, 0.6, Scalar(0, 208, 255), 1);

        putText(rune_info, "predict pitch: " + to_string(send_data.pitch_angle),
                Point2f(0, 120), 5, 0.6, Scalar(0, 208, 255), 1);

    }

    putText(rune_info, "r center: " + string("x-") + to_string(r_center.x) + string("  y-") + to_string(r_center.y),
            Point2f(0, 180), 5, 0.6, Scalar(247, 219, 116), 1);

    putText(rune_info, "send yaw: " + to_string(send_data.yaw_angle),
            Point2f(400, 20), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(rune_info, "send pitch: " + to_string(send_data.pitch_angle),
            Point2f(400, 40), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(rune_info, "send distance: " + to_string(send_data.distance),
            Point2f(400, 60), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(rune_info, "detect flag: " + to_string(send_data.detect_flag),
            Point2f(400, 80), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(rune_info, "predict angle: " + to_string(pre_angle * 180 / CV_PI),
            Point2f(400, 220), 5, 0.6, Scalar(201, 116, 201), 1);

#ifdef OPEN_SERIAL
    // receive data
    putText(rune_info, "receive yaw: " + to_string(predictor.plat_state.yaw_angle),
            Point2f(0, 200), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(rune_info, "receive pitch: " + to_string(predictor.plat_state.pitch_angle),
            Point2f(0, 220), 5, 0.6, Scalar(201, 116, 201), 1);
#endif
//    resize(rune_info,rune_info,Size(720,540),INTER_NEAREST);
    //why hwy why
    //why resize...
    imshow("rune info", rune_info);
}

bool Serial_receive::receive(Aimpack& pack) {
    bool receiveSuccess = receive_port.receiveData(R_NUM, 0xAA, 0xAF, r_data);
    if(receiveSuccess){
        pack.yaw_angle = r_data[1]*100+r_data[2]+(float)r_data[3]/100;//todo:iaosdjjdoa;ijdo;isjoi;

        if( r_data[4]-90 < 0){
            pack.pitch_angle = int(r_data[4]-90) + 1.0 - (float)(r_data[5]/100.0);
        }else{
            pack.pitch_angle = int(r_data[4]-90) + (float)(r_data[5]/100.0);
        }
        pack.aim_mode = r_data[6]/10;
        pack.enemy_color = int(r_data[6]%10);
//        rune_detector.enemy_color = RuneDetector::Color((r_data[6]%10+1)%2);
        pack.bullet_speed = double(r_data[7]) * 2 / 10;
        if(r_data[7]<=0||r_data[7]>30){
            pack.bullet_speed=27;
        }
//        if(bullet_speed==0){
//            bullet_speed=13.5;
//        }else if(bullet_speed-bullet_speed_queue[9]>3){
//            bullet_speed_queue.clear();
//            bullet_speed_queue.push_back(bullet_speed);
//        }
//        bullet_speed_queue.push_back(bullet_speed);
//        if(bullet_speed_queue.size() > 10) {
//            bullet_speed_queue.pop_front();
//        }
    }
    return receiveSuccess;
}

bool Serial_receive::openSerial(){
    bool usb = receive_port.initSerial("/dev/ttyRM", 115200, 'N', 8, 1);
    return usb;
}


/*
/*
/**
 * @brief 数据发送线程
 *
 * @param serial SerialPort类
 * @param transmit_factory Factory类
 * @return true
 * @return false
 */
/*
bool dataTransmitter(SerialPort &serial,Factory<VisionData> &transmit_factory)
{
    while(1)
    {
        VisionData transmit;
        transmit_factory.consume(transmit);
        //若串口离线则跳过数据发送
        //TODO:使用无串口的模式时会导致此线程死循环，浪费CPU性能
        if (serial.need_init == true)
        {
            // cout<<"offline..."<<endl;
            #ifndef DEBUG_WITHOUT_COM
                #ifdef SAVE_LOG_ALL
                    LOG(ERROR) << "[TRANSMITTER] Serial offline, trying to reconnect...";
                #endif //SAVE_LOG_ALL
            #endif //DEBUG_WITHOUT_COM
            usleep(5000);
            continue;
        }
        serial.TransformData(transmit);
        serial.send();
        // cout<<"transmitting..."<<endl;
    }
    return true;
}
*/
/*
#ifdef USING_IMU_C_BOARD
*/
/**
 * @brief 数据接收线程
 *
 * @param serial
 * @param receive_factory
 * @param time_start
 * @return true
 * @return false
 *//*

bool dataReceiver(SerialPort &serial, MessageFilter<MCUData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start)
{
    while(1)
    {
        //若串口离线则跳过数据发送
        if (serial.need_init == true)
        {
            // cout<<"offline..."<<endl;
            usleep(5000);
            continue;
        }
        //数据读取不成功进行循环
        while (!serial.get_Mode())
            ;
        auto time_cap = std::chrono::steady_clock::now();
        auto timestamp = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count());
        // cout<<"Quat: "<<serial.quat[0]<<" "<<serial.quat[1]<<" "<<serial.quat[2]<<" "<<serial.quat[3]<<" "<<endl;
        // Eigen::Quaterniond quat = {serial.quat[0],serial.quat[1],serial.quat[2],serial.quat[3]};
        //FIXME:注意此处mode设置
        int mode = serial.mode;
        float bullet_speed = serial.bullet_speed;

        // int mode = 2;
        Eigen::Quaterniond quat = {serial.quat[0],serial.quat[1],serial.quat[2],serial.quat[3]};
        Eigen::Vector3d acc = {serial.acc[0],serial.acc[1],serial.acc[2]};;
        Eigen::Vector3d gyro = {serial.gyro[0],serial.gyro[1],serial.gyro[2]};;
        MCUData mcu_status = {mode, acc, gyro, quat, bullet_speed, timestamp};
        receive_factory.produce(mcu_status, timestamp);
        // Eigen::Vector3d vec = quat.toRotationMatrix().eulerAngles(2,1,0);
        // cout<<"Euler : "<<vec[0] * 180.f / CV_PI<<" "<<vec[1] * 180.f / CV_PI<<" "<<vec[2] * 180.f / CV_PI<<endl;
        // cout<<"transmitting..."<<endl;
    }
    return true;
}
#endif //USING_IMU_C_BOARD

*/
/**
 * @brief 串口监视线程
 *
 * @param serial
 * @return true
 * @return false
 *//*

bool serialWatcher(SerialPort &serial)
{
    int last = 0;
#ifdef DEBUG_WITHOUT_COM
    #ifdef SAVE_TRANSMIT_LOG
    LOG(WARNING)<<"[SERIAL] Warning: You are not using Serial port";
    #endif //SAVE_TRANSMIT_LOG
#endif // DEBUG_WITHOUT_COM

    while(1)
    {
        sleep(1);
        //检测文件夹是否存在或串口需要初始化
        if (access(serial.device.path.c_str(),F_OK) == -1 || serial.need_init)
        {
            serial.need_init = true;
#ifdef DEBUG_WITHOUT_COM
            int now = clock()/CLOCKS_PER_SEC;
            if (now - last > 10)
            {
                last = now;
                fmt::print(fmt::fg(fmt::color::orange), "[SERIAL] Warning: You are not using Serial port\n");
            }
            serial.withoutSerialPort();
#else
            serial.initSerialPort();
#endif //DEBUG_WITHOUT_COM
        }
    }
}


// #ifdef USING_IMU_WIT
// bool dataReceiver(IMUSerial &serial_imu, MessageFilter<MCUData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start)
// {
//     while(1)
//     {
//         //若串口离线则跳过数据发送
//         if (serial_imu.need_init == true)
//         {
//             // cout<<"offline..."<<endl;
//             continue;
//         }
//         if (!serial_imu.readData())
//         {
//             continue;
//         }
//         auto time_cap = std::chrono::steady_clock::now();
//         auto timestamp = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count());
//         if (!serial_imu.is_quat_initialized)
//         {
//             continue;
//         }
//         Eigen::Quaterniond quat = serial_imu.quat;
//         Eigen::Vector3d acc = serial_imu.acc;
//         Eigen::Vector3d gyro =serial_imu.gyro;
//         MCUData imu_status = {acc, gyro, quat, timestamp};

//         receive_factory.produce(imu_status, timestamp);
//         Eigen::Vector3d vec = quat.toRotationMatrix().eulerAngles(2,1,0);
//     }
//     return true;
// }

// bool serialWatcher(SerialPort &serial, IMUSerial &serial_imu)
// {
//     while(1)
//     {
//         sleep(0.1);
//         //检测文件夹是否存在或串口需要初始化
//         if (access(serial.device.path.c_str(),F_OK) == -1 || serial.need_init)
//         {
//             serial.need_init = true;
//             serial.initSerialPort();
//         }
//         if (access(serial_imu.device.path.c_str(),F_OK) == -1 || serial_imu.need_init)
//         {
//             serial_imu.need_init = true;
//             serial_imu.initSerialPort();
//         }

//     }
// }
// #endif //USING_WIT_IMU

// #ifndef USING_IMU
// bool serialWatcher(SerialPort &serial)
// {
//     while(1)
//     {
//         sleep(0.1);
//         //检测文件夹是否存在或串口需要初始化
//         if (access(serial.device.path.c_str(),F_OK) == -1 || serial.need_init)
//         {
//             serial.need_init = true;
//             serial.initSerialPort();
//         }

//     }
// }
// #endif //USING_IMU

*/




#endif



#ifdef byvideo
#include "thread.h"
#include <fstream>
#define SAVE_VIDEO
#define USING_VIDEO
//#define USING_HIK
/**
 * @brief 生产者线程
 * @param factory 工厂类
**/
[[noreturn]] bool producer(Factory<Aimpack> &factory, std::chrono::_V2::steady_clock::time_point time_start)
{
    int last_mode=0;
    int last_timestamp;
    Serial_receive receive_port;
#ifdef USING_HIK
    {
        start_get_img:
        HaiKangCamera HaiKang;
        HaiKang.StartDevice(0);
        HaiKang.SetResolution(1440, 1080);// 设置分辨率
        HaiKang.SetFPS(400);
        HaiKang.UpdateTimestampOffset(time_start);//更新时间戳，设置时间戳偏移量
        HaiKang.SetStreamOn();// 开始采集帧
        HaiKang.SetExposureTime(2400);// 设置曝光
        // 设置1
        HaiKang.SetGAIN(0, 12);
        HaiKang.GetImageParam(HaiKang.m_param);
    }
#endif //USING_HIK
#ifdef USING_VIDEO
//    sleep(6);//防止网络加载完成前视频开始播放
//    VideoCapture cap("/home/yukki/0307.mp4");
//    VideoCapture cap("/home/yukki/true2.mp4");
VideoCapture cap("/home/yukki/LIT FULL.mp4");
//    VideoCapture cap("/home/yukki/LITLITLIT.MP4");
//    cap.set(5, 1);

#endif //USING_VIDEO
    cout<<"[CAMERA] Set param finished"<<endl;
    auto lasttime=std::chrono::steady_clock::now();
    while(1)
    {
        Aimpack src;
        auto time_cap = std::chrono::steady_clock::now();

#ifdef USING_HIK
        auto HaiKang_stauts = HaiKang.GetMat(src.img);
        if (!HaiKang_stauts){
            fmt::print(fmt::fg(fmt::color::red), "[CAMERA] GetMat false return\n");
            goto start_get_img;
        }
        auto time_capppp = std::chrono::steady_clock::now();
        src.timestamp = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count());
#ifdef SHOW_PRODUCER_LATENCY
        auto time = (std::chrono::duration<double,std::milli>(time_capppp - time_cap).count());
//        fmt::print(fmt::fg(fmt::color::yellow), "[producer latency]--{}\n",time);
#endif
#ifdef OPEN_SERIAL
        if(!receive_port.receive(src)){
            int cnts=0;
            while(!receive_port.openSerial()){
                if(cnts>500){
                    exit(0);
                }
                cnts++;
            }
        }

#endif
//        src.aim_mode=BIG_BUFF;
//        if(last_mode!=src.aim_mode){
//            if(src.aim_mode>1){
//                HaiKang.SetResolution(1440,1080);
//            }else{
//                HaiKang.SetResolution(1440,1080);// 设置曝光
//            }
//        }
        last_mode=src.aim_mode;
        last_timestamp=src.timestamp;
#endif //USING_HIK


#ifdef USING_VIDEO
        cap >> src.img;
        auto nowtime=std::chrono::steady_clock::now();
        src.timestamp = (double)(std::chrono::duration<double,std::milli>(time_cap - time_start).count());
        auto fku_qwq = (double)(std::chrono::duration<double,std::milli>(lasttime - nowtime).count());
        cout<<fku_qwq
        <<"lasttime - nowtime"
        <<endl;
        lasttime=nowtime;
        //dont know whether is causeof thisor else but speed return normal after cout qwq......wth.
        imshow("76",src.img);
        //why is cause of this..
        //wtf
        // sleep(0.02);
//        waitKey(1);
#endif //USING_VIDEO
//        if (src.img.empty()){
//            continue;
//        }
        factory.produce(src);

    }
}


// Construct function "Aim".
Aim::Aim(double & max_match_distance,int & tracking_threshold,int & lost_threshold,double & max_match_yaw_diff,
         std::array<double, 9> & camera_matrix,vector<double> & dist_coeffs)://Member object initialization
        state_predictor(max_match_distance, tracking_threshold, lost_threshold,max_match_yaw_diff),solver(camera_matrix,dist_coeffs){}

bool consumer(Factory<Aimpack> &task_factory)
{
    Serial_receive send_port;
    Aimpack dst;
    Aim aim(max_Matchdistance, trackingThreshold, lostThreshold, maxMatchYawdiff,cameraMatrix, distCoeffs);
    auto time_start = std::chrono::steady_clock::now();
    while(1)
    {
        task_factory.consume(aim.src);
        auto time_end=std::chrono::steady_clock::now();
        aim.end_time = (std::chrono::duration<double,std::milli>(time_end - time_start).count());
        aim.dt=aim.end_time-aim.last_end_time;
        aim.run(aim.src);
        aim.last_end_time = aim.end_time;
//        if (waitKey(100) == 'q')
          if(waitKey(360)=='q')
            break;
    }
    return true;
}
void Aim::run(Aimpack src) {
   src.aim_mode=3;
    if(src.aim_mode > 1){
        rune_info = src.img.clone();

        // 0 small buff, 1 big buff.
        rune_predictor.mode = aim_mode - 2;
        rune(src);
        showRuneInfo();
        waitKey(500);
        cout<<"rune_pmod:"<<rune_predictor.mode<<endl;
        //?? dog toll
    }
    else{
        // Clear the params of buff.
        if(rune_predictor.params[0]!=0){
            for(int i=0;i<4;i++){
                rune_predictor.params[0]=0;
                rune_predictor.params[1]=0;
                rune_predictor.params[2]=0;
                rune_predictor.params[3]=0;
            }
        }
        armor_info = src.img.clone();
        // 0 loose, 1 strict.
        detector.more_strict_classification =1;
        armor(src);
        showArmorInfo();
    }
}



// TODO: ------------------------------ tool part ------------------------------// TODO: tool part
void Aim::showArmorInfo() {

    putText(armor_info, "mode: " + to_string(aim_mode),
            Point2f(0, 60), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(armor_info, "strict: " + to_string(detector.more_strict_classification),
            Point2f(0, 80), 5, 0.6, Scalar(0, 208, 255), 1);
    putText(armor_info, "speed: " + to_string(bullet_speed),
            Point2f(0, 300), 5, 0.6, Scalar(0, 208, 255), 1);


    // state
    if (!armors.empty()) {
        putText(armor_info, "ARMOR SEEN!", Point2f(0, 20),
                7, 0.8, Scalar(5, 57, 245), 2);

        putText(armor_info, armors[0].classification_result,
                Point2f(0, 40), 5, 0.6, Scalar(255, 233, 15), 1);

        putText(armor_info, "armor number: " + to_string(armors.size()),
                Point2f(80, 40), 5, 0.6, Scalar(255, 233, 15), 1);


#ifdef CV_INIT
//        putText(armor_info, "now yaw: " + to_string(predictor.now_euler.yaw),
//                Point2f(0, 180), 5, 0.6, Scalar(133, 245, 5), 1);
//
//        putText(armor_info, "now pitch: " + to_string(predictor.now_euler.pitch),
//                Point2f(0, 200), 5, 0.6, Scalar(133, 245, 5), 1);
//
        putText(armor_info, "distance: " + to_string(predictor.now_euler.distance),
                Point2f(0, 250), 5, 0.6, Scalar(133, 245, 5), 1);
//
//        putText(armor_info, "predict yaw: " + to_string(predictor.predict_euler.yaw),
//                Point2f(0, 240), 5, 0.6, Scalar(133, 245, 5), 1);
//
//        putText(armor_info, "predict pitch: " + to_string(predictor.predict_euler.pitch),
//                Point2f(0, 260), 5, 0.6, Scalar(133, 245, 5), 1);
#endif

    }
#ifdef OPEN_SERIAL

    // receive data
    putText(armor_info, "receive yaw: " + to_string(predictor.plat_state.yaw_angle),
            Point2f(0, 200), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(armor_info, "receive pitch: " + to_string(predictor.plat_state.pitch_angle),
            Point2f(0, 220), 5, 0.6, Scalar(201, 116, 201), 1);

    // send data
    putText(armor_info, "send yaw: " + to_string(send_data.yaw_angle),
            Point2f(400, 20), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(armor_info, "send pitch: " + to_string(send_data.pitch_angle),
            Point2f(400, 40), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(armor_info, "send distance: " + to_string(send_data.distance),
            Point2f(400, 60), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(armor_info, "detect flag: " + to_string(send_data.detect_flag),
            Point2f(400, 80), 5, 0.6, Scalar(0, 208, 255), 1);
#endif
    resize(armor_info,armor_info,Size(720,540),INTER_NEAREST);
    namedWindow("armor info",2);
    imshow("armor info", armor_info);

}


void Aim::showRuneInfo() {

    putText(rune_info, "rune mode: " + to_string(rune_predictor.mode),
            Point2f(0, 40), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(rune_info, "detect flag: " + to_string(1),
            Point2f(0, 160), 5, 0.6, Scalar(0, 208, 255), 1);

    putText(rune_info, "r2c distance: " + to_string(send_data.distance),
            Point2f(0, 140), 5, 0.6, Scalar(0, 208, 255), 1);

    if (!rune_detector.armors.empty()) {
        // state
        putText(rune_info, "RUNE SEEN!", Point2f(0, 20),
                7, 0.8, Scalar(5, 57, 245), 2);

        putText(rune_info, "predict yaw: " + to_string(send_data.yaw_angle),
                Point2f(0, 100), 5, 0.6, Scalar(0, 208, 255), 1);

        putText(rune_info, "predict pitch: " + to_string(send_data.pitch_angle),
                Point2f(0, 120), 5, 0.6, Scalar(0, 208, 255), 1);

    }

    putText(rune_info, "r center: " + string("x-") + to_string(r_center.x) + string("  y-") + to_string(r_center.y),
            Point2f(0, 180), 5, 0.6, Scalar(247, 219, 116), 1);

    putText(rune_info, "send yaw: " + to_string(send_data.yaw_angle),
            Point2f(400, 20), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(rune_info, "send pitch: " + to_string(send_data.pitch_angle),
            Point2f(400, 40), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(rune_info, "send distance: " + to_string(send_data.distance),
            Point2f(400, 60), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(rune_info, "detect flag: " + to_string(send_data.detect_flag),
            Point2f(400, 80), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(rune_info, "predict angle: " + to_string(pre_angle * 180 / CV_PI),
            Point2f(400, 220), 5, 0.6, Scalar(201, 116, 201), 1);

#ifdef OPEN_SERIAL
    // receive data
    putText(rune_info, "receive yaw: " + to_string(predictor.plat_state.yaw_angle),
            Point2f(0, 200), 5, 0.6, Scalar(201, 116, 201), 1);

    putText(rune_info, "receive pitch: " + to_string(predictor.plat_state.pitch_angle),
            Point2f(0, 220), 5, 0.6, Scalar(201, 116, 201), 1);
#endif
//    resize(rune_info,rune_info,Size(720,540),INTER_NEAREST);
    //why hwy why
    //why resize...
    imshow("rune info", rune_info);
}

bool Serial_receive::receive(Aimpack& pack) {
    bool receiveSuccess = receive_port.receiveData(R_NUM, 0xAA, 0xAF, r_data);
    if(receiveSuccess){
        pack.yaw_angle = r_data[1]*100+r_data[2]+(float)r_data[3]/100;//todo:iaosdjjdoa;ijdo;isjoi;

        if( r_data[4]-90 < 0){
            pack.pitch_angle = int(r_data[4]-90) + 1.0 - (float)(r_data[5]/100.0);
        }else{
            pack.pitch_angle = int(r_data[4]-90) + (float)(r_data[5]/100.0);
        }
        pack.aim_mode = r_data[6]/10;
        pack.enemy_color = int(r_data[6]%10);
//        rune_detector.enemy_color = RuneDetector::Color((r_data[6]%10+1)%2);
        pack.bullet_speed = double(r_data[7]) * 2 / 10;
        if(r_data[7]<=0||r_data[7]>30){
            pack.bullet_speed=27;
        }
//        if(bullet_speed==0){
//            bullet_speed=13.5;
//        }else if(bullet_speed-bullet_speed_queue[9]>3){
//            bullet_speed_queue.clear();
//            bullet_speed_queue.push_back(bullet_speed);
//        }
//        bullet_speed_queue.push_back(bullet_speed);
//        if(bullet_speed_queue.size() > 10) {
//            bullet_speed_queue.pop_front();
//        }
    }
    return receiveSuccess;
}

bool Serial_receive::openSerial(){
    bool usb = receive_port.initSerial("/dev/ttyRM", 115200, 'N', 8, 1);
    return usb;
}


/*
/*
/**
 * @brief 数据发送线程
 *
 * @param serial SerialPort类
 * @param transmit_factory Factory类
 * @return true
 * @return false
 */
/*
bool dataTransmitter(SerialPort &serial,Factory<VisionData> &transmit_factory)
{
    while(1)
    {
        VisionData transmit;
        transmit_factory.consume(transmit);
        //若串口离线则跳过数据发送
        //TODO:使用无串口的模式时会导致此线程死循环，浪费CPU性能
        if (serial.need_init == true)
        {
            // cout<<"offline..."<<endl;
            #ifndef DEBUG_WITHOUT_COM
                #ifdef SAVE_LOG_ALL
                    LOG(ERROR) << "[TRANSMITTER] Serial offline, trying to reconnect...";
                #endif //SAVE_LOG_ALL
            #endif //DEBUG_WITHOUT_COM
            usleep(5000);
            continue;
        }
        serial.TransformData(transmit);
        serial.send();
        // cout<<"transmitting..."<<endl;
    }
    return true;
}
*/
/*
#ifdef USING_IMU_C_BOARD
*/
/**
 * @brief 数据接收线程
 *
 * @param serial
 * @param receive_factory
 * @param time_start
 * @return true
 * @return false
 *//*

bool dataReceiver(SerialPort &serial, MessageFilter<MCUData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start)
{
    while(1)
    {
        //若串口离线则跳过数据发送
        if (serial.need_init == true)
        {
            // cout<<"offline..."<<endl;
            usleep(5000);
            continue;
        }
        //数据读取不成功进行循环
        while (!serial.get_Mode())
            ;
        auto time_cap = std::chrono::steady_clock::now();
        auto timestamp = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count());
        // cout<<"Quat: "<<serial.quat[0]<<" "<<serial.quat[1]<<" "<<serial.quat[2]<<" "<<serial.quat[3]<<" "<<endl;
        // Eigen::Quaterniond quat = {serial.quat[0],serial.quat[1],serial.quat[2],serial.quat[3]};
        //FIXME:注意此处mode设置
        int mode = serial.mode;
        float bullet_speed = serial.bullet_speed;

        // int mode = 2;
        Eigen::Quaterniond quat = {serial.quat[0],serial.quat[1],serial.quat[2],serial.quat[3]};
        Eigen::Vector3d acc = {serial.acc[0],serial.acc[1],serial.acc[2]};;
        Eigen::Vector3d gyro = {serial.gyro[0],serial.gyro[1],serial.gyro[2]};;
        MCUData mcu_status = {mode, acc, gyro, quat, bullet_speed, timestamp};
        receive_factory.produce(mcu_status, timestamp);
        // Eigen::Vector3d vec = quat.toRotationMatrix().eulerAngles(2,1,0);
        // cout<<"Euler : "<<vec[0] * 180.f / CV_PI<<" "<<vec[1] * 180.f / CV_PI<<" "<<vec[2] * 180.f / CV_PI<<endl;
        // cout<<"transmitting..."<<endl;
    }
    return true;
}
#endif //USING_IMU_C_BOARD

*/
/**
 * @brief 串口监视线程
 *
 * @param serial
 * @return true
 * @return false
 *//*

bool serialWatcher(SerialPort &serial)
{
    int last = 0;
#ifdef DEBUG_WITHOUT_COM
    #ifdef SAVE_TRANSMIT_LOG
    LOG(WARNING)<<"[SERIAL] Warning: You are not using Serial port";
    #endif //SAVE_TRANSMIT_LOG
#endif // DEBUG_WITHOUT_COM

    while(1)
    {
        sleep(1);
        //检测文件夹是否存在或串口需要初始化
        if (access(serial.device.path.c_str(),F_OK) == -1 || serial.need_init)
        {
            serial.need_init = true;
#ifdef DEBUG_WITHOUT_COM
            int now = clock()/CLOCKS_PER_SEC;
            if (now - last > 10)
            {
                last = now;
                fmt::print(fmt::fg(fmt::color::orange), "[SERIAL] Warning: You are not using Serial port\n");
            }
            serial.withoutSerialPort();
#else
            serial.initSerialPort();
#endif //DEBUG_WITHOUT_COM
        }
    }
}


// #ifdef USING_IMU_WIT
// bool dataReceiver(IMUSerial &serial_imu, MessageFilter<MCUData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start)
// {
//     while(1)
//     {
//         //若串口离线则跳过数据发送
//         if (serial_imu.need_init == true)
//         {
//             // cout<<"offline..."<<endl;
//             continue;
//         }
//         if (!serial_imu.readData())
//         {
//             continue;
//         }
//         auto time_cap = std::chrono::steady_clock::now();
//         auto timestamp = (int)(std::chrono::duration<double,std::milli>(time_cap - time_start).count());
//         if (!serial_imu.is_quat_initialized)
//         {
//             continue;
//         }
//         Eigen::Quaterniond quat = serial_imu.quat;
//         Eigen::Vector3d acc = serial_imu.acc;
//         Eigen::Vector3d gyro =serial_imu.gyro;
//         MCUData imu_status = {acc, gyro, quat, timestamp};

//         receive_factory.produce(imu_status, timestamp);
//         Eigen::Vector3d vec = quat.toRotationMatrix().eulerAngles(2,1,0);
//     }
//     return true;
// }

// bool serialWatcher(SerialPort &serial, IMUSerial &serial_imu)
// {
//     while(1)
//     {
//         sleep(0.1);
//         //检测文件夹是否存在或串口需要初始化
//         if (access(serial.device.path.c_str(),F_OK) == -1 || serial.need_init)
//         {
//             serial.need_init = true;
//             serial.initSerialPort();
//         }
//         if (access(serial_imu.device.path.c_str(),F_OK) == -1 || serial_imu.need_init)
//         {
//             serial_imu.need_init = true;
//             serial_imu.initSerialPort();
//         }

//     }
// }
// #endif //USING_WIT_IMU

// #ifndef USING_IMU
// bool serialWatcher(SerialPort &serial)
// {
//     while(1)
//     {
//         sleep(0.1);
//         //检测文件夹是否存在或串口需要初始化
//         if (access(serial.device.path.c_str(),F_OK) == -1 || serial.need_init)
//         {
//             serial.need_init = true;
//             serial.initSerialPort();
//         }

//     }
// }
// #endif //USING_IMU

*/






#endif