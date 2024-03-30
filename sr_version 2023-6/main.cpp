#include "tools.h"
#include "supervisor.h"
#include "./Thread/thread.h"
#include <thread>
//#define MAIN
//#define SAVE_VIDEO
//#define CAPTURE
#define PRODUCER_CONSUMER
//#define CAMERA_THREAD
//#define CV_INIT
//#define SHOW_SRC
#ifdef MAIN

int main() {

    HaiKangCamera HaiKang;

    HaiKang.StartDevice(0);
    // 设置分辨率
    HaiKang.SetResolution(640,480);

    HaiKang.SetFPS(400);
    //更新时间戳，设置时间戳偏移量
//    HaiKang.UpdateTimestampOffset(time_start);
    // 开始采集帧
    HaiKang.SetStreamOn();
    // 设置曝光事件
    HaiKang.SetExposureTime(2400);
    // 设置1
    HaiKang.SetGAIN(0, 10);
//     HaiKang.SetGAIN(1, 8);
//     HaiKang.SetGAIN(2, 8);
//    HaiKang.SetGAIN(3, 16);
    // 是否启用自动白平衡7
    // HaiKang.Set_BALANCE_AUTO(0);
    // manual白平衡 BGR->012
//    HaiKang.Set_BALANCE(0, 1690);
//    HaiKang.Set_BALANCE(1, 1024);
//    HaiKang.Set_BALANCE(2, 2022);
    HaiKang.GetImageParam(HaiKang.m_param);
//    Camera hk_camera;
//    hk_camera.initCamera(3000,16);
    Mat src;
    Aim aim(1000, 1000, 1000, cameraMatrix, distCoeffs);

#ifdef CV_INIT
    aim.detector.detect_color = BLUE;
    aim.rune_detector.detect_color = RuneDetector::Color::self_RED;
    aim.aim_mode = ANTI_SENTRY;
    aim.detector.more_strict_classification = 0;
#endif

#ifdef SAVE_VIDEO
    time_t systime;
    struct tm * tmp;
    time(&systime);
    char sstime[64];
    tmp = localtime(&systime);
    strftime(sstime,64,"%Y_%m_%d_%H%M%S",tmp);
    string videoName = "videoSrc_"+string(sstime);
    VideoWriter vw;
    vw.open("../Files/"+videoName+".avi",CV_FOURCC('M','J','P','G'),144,Size(640,480));
#endif

#ifdef OPEN_SERIAL
    while(aim.openSerial() == false);
#endif

    VideoCapture vd;
    vd.open("/home/j11218cpu/Videos/qwq/2023-02-27_21_06_411804289383.avi");
    long start,end;
    while (1) {
        GetNowTime(start);
//        hk_camera.readImage(src);
//        HaiKang.GetMat(src);
        GetNowTime(end);
        double ddt = (end - start) / 1000.0;
        printf("time: -- %.2f ms\n", ddt);
        vd >> src;
#ifdef OPEN_SERIAL
        aim.receive();
#endif
        GetNowTime(aim.end_time);
        aim.dt = (aim.end_time - aim.last_end_time) / 1000.0;
//        printf("time: -- %.2f ms\n", aim.dt);
        aim.run(src);

#ifdef SHOW_SRC
        imshow("src", src);
#endif
#ifdef SAVE_VIDEO
        vw.write(aim.rune_info);
#endif
        if (waitKey(100) == 'q')
            break;
        aim.last_end_time = aim.end_time;

    }
    return 0;
}
#endif


#ifdef CAPTURE
int main() {
    Camera hk_camera;
    hk_camerfa.initCamera(5000);
    Mat src;

#ifdef SAVE_VIDEO
    time_t systime;
    struct tm * tmp;
    time(&systime);
    char sstime[64];
    tmp = localtime(&systime);
    strftime(sstime,64,"%Y_%m_%d_%H%M%S",tmp);
    string videoName = "videoSrc_"+string(sstime);
    VideoWriter vw;
    vw.open("../Files/"+videoName+".avi",CV_FOURCC('M','J','P','G'),144,Size(640,480));
#endif
    int n=0;
    while (1) {
        hk_camera.readImage(src);

        imshow("src", src);

#ifdef SAVE_VIDEO
        vw.write(src);
#endif
        if (waitKey() == 'q')
            break;
        else if(waitKey()== 's'){
            imwrite("/home/avir/Pictures/pic2/"+to_string(n++)+".jpg",src);
            cout<<"save "<<n<<" success!"<<endl;
        }
        else{
            continue;
        }
    }
    return 0;
}
#endif


#ifdef PRODUCER_CONSUMER

int main()  {

        auto time_start = std::chrono::steady_clock::now();
        Factory<Aimpack> task_factory(5);
//        Factory<VisionData> data_transmit_factory(5);
//        MessageFilter<MCUData> data_receiver(100);

        std::thread task_producer(&producer, ref(task_factory), time_start);
        std::thread task_consumer(&consumer, ref(task_factory));

        task_producer.join();
        task_consumer.join();

        return 0;
}
#endif

#ifdef CAMERA_THREAD
int main() {
    camera::HikCamera MVS_cap;                // 海康相机对象
    VideoCapture capture;
    // 根据条件输入选择视频源 (1、海康相机  0、视频文件)
    int from_camera = 0;

    if (1) {
        MVS_cap.Init(); // 初始化海康相机
        MVS_cap.CamInfoShow();
    } else {
        string filename = "/videoTest/armor_red.avi";
        capture.open(filename);
//        LOGM("video_source initialization successfully.");
        if(!capture.isOpened()){
//            LOGW("video_source unavailable!");
            return -1;
        }
    }
    Aimpack src;
    Aim aim(max_Matchdistance, trackingThreshold, lostThreshold, maxMatchYawdiff,cameraMatrix, distCoeffs);

#ifdef CV_INIT
    aim.src.enemy_color = RED;
    aim.src.aim_mode = NORMAL_AIM;
    aim.detector.more_strict_classification = 1;
#endif

#ifdef SAVE_VIDEO
    time_t systime;
    struct tm * tmp;
    time(&systime);
    char sstime[64];
    tmp = localtime(&systime);
    strftime(sstime,64,"%Y_%m_%d_%H%M%S",tmp);
    string videoName = "videoSrc_"+string(sstime);
    VideoWriter vw;
    vw.open("../Files/"+videoName+".avi",CV_FOURCC('M','J','P','G'),144,Size(640,480));
#endif

#ifdef OPEN_SERIAL
    while(aim.openSerial() == false);
#endif

//    VideoCapture vd;
//    vd.open("/home/j11218cpu/Videos/qwq/2023-02-27_21_06_411804289383.avi");
    long start,end;
    while (1) {
        GetNowTime(start);
        if (1) {
            MVS_cap.ReadImg(src.img);
            if (src.img.empty())          // 海康相机初始化时开启线程需要一定时间,防止空图
                continue;
        } else {
            capture.read(src.img);
        }

//        vd >> src.img;
#ifdef OPEN_SERIAL
        aim.receive();
#endif
        cout<<1<<endl;
        aim.run(src);
        GetNowTime(aim.end_time);
        aim.dt = (aim.end_time - aim.last_end_time) / 1000.0;
//        printf("time: -- %.2f ms\n", aim.dt);
        imshow("src", src.img);

#ifdef SHOW_SRC
        imshow("src", src);
#endif
#ifdef SAVE_VIDEO
        vw.write(aim.rune_info);
#endif
        GetNowTime(end);
        double ddt = (end - start) / 1000.0;
        printf("time: -- %.2f ms\n", ddt);
        if (waitKey(1) == 'q')
            break;
        aim.last_end_time = aim.end_time;

    }
    return 0;
}
#endif