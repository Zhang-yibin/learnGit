
#include "detector.h"
#include "EKF_Predictor.h"
#include "StateEstimation/state_predict.h"
#include "StateEstimation/tracker.hpp"
#include "pnpsolver.h"
#include "serial.h"
#include "rune_detector.h"
#include "rune_predictor.h"
#include "HaiKangCamera.h"
#include <iterator>
#include <thread>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <iostream>
#include <atomic>
#include <opencv2/opencv.hpp>

// #include <Eigen/Core>

using namespace std;
using namespace cv;

typedef struct Aimpack{
    cv::Mat img;
    double timestamp;
    double yaw_angle;
    double pitch_angle;
    double bullet_speed;
    int aim_mode;
    int enemy_color;
}Aimpack;


class Aim{

public:
    double bullet_speed = 27;
    unsigned char aim_mode = ANTI_TOP;
    long end_time = 0;
    long last_end_time = 0;
    double real_angle=0;
    double super_angle=0;
    Aimpack src;
    void run(Aimpack src);

// TODO: rune part
public:
    RuneDetector rune_detector;
    RunePredictor rune_predictor;

    RotatedRect rune_armor;
    float spin_speed = 0;
    float radian;
    float last_radian = 0;
    Point2f pre_center;
    double pre_angle = 0;
    double rune_distance;
    Point2f r_center;
    Mat rune_debug;

    queue<float> rotation_judge;
    int rotation_flag=0;
    int speed_positive=0;
    int speed_negative=0;

    void rune(Aimpack src);

    deque<double> bullet_speed_queue;

private:
    Mat ceres_debug = Mat::zeros(200,1500,CV_8UC3);
    float time_sum = 0;
    float r2c_distance = 6.000;

// TODO: armor part
public:

    vector<Armor> armors;
    Point3f pt3f;

    float dt = 0;

    Detector detector;
    StatePredict state_predictor;
    PnPSolver solver;
    EKF_Predictor predictor;

    SerialPort serial_port;
    SendData send_data;

    unsigned char r_data[R_NUM];

public:
    Aim(double & max_match_distance,int & tracking_threshold,int & lost_threshold,double & max_match_yaw_diff,
        std::array<double, 9> & camera_matrix,vector<double> & dist_coeffs);


    void armor(Aimpack src);

private:
    int targetFollowing();



// TODO: serial part
public:
//    bool openSerial();
    bool send();
//    bool receive();
private:
    bool makeSendData();

// TODO: tools part
public:

    Mat armor_info,rune_info;
    void showArmorInfo();
    void showRuneInfo();

private:

    Mat filter_debug = Mat::zeros(500,1500,CV_8UC3);
    float timeSum = 0.0;

    int max_iter=10;
    float stop_error=0.001;
    int R_K_iter=50;
//    double bullet_speed_ = 28;
    // double bullet_speed = 16;            //TODO:弹速可变
//    const double k = 0.01903;
    const double k = 0.02903;    //25°C,1atm,小弹丸
    // const double k = 0.000556;                //25°C,1atm,大弹丸
    // const double k = 0.000530;                //25°C,1atm,发光大弹丸
    const double g = 9.788;

    double dynamicCalcPitchOffset(Eigen::Vector3d &xyz)
    {
        bullet_speed=25;
        //TODO:根据陀螺仪安装位置调整距离求解方式
        //降维，坐标系Y轴以垂直向上为正方向
//        cout<<"x"<<xyz[0]<<endl;
//        cout<<"y"<<xyz[1]<<endl;
//        cout<<"z"<<xyz[2]<<endl;
        auto dist_vertical = xyz[2];
        auto vertical_tmp = dist_vertical;
//        auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
//        dist_horizonal = xyz[2];
//        dist_horizonal=6;
        // auto dist_vertical = xyz[2];
         auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
//        cout<<"dist"<<dist_horizonal<<endl;
//        cout<<"qwq          "<<dist_horizonal<<endl;
//        cout<<"speed:::::::::::"<<bullet_speed<<endl;
        auto pitch = atan(dist_vertical / dist_horizonal) * 180 / CV_PI;
        auto pitch_new = pitch;
        auto pitch_offset = 0.0;

//        cout<<endl<<endl<<"speed::::"<<bullet_speed<<endl;
        //开始使用龙格库塔法求解弹道补偿
        for (int i = 0; i < max_iter; i++){
            //TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
            //初始化
            auto x = 0.0;
            auto y = 0.0;
            auto p = tan(pitch_new / 180 * CV_PI);
            auto v = bullet_speed;
            auto u = v / sqrt(1 + pow(p,2));
            auto delta_x = dist_horizonal / R_K_iter;
            for (int j = 0; j < R_K_iter; j++){
                auto k1_u = -k * u * sqrt(1 + pow(p, 2));
                auto k1_p = -g / pow(u, 2);
                auto k1_u_sum = u + k1_u * (delta_x / 2);
                auto k1_p_sum = p + k1_p * (delta_x / 2);

                auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
                auto k2_p = -g / pow(k1_u_sum, 2);
                auto k2_u_sum = u + k2_u * (delta_x / 2);
                auto k2_p_sum = p + k2_p * (delta_x / 2);

                auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
                auto k3_p = -g / pow(k2_u_sum, 2);
                auto k3_u_sum = u + k3_u * (delta_x / 2);
                auto k3_p_sum = p + k3_p * (delta_x / 2);

                auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
                auto k4_p = -g / pow(k3_u_sum, 2);

                u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
                p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

                x+=delta_x;
                y+=p * delta_x;
            }
            //评估迭代结果,若小于迭代精度需求则停止迭代
            auto error = dist_vertical - y;
            if (abs(error) <= stop_error){
                break;
            }
            else{
                vertical_tmp+=error;
                // xyz_tmp[1] -= error;
                pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / CV_PI;
            }
        }
        return pitch_new - pitch;
    }
    float bulletDroppingCompensation(float distance) {
        /* angle * PI / 180.0 */
        float sita = 0.0;
        if(distance <= 0)
            return 0.0;
        float t = 2 * bullet_speed *  bullet_speed / 9.8 / distance;
        sita = asin((sqrt(t * t + 4) - t) / 2) * 180.0 / CV_PI;
        return sita;
    }
};


class Serial_receive{
    SerialPort receive_port;
    unsigned char r_data[R_NUM];
public:
    bool receive(Aimpack& pack);
    bool openSerial();
};








template <typename T>
class Factory
{
private:
    std::deque<T> buffer;// 存储产品的双端队列
    int buffer_size;// 队列长度
    mutex lock;// 互斥锁，用于在多线程环境中保护对队列的访问

public:
    /**
     * @brief 工厂类初始化
     * @param size 队列长度
    **/
    Factory(int size)
    {
        buffer_size = size;
    };
    bool produce(T &product);
    bool consume(T &product);
};

template <typename T>
bool Factory<T>::produce(T &product)
{

    lock.lock();
    if (buffer.size() < buffer_size)
        buffer.push_back(product);// 如果队列未满，则将产品放入队列的末尾
    else
    {
        buffer.pop_front();// 如果队列已满，则移除队列头部的产品
        buffer.push_back(product);// 将新的产品放入队列的末尾
    
    }
    lock.unlock();

    return true;
}

template <typename T>
bool Factory<T>::consume(T &product)
{
    while (1)
    {
        lock.lock();
        if (!buffer.empty()){
            break;
        }

        lock.unlock();
        usleep(1e3);
    }
    product = buffer.front();
    buffer.pop_front();
    lock.unlock();

    return true;
}
//-----------------------------------------------------------------
template <typename T>
class MessageFilter
{
private:
    struct Product
    {
        T message;
        int timestamp;
    };
    std::deque<Product> buffer;
    atomic_bool is_editing;
    mutex lock;
    int buffer_size;
public:
    /**
     * @brief 工厂类初始化
     * @param size 队列长度
    **/
    MessageFilter(int size)
    {
        buffer_size = size;
        is_editing = false;
    };
    bool produce(T &message, int timestamp);
    bool consume(T &message, int timestamp);
};

template <typename T>
bool MessageFilter<T>::produce(T &message, int timestamp)
{
    lock.lock();
    Product product = {message, timestamp};
    if (buffer.size() < buffer_size)
        buffer.push_back(product);
    else
    {
        buffer.pop_front();
        buffer.push_back(product);
    }
    lock.unlock();

    return true;
}

template <typename T>
bool MessageFilter<T>::consume(T &message, int timestamp)
{
    //队列为空时阻塞消费者
    while (1)
    {
        lock.lock();
        if (!buffer.empty())
            break;
        lock.unlock();
        usleep(1e3);
    }
    // int cnt = 0;
    // for (auto info : buffer)
    // {
        
    //     cout<<cnt++<<" : "<<info.timestamp<<endl;
    // }
    auto it = std::lower_bound(buffer.begin(), buffer.end(), timestamp, [](Product &prev, const int &timestamp)
                               { return prev.timestamp < timestamp; });
    if (it == buffer.end())
    {
        //时间戳时间差大于10ms则认为该帧不可用
        if (abs((buffer.back().timestamp - timestamp)) > 10)
        {
            buffer.pop_front();
            lock.unlock();
            return false;            
        }
        else
        {
            message = (buffer.back()).message;
            buffer.pop_front();
            lock.unlock();
            return true;
        }
    }
    else
    {
        it--;
        message = (*it).message;
        buffer.erase(it);
    }
    lock.unlock();
    // cout<<(*it).timestamp<<":"<<timestamp<<"|"<<buffer.size()<<endl;
    // cout<<"///////////////////////////////////"<<endl;
    return true;
}

[[noreturn]] bool producer(Factory<Aimpack> &factory, std::chrono::_V2::steady_clock::time_point time_start);
bool consumer(Factory<Aimpack> &task_factory);
//bool dataTransmitter(SerialPort &serial, Factory<VisionData> &transmit_factory);

#ifdef USING_IMU_C_BOARD
bool dataReceiver(SerialPort &serial, MessageFilter<MCUData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start);
bool serialWatcher(SerialPort &serial);
#endif // USING_IMU_C_BOARD
#ifdef USING_IMU_WIT
bool dataReceiver(IMUSerial &serial_imu, MessageFilter<MCUData> &receive_factory, std::chrono::_V2::steady_clock::time_point time_start);
bool serialWatcher(SerialPort &serial, IMUSerial &serial_imu);
#endif // USING_IMU_WIT
#ifndef USING_IMU
//bool serialWatcher(SerialPort &serial);
#endif // USING_IMU