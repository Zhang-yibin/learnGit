
#ifndef RUNE_PREDICTOR__RUNE_PREDICTOR_HPP_
#define RUNE_PREDICTOR__RUNE_PREDICTOR_HPP_

// ROS
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include <ceres/ceres.h>
// #include <yaml-cpp/yaml.h>
#include <Eigen/Core>

#include <ctime>
#include <future>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <random>
#include <vector>

#include "rune_predictor/particle_filter.hpp"

class RunePredictor
{
private:
  struct CURVE_FITTING_COST
  {
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}
    // 残差的计算
    template <typename T>
    bool operator()(
      const T * params,    // 模型参数，有3维
      T * residual) const  // 残差
    {
      residual[0] = T(_y) - params[0] * ceres::sin(params[1] * T(_x) + params[2]) -
                    params[3];  // f(x) = a * sin(ω * t + θ) + b
      return true;
    }
    const double _x, _y;  // x,y数据
  };
  struct CURVE_FITTING_COST_PHASE
  {
    CURVE_FITTING_COST_PHASE(double x, double y, double a, double omega, double dc)
    : _x(x), _y(y), _a(a), _omega(omega), _dc(dc)
    {
    }
    // 残差的计算
    template <typename T>
    bool operator()(
      const T * phase,     // 模型参数，有1维
      T * residual) const  // 残差
    {
      residual[0] = T(_y) - T(_a) * ceres::sin(T(_omega) * T(_x) + phase[0]) -
                    T(_dc);  // f(x) = a * sin(ω * t + θ)
      return true;
    }
    const double _x, _y, _a, _omega, _dc;  // x,y数据
  };

  //目标信息
  struct TargetInfo
  {
    double speed;
    double dist;
    int timestamp;
  };

  struct PredictStatus
  {
    bool xyz_status[3];
  };

public:
  int history_deque_len_cos = 500;  //大符全部参数拟合队列长度
  int history_deque_len_phase = 100;    //大符相位参数拟合队列长度

public:
  double bullet_speed = 27;
  std::deque<TargetInfo> history_info;    //目标队列
  const int max_timespan = 20000;         //最大时间跨度，大于该时间重置预测器(ms)
  const double max_rmse = 0.5;            //TODO:回归函数最大Cost
  const int max_v = 3;                    //设置最大速度,单位rad/s
  const int max_a = 8;                    //设置最大角加速度,单位rad/s^2
  const int history_deque_len_uniform = 100;  //小符转速求解队列长度
  const int delay_small = 175;                //小符发弹延迟
  const int delay_big = 130;                  //大符发弹延迟
  const int window_size = 2;                  //滑动窗口大小

public:
  double params[4];
  TargetInfo last_target;  //最后目标
  ParticleFilter pf;
  ParticleFilter pf_param_loader;
  int mode;  //预测器模式，0为小符，1为大符
  int last_mode = 7;
  bool is_params_confirmed;

  double t_speed;
  double c_function;

  RunePredictor(int history_deque_len_cos_, int history_deque_len_phase_);

  bool predict(double speed, double dist, int timestamp, float & result);
  double calcAimingAngleOffset(double params[4], double t0, double t1, int mode);
  double shiftWindowFilter(int start_idx);
  bool setBulletSpeed(double speed);
  double evalRMSE(double params[4]);
  double evalMAPE(double params[4]);
};

struct Rune {
  double time_Sum;

  float real_angle;
  float spin_speed;
  float pre_angle;
  float rune_distance;
  float r2c_dist = 6.0;
  cv::Point2f pre_center;
};

#endif // RUNE_PREDICTOR__RUNE_PREDICTOR_HPP_
