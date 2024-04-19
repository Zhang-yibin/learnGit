
#include "rune_predictor/rune_predictor.hpp"

#include <fstream>

RunePredictor::RunePredictor(int history_deque_len_cos_, int history_deque_len_phase_)
{
  is_params_confirmed = false;
  params[0] = 0;
  params[1] = 0;
  params[2] = 0;
  params[3] = 0;
  history_deque_len_cos = history_deque_len_cos_;
  history_deque_len_phase = history_deque_len_phase_;
  // YAML::Node config = YAML::LoadFile(
    // "/home/zjr/rv_ws/src/rm_auto_aim/armor_tracker/docs/filtert_param.yaml");
  pf_param_loader.initParam();
}


/**
 * @brief 预测
 * @param speed 旋转速度
 * @param dist 距离
 * @param timestamp 时间戳
 * @param result 结果输出
 * @return 是否预测成功
*/
bool RunePredictor::predict(double speed, double dist, int timestamp, float & result)
{
  // auto t1 = std::chrono::steady_clock::now();
  TargetInfo target = {speed, dist, timestamp};
 
  //当时间跨度过长视作目标已更新，需清空历史信息队列
  if (
    history_info.size() == 0 || target.timestamp - history_info.front().timestamp >= max_timespan) {
    history_info.clear();
    history_info.push_back(target);
    params[0] = 0;
    params[1] = 0;
    params[2] = 0;
    params[3] = 0;
    pf.initParam(pf_param_loader);
    last_target = target;
    is_params_confirmed = false;
    return false;
  }
  //输入数据前进行滤波
  auto is_ready = pf.is_ready;
  Eigen::VectorXd measure(1);
  measure << speed;
  pf.update(measure);
  // std::cout<<"2q"<<std::endl;
  if (is_ready) {
    auto predict = pf.predict();
    target.speed = predict[0];
    t_speed = target.speed;
  }

  size_t deque_len = 0;
  mode =1;
  if (mode == 0) {
    deque_len = history_deque_len_uniform;
  } else if (mode == 1) {
    if (!is_params_confirmed)
      deque_len = history_deque_len_cos;
    else
      deque_len = history_deque_len_phase;
  }

  if (history_info.size() < deque_len) {
    history_info.push_back(target);
    last_target = target;
    return false;
  } else if (history_info.size() == deque_len) {
    history_info.pop_front();
    history_info.push_back(target);
  } else if (history_info.size() > deque_len) {
    while (history_info.size() >= deque_len) history_info.pop_front();
    history_info.push_back(target);
  }

  //计算旋转方向
  double rotate_speed_sum = 0;
  int rotate_sign = 0;
  for (auto target_info : history_info) rotate_speed_sum += target_info.speed;
  auto mean_velocity = rotate_speed_sum / history_info.size();

  //TODO:小符模式不需要额外计算,也可增加判断，小符模式给定恒定转速进行击打
  if (mode == 0) {
    params[3] = mean_velocity;
  }
  //若为大符
  else if (mode == 1) {
    //拟合函数: f(x) = a * sin(ω * t + θ) + b， 其中a， ω， θ需要拟合.
    //参数未确定时拟合a， ω， θ
    if (!is_params_confirmed) {
      std::cout<<"ew"<<std::endl;
      ceres::Problem problem;
      ceres::Solver::Options options;
      ceres::Solver::Summary summary;  // 优化信息
      double params_fitting[4] = {1, 1, 1, mean_velocity};

      // TODO: maybe is wrong
      //旋转方向，逆时针为正
      if (rotate_speed_sum / fabs(rotate_speed_sum) >= 0)
        rotate_sign = 1;
      else
        rotate_sign = -1;

      for (auto target_info : history_info) {
        problem.AddResidualBlock(  // 向问题中添加误差项
          // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
          new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4>(new CURVE_FITTING_COST(
            (float)(target_info.timestamp) / 1e3, target_info.speed * rotate_sign)),
          new ceres::CauchyLoss(0.5),
          params_fitting  // 待估计参数
        );
      }

      //设置上下限
      //FIXME:参数需根据场上大符实际调整
            // problem.SetParameterLowerBound(params_fitting,0,0.7);
            // problem.SetParameterUpperBound(params_fitting,0,1.2);
            // problem.SetParameterLowerBound(params_fitting,1,1.6);
            // problem.SetParameterUpperBound(params_fitting,1,2.2);
            // problem.SetParameterLowerBound(params_fitting,2,-CV_PI);
            // problem.SetParameterUpperBound(params_fitting,2,CV_PI);
            // problem.SetParameterLowerBound(params_fitting,3,0.5);
            // problem.SetParameterUpperBound(params_fitting,3,2.5);

      ceres::Solve(options, &problem, &summary);
      double params_tmp[4] = {
        params_fitting[0] * rotate_sign, params_fitting[1], params_fitting[2],
        params_fitting[3] * rotate_sign};
      auto rmse = evalRMSE(params_tmp);
      if (rmse > max_rmse) {
        // RCLCPP_DEBUG(rclcpp::get_logger("rune_predictor"), "%s", summary.BriefReport());
        return false;
      } else {
        params[0] = params_fitting[0] * rotate_sign;
        params[1] = params_fitting[1];
        params[2] = params_fitting[2];
        params[3] = params_fitting[3] * rotate_sign;
        is_params_confirmed = true;
        
      }
    }
    //参数确定时拟合θ
    else {
      // std::cout<<"qwq"<<std::endl;
      ceres::Problem problem;
      ceres::Solver::Options options;
      ceres::Solver::Summary summary;  // 优化信息
      double phase;

      for (auto target_info : history_info) {
        problem.AddResidualBlock(  // 向问题中添加误差项
          // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
          new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_PHASE, 1, 1>(
            new CURVE_FITTING_COST_PHASE(
              (float)(target_info.timestamp) / 1e3, (target_info.speed - params[3]) * rotate_sign,
              params[0], params[1], params[3])),
          new ceres::CauchyLoss(1e1),
          &phase  // 待估计参数
        );
      }

      //设置上下限
      problem.SetParameterLowerBound(&phase, 0, -CV_PI);
      problem.SetParameterUpperBound(&phase, 0, CV_PI);

      ceres::Solve(options, &problem, &summary);
      double params_new[4] = {params[0], params[1], phase, params[3]};
      auto old_rmse = evalRMSE(params);
      auto new_rmse = evalRMSE(params_new);
      if (new_rmse < old_rmse) {
        params[2] = phase;
      }
    }
  }

  /*    for (auto param : params)
        cout<<param<<" ";*/

  // int delay = (mode == 1 ? delay_big : delay_small);
  float delta_time_estimate = ((double)dist / bullet_speed) * 1e3 + delay_big;
  float timespan = history_info.back().timestamp;
  float time_estimate = delta_time_estimate + timespan;

  result = calcAimingAngleOffset(params, timespan / 1e3, time_estimate / 1e3, mode);
  last_target = target;
  c_function = params[0] * sin(params[1] * (double)timestamp / 1e3 + params[2]) + params[3];

  return true;
}

/**
 * @brief 计算角度提前量
 * @param params 拟合方程参数
 * @param t0 积分下限
 * @param t1 积分上限
 * @param mode 模式
 * @return 角度提前量(rad)
*/
double RunePredictor::calcAimingAngleOffset(double params[4], double t0, double t1, int mode)
{
  auto a = params[0];
  auto omega = params[1];
  auto theta = params[2];
  auto b = params[3];
  double theta1;
  double theta0;
  // f(x) = a * sin(ω * t + θ) + b
  // 对目标函数进行积分
  // 适用于小符模式
  if (mode == 0) {
    theta0 = b * t0;
    theta1 = b * t1;
  } else {
    theta0 = (b * t0 - (a / omega) * cos(omega * t0 + theta));
    theta1 = (b * t1 - (a / omega) * cos(omega * t1 + theta));
  }
  return theta1 - theta0;
}

/**
 * @brief 滑窗滤波
 *
 * @param start_idx 开始位置
 * @return double 滤波结果
 */
inline double RunePredictor::shiftWindowFilter(int start_idx = 0)
{
  //TODO:修改传入参数，由 start_idx 改为 max_iter
  //计算最大迭代次数
  auto max_iter = int(history_info.size() - start_idx) - window_size + 1;

  if (max_iter <= 0 || start_idx < 0) return history_info.back().speed;
  double total_sum = 0;
  for (int i = 0; i < max_iter; i++) {
    double sum = 0;
    for (int j = 0; j < window_size; j++) sum += history_info.at(start_idx + i + j).speed;
    total_sum += sum / window_size;
  }
  return total_sum / max_iter;
}

/**
 * @brief 设置弹速
 *
 * @param speed 传入弹速
 * @return true
 * @return false
 */
bool RunePredictor::setBulletSpeed(double speed)
{
  bullet_speed = speed;
  return true;
}

/**
 * @brief 计算RMSE指标
 *
 * @param params 参数首地址指针
 * @return RMSE值
 */
double RunePredictor::evalRMSE(double params[4])
{
  double rmse_sum = 0;
  double rmse = 0;
  for (auto target_info : history_info) {
    auto t = (float)(target_info.timestamp) / 1e3;
    auto pred = params[0] * sin(params[1] * t + params[2]) + params[3];
    auto measure = target_info.speed;
    rmse_sum += pow((pred - measure), 2);
  }
  rmse = sqrt(rmse_sum / history_info.size());
  return rmse;
}

/**
 * @brief 计算RMSE指标
 *
 * @param params 参数首地址指针
 * @return RMSE值
 */
double RunePredictor::evalMAPE(double params[4])
{
  double mape_sum = 0;
  double mape = 0;
  for (auto target_info : history_info) {
    auto t = (float)(target_info.timestamp) / 1e3;
    auto pred = params[0] * sin(params[1] * t + params[2]) + params[3];
    auto measure = target_info.speed;

    mape_sum += abs((measure - pred) / measure);
  }
  return mape;
}
