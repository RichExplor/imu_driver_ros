#pragma once

#include "algorithm/attitude_estimator.h"
#include "imu_parser.h"
#include "imu_publisher.h"
#include "serial_port.h"
#include <memory>
#include <ros/ros.h>

class ImuDriverNode {
public:
  /// @brief 构造函数
  ImuDriverNode(ros::NodeHandle& nh);

  bool Init();

  void Run();

  void Shutdown();

private:
  void loadParams();

  void publishDiagnostics();

private:
  ros::NodeHandle nh_;

  std::unique_ptr<SerialPort>                       serial_ptr_;
  std::unique_ptr<ImuParser>                        parser_ptr_;
  std::unique_ptr<ImuPublisher>                     publisher_ptr_;
  std::shared_ptr<imu_algorithm::AttitudeEstimator> attitude_estimator_ptr_;

  // 串口参数
  std::string port_;
  int         baud_;
  int         timeout_ms_;

  // 发布参数
  bool        publish_custom_;
  bool        publish_sensor_msgs_;
  std::string frame_id_;

  // 姿态解算参数
  bool        enable_attitude_estimation_; ///< 是否启用姿态解算
  std::string algorithm_type_;             ///< 算法类型："complementary" 或 "rk4"
  std::string axis_mode_;                  ///< 轴数模式："6" 或 "9"
  double      alpha_acc_;                  ///< 加速度计融合系数
  double      alpha_mag_;                  ///< 磁力计融合系数

  // 诊断统计
  size_t    total_frames_;
  size_t    failed_frames_;
  ros::Time last_diag_time_;
};
