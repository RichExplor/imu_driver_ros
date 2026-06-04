#pragma once

#include "algorithm/attitude_estimator.h"
#include "imu_parser.h"
#include "imu_driver_ros/ImuData.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

/// @brief IMU 消息发布器，管理自定义消息和标准 sensor_msgs 的发布
///
/// 集成姿态解算模块，支持可配置的算法类型（互补滤波/RK4）和轴数模式（6轴/9轴），
/// 将原始 IMU 数据经过姿态解算后发布带有姿态四元数的消息。
class ImuPublisher {
public:
  /// @brief 构造函数
  /// @param nh 节点句柄（私有命名空间）
  /// @param publish_custom 是否发布自定义 ImuData 消息
  /// @param publish_sensor_msgs 是否发布 sensor_msgs/Imu 和 MagneticField
  /// @param frame_id 坐标系 ID
  /// @param estimator 姿态解算器（外部传入，共享所有权）
  ImuPublisher(ros::NodeHandle& nh, bool publish_custom, bool publish_sensor_msgs, const std::string& frame_id,
               std::shared_ptr<imu_algorithm::AttitudeEstimator> estimator);

  /// @brief 发布消息
  /// @param raw 解析后的原始数据
  /// @param stamp 时间戳
  void Publish(const ImuRawData& raw, const ros::Time& stamp);

private:
  /// @brief 填充 sensor_msgs/Imu 的协方差矩阵
  /// @param cov 9 元素数组
  /// @param unknown 是否将第一个元素设为 -1（表示方向未知）
  static void fillCovariance(double cov[9], bool unknown = false);

  /// @brief 将原始数据转换为 SI 单位，并应用缩放系数
  /// @param raw 输入原始数据
  /// @param stamp 当前时间戳（用于姿态解算）
  /// @param accel 输出线性加速度（m/s^2）
  /// @param gyro 输出角速度（rad/s）
  /// @param mag 输出磁场（T）
  /// @param quat 输出姿态四元数（如果有）
  void convertRawData(const ImuRawData& raw, const ros::Time& stamp, geometry_msgs::Vector3& accel,
                      geometry_msgs::Vector3& gyro, geometry_msgs::Vector3& mag, geometry_msgs::Quaternion& quat);

  /// @brief 使用姿态解算器计算当前姿态四元数
  /// @param accel 加速度计数据（m/s²，机体系）
  /// @param gyro 陀螺仪数据（rad/s，机体系）
  /// @param mag 磁力计数据（T，机体系）
  /// @param orientation 输出姿态四元数
  /// @param stamp 当前时间戳
  void attitudeEstimate(const geometry_msgs::Vector3& accel, const geometry_msgs::Vector3& gyro,
                        const geometry_msgs::Vector3& mag, geometry_msgs::Quaternion& orientation,
                        const ros::Time& stamp);

private:
  ros::Publisher pub_custom_;
  ros::Publisher pub_imu_;
  ros::Publisher pub_mag_;

  bool        publish_custom_;
  bool        publish_sensor_msgs_;
  std::string frame_id_;

  /// @brief 姿态解算器
  std::shared_ptr<imu_algorithm::AttitudeEstimator> estimator_ptr_;

  /// @brief 上一帧时间戳，用于计算 dt
  ros::Time last_stamp_;
  bool      first_frame_;

  /// @brief 协方差矩阵常量（未知方向设 -1，其余为 0）
  static constexpr double COV_UNKNOWN = -1.0;
  static constexpr double COV_ZERO    = 0.0;

  /// @brief 转换原始数据为 SI 单位（m/s^2, rad/s, T）
  static constexpr double GRAVITY = 9.80665; // 标准重力加速度

  // 新协议原始 DATA 需乘以 1e-6 得到物理量（除温度）
  static constexpr double SCALE_ACCEL           = 1e-6; // 加速度: DATA * SCALE_ACCEL -> m/s^2
  static constexpr double SCALE_GYRO            = 1e-6; // 角速度: DATA * SCALE_GYRO -> deg/s
  static constexpr double SCALE_MAG             = 1e-6; // 磁力归一化: DATA * SCALE_MAG -> 单位向量分量
  static constexpr double SCALE_MAG_STRENGTH    = 1e-3; // 磁场强度: DATA * SCALE_MAG_STRENGTH -> mGauss
  static constexpr double SCALE_NOT_DIMENSIONAL = 1e-6; // 无量纲缩放因子

  // 磁场强度转 Tesla：DATA * 0.001 (mGauss) -> Tesla = DATA * 0.001 * 1e-4 = DATA * 1e-7
  // (1 mGauss = 1e-4 Tesla, 即 1e-7 T/mGauss)
  static constexpr double SCALE_MAG_STRENGTH_TO_TESLA = 1e-7;

  /// @brief 角度转换常量
  static constexpr double Rad2Deg = 180.0 / M_PI;
  static constexpr double Deg2Rad = M_PI / 180.0;
};
