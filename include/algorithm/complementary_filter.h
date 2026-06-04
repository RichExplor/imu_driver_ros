#pragma once

#include "algorithm/quaternion.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace imu_algorithm {

/// @brief 互补滤波姿态解算算法（基于 Eigen 实现）
///
/// 支持 6 轴（加速度计 + 陀螺仪）和 9 轴（加速度计 + 陀螺仪 + 磁力计）模式。
///
/// 6 轴模式：利用加速度计重力方向修正陀螺仪漂移的 roll/pitch，
///           yaw 仅靠陀螺仪积分（会漂移）。
///
/// 9 轴模式：在 6 轴基础上，利用磁力计修正 yaw 漂移，
///           实现全姿态（roll/pitch/yaw）无漂移估计。
///
/// 算法原理：
///   1. 陀螺仪积分得到预测四元数
///   2. 从加速度计（和磁力计）计算参考姿态
///   3. 计算预测姿态与参考姿态的误差
///   4. 用互补滤波系数融合误差修正预测
class ComplementaryFilter {
public:
  /// @brief 轴数配置
  enum class AxisMode {
    SIX_AXIS  = 6, ///< 6 轴：加速度计 + 陀螺仪
    NINE_AXIS = 9  ///< 9 轴：加速度计 + 陀螺仪 + 磁力计
  };

  /// @brief 构造函数
  /// @param mode 轴数模式（6 轴或 9 轴）
  /// @param alpha_acc 加速度计融合系数（0~1），越大越信任加速度计
  /// @param alpha_mag 磁力计融合系数（0~1），越大越信任磁力计
  ComplementaryFilter(AxisMode mode = AxisMode::NINE_AXIS, double alpha_acc = 0.02, double alpha_mag = 0.01);

  /// @brief 更新姿态（6轴接口）
  /// @param gx 陀螺仪 x 角速度（rad/s，机体系）
  /// @param gy 陀螺仪 y 角速度（rad/s，机体系）
  /// @param gz 陀螺仪 z 角速度（rad/s，机体系）
  /// @param ax 加速度计 x（m/s²，机体系）
  /// @param ay 加速度计 y（m/s²，机体系）
  /// @param az 加速度计 z（m/s²，机体系）
  /// @param dt 时间步长（秒）
  void Update(double gx, double gy, double gz, double ax, double ay, double az, double dt);

  /// @brief 更新姿态（9轴接口）
  /// @param gx 陀螺仪 x 角速度（rad/s，机体系）
  /// @param gy 陀螺仪 y 角速度（rad/s，机体系）
  /// @param gz 陀螺仪 z 角速度（rad/s，机体系）
  /// @param ax 加速度计 x（m/s²，机体系）
  /// @param ay 加速度计 y（m/s²，机体系）
  /// @param az 加速度计 z（m/s²，机体系）
  /// @param mx 磁力计 x（任意单位，机体系）
  /// @param my 磁力计 y（任意单位，机体系）
  /// @param mz 磁力计 z（任意单位，机体系）
  /// @param dt 时间步长（秒）
  void Update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz,
              double dt);

  /// @brief 更新姿态（Eigen 向量接口，6轴）
  /// @param gyro 陀螺仪角速度（rad/s，机体系）
  /// @param accel 加速度计（m/s²，机体系）
  /// @param dt 时间步长（秒）
  void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt);

  /// @brief 更新姿态（Eigen 向量接口，9轴）
  /// @param gyro 陀螺仪角速度（rad/s，机体系）
  /// @param accel 加速度计（m/s²，机体系）
  /// @param mag 磁力计（任意单位，机体系）
  /// @param dt 时间步长（秒）
  void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag, double dt);

  /// @brief 获取当前姿态四元数
  const Eigen::Quaterniond& Quaternion() const {
    return q_;
  }

  /// @brief 获取欧拉角
  /// @param roll  横滚角（弧度）
  /// @param pitch 俯仰角（弧度）
  /// @param yaw   偏航角（弧度）
  void EulerAngle(double& roll, double& pitch, double& yaw) const;

  /// @brief 重置姿态为单位四元数
  void Reset();

  /// @brief 设置加速度计融合系数
  void SetAlphaAcc(double alpha) {
    alpha_acc_ = alpha;
  }

  /// @brief 设置磁力计融合系数
  void SetAlphaMag(double alpha) {
    alpha_mag_ = alpha;
  }

  /// @brief 获取加速度计融合系数
  double GetAlphaAcc() const {
    return alpha_acc_;
  }

  /// @brief 获取磁力计融合系数
  double GetAlphaMag() const {
    return alpha_mag_;
  }

  /// @brief 设置轴数模式
  void SetAxisMode(AxisMode mode) {
    mode_ = mode;
  }

  /// @brief 获取轴数模式
  AxisMode GetAxisMode() const {
    return mode_;
  }

  /// @brief 设置加速度计异常检测阈值（比例，0~1）
  void SetAccelRejectionThreshold(double threshold) {
    accel_reject_threshold_ = threshold;
  }

private:
  /// @brief 从加速度计计算重力方向误差，修正 roll/pitch
  void correctWithAccel(const Eigen::Vector3d& accel);

  /// @brief 从磁力计计算磁场方向误差，修正 yaw
  void correctWithMag(const Eigen::Vector3d& mag);

  /// @brief 检查加速度计数据是否有效（模长接近重力）
  bool isAccelValid(const Eigen::Vector3d& accel) const;

private:
  Eigen::Quaterniond q_;                      ///< 当前姿态四元数
  AxisMode           mode_;                   ///< 轴数模式
  double             alpha_acc_;              ///< 加速度计融合系数
  double             alpha_mag_;              ///< 磁力计融合系数
  double             accel_reject_threshold_; ///< 加速度计异常检测阈值

  static constexpr double GRAVITY = 9.80665;
};

} // namespace imu_algorithm
