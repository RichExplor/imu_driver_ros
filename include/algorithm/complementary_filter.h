#pragma once

#include "algorithm/quaternion.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace imu_algorithm {

/// @brief 互补滤波姿态解算算法（基于 Eigen 实现）
/// 算法原理：
///   1. 陀螺仪积分得到预测四元数
///   2. 从加速度计（和磁力计）计算参考姿态
///   3. 计算预测姿态与参考姿态的误差
///   4. 用互补滤波系数融合误差修正预测
class ComplementaryFilter {
public:
  enum class AxisMode {
    SIX_AXIS  = 6, ///< 6 轴：加速度计 + 陀螺仪
    NINE_AXIS = 9  ///< 9 轴：加速度计 + 陀螺仪 + 磁力计
  };

  ComplementaryFilter(AxisMode mode = AxisMode::NINE_AXIS, double alpha_acc = 0.02, double alpha_mag = 0.01);

  /// @brief 更新姿态（6轴接口）
  void Update(double gx, double gy, double gz, double ax, double ay, double az, double dt);

  /// @brief 更新姿态（9轴接口）
  void Update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz,
              double dt);

  /// @brief 更新姿态（Eigen 向量接口，6轴）
  void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt);

  /// @brief 更新姿态（Eigen 向量接口，9轴）
  void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag, double dt);

  const Eigen::Quaterniond& Quaternion() const {
    return q_;
  }

  void EulerAngle(double& roll, double& pitch, double& yaw) const;

  void Reset();

  void SetAlphaAcc(double alpha) {
    alpha_acc_ = alpha;
  }

  void SetAlphaMag(double alpha) {
    alpha_mag_ = alpha;
  }

  double GetAlphaAcc() const {
    return alpha_acc_;
  }

  double GetAlphaMag() const {
    return alpha_mag_;
  }

  void SetAxisMode(AxisMode mode) {
    mode_ = mode;
  }

  AxisMode GetAxisMode() const {
    return mode_;
  }

  void SetAccelRejectionThreshold(double threshold) {
    accel_reject_threshold_ = threshold;
  }

private:
  void correctWithAccel(const Eigen::Vector3d& accel);

  void correctWithMag(const Eigen::Vector3d& mag);

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
