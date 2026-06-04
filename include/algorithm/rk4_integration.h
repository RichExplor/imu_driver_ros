#pragma once

#include "algorithm/quaternion.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace imu_algorithm {

/// @brief 四阶龙格-库塔（RK4）姿态积分算法（基于 Eigen 实现）
///
/// 使用 RK4 数值积分方法对四元数微分方程进行高精度积分：
///   dq/dt = 0.5 * q ⊗ ω
///
/// RK4 相比一阶欧拉法具有更高精度（O(dt^5) 截断误差），
/// 适合高动态运动或低采样率场景。
///
/// 支持 6 轴和 9 轴模式：
/// - 6 轴：仅使用陀螺仪积分，加速度计用于修正
/// - 9 轴：在 6 轴基础上，磁力计用于 yaw 修正
///
/// 注意：RK4 本身是纯积分方法，对于陀螺仪漂移无法自身消除。
/// 因此在 6/9 轴模式下，额外加入了加速度计/磁力计的互补修正步骤。
class RK4Integration {
public:
  /// @brief 轴数配置
  enum class AxisMode {
    SIX_AXIS  = 6, ///< 6 轴：加速度计 + 陀螺仪
    NINE_AXIS = 9  ///< 9 轴：加速度计 + 陀螺仪 + 磁力计
  };

  /// @brief 构造函数
  /// @param mode 轴数模式
  /// @param alpha_acc 加速度计修正系数（0~1）
  /// @param alpha_mag 磁力计修正系数（0~1）
  RK4Integration(AxisMode mode = AxisMode::NINE_AXIS, double alpha_acc = 0.02, double alpha_mag = 0.01);

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
  void EulerAngle(double& roll, double& pitch, double& yaw) const;

  /// @brief 重置姿态为单位四元数
  void Reset();

  /// @brief 设置加速度计修正系数
  void SetAlphaAcc(double alpha) {
    alpha_acc_ = alpha;
  }

  /// @brief 设置磁力计修正系数
  void SetAlphaMag(double alpha) {
    alpha_mag_ = alpha;
  }

  /// @brief 获取加速度计修正系数
  double GetAlphaAcc() const {
    return alpha_acc_;
  }

  /// @brief 获取磁力计修正系数
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

  /// @brief 设置加速度计异常检测阈值
  void SetAccelRejectionThreshold(double threshold) {
    accel_reject_threshold_ = threshold;
  }

private:
  /// @brief RK4 四元数积分核心
  /// @param gyro 角速度（rad/s，机体系）
  /// @param dt 时间步长（秒）
  void rk4Step(const Eigen::Vector3d& gyro, double dt);

  /// @brief 加速度计修正 roll/pitch
  void correctWithAccel(const Eigen::Vector3d& accel);

  /// @brief 磁力计修正 yaw
  void correctWithMag(const Eigen::Vector3d& mag);

  /// @brief 检查加速度计数据是否有效
  bool isAccelValid(const Eigen::Vector3d& accel) const;

private:
  Eigen::Quaterniond q_;                      ///< 当前姿态四元数
  AxisMode           mode_;                   ///< 轴数模式
  double             alpha_acc_;              ///< 加速度计修正系数
  double             alpha_mag_;              ///< 磁力计修正系数
  double             accel_reject_threshold_; ///< 加速度计异常检测阈值

  static constexpr double GRAVITY = 9.80665;
};

} // namespace imu_algorithm
