#pragma once

#include "algorithm/complementary_filter.h"
#include "algorithm/rk4_integration.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <string>

namespace imu_algorithm {

/// @brief 姿态解算管理器，提供可配置的算法和轴数选择（基于 Eigen 实现）
///
/// 支持两种算法：
/// - 互补滤波（ComplementaryFilter）：计算量小，适合嵌入式场景
/// - 四阶龙格库塔（RK4Integration）：高精度积分，适合高动态场景
///
/// 支持两种轴数模式：
/// - 6 轴（加速度计 + 陀螺仪）：yaw 会漂移
/// - 9 轴（加速度计 + 陀螺仪 + 磁力计）：全姿态无漂移
class AttitudeEstimator {
public:
  enum class AlgorithmType {
    COMPLEMENTARY, ///< 互补滤波
    RK4            ///< 四阶龙格库塔
  };

  enum class AxisMode {
    SIX_AXIS  = 6, ///< 6 轴
    NINE_AXIS = 9  ///< 9 轴
  };

  AttitudeEstimator(AlgorithmType algo = AlgorithmType::COMPLEMENTARY, AxisMode axis_mode = AxisMode::NINE_AXIS,
                    double alpha_acc = 0.02, double alpha_mag = 0.01);

  /// @brief 更新姿态（6轴，标量接口）
  void Update(double gx, double gy, double gz, double ax, double ay, double az, double dt);

  /// @brief 更新姿态（9轴，标量接口）
  void Update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz,
              double dt);

  /// @brief 更新姿态（6轴，Eigen 向量接口）

  void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt);

  /// @brief 更新姿态（9轴，Eigen 向量接口）

  void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag, double dt);

  /// @brief 获取当前姿态四元数
  const Eigen::Quaterniond& Quaternion() const;

  void EulerAngle(double& roll, double& pitch, double& yaw) const;

  void Reset();

  void SetAlgorithm(AlgorithmType algo);

  void SetAxisMode(AxisMode mode);

  AxisMode GetAxisMode() const {
    return axis_mode_;
  }

  void SetAlphaAcc(double alpha);

  void SetAlphaMag(double alpha);

  double GetAlphaAcc() const;

  double GetAlphaMag() const;

  void SetAccelRejectionThreshold(double threshold);

  const char* GetAlgorithmName() const;

  const char* GetAxisModeName() const;

  static AlgorithmType AlgorithmFromString(const std::string& name);

  static AxisMode AxisModeFromString(const std::string& name);

private:
  static ComplementaryFilter::AxisMode toCompAxisMode(AxisMode mode);

  static RK4Integration::AxisMode toRK4AxisMode(AxisMode mode);

private:
  AlgorithmType algo_;
  AxisMode      axis_mode_;
  double        alpha_acc_;
  double        alpha_mag_;

  std::unique_ptr<ComplementaryFilter> comp_filter_ptr_;
  std::unique_ptr<RK4Integration>      rk4_filter_ptr_;
};

} // namespace imu_algorithm
