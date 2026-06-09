#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

namespace imu_algorithm {

/// @brief 抽象基类：姿态解算器接口（Mahony / Madgwick 的基类）
class AttitudeEstimator {
public:
  enum class AxisMode { SIX_AXIS = 6, NINE_AXIS = 9 };

  explicit AttitudeEstimator(AxisMode mode = AxisMode::NINE_AXIS) : axis_mode_(mode) {
  }
  virtual ~AttitudeEstimator() = default;

  // 更新接口（6/9 轴重载）
  virtual void Update(double gx, double gy, double gz, double ax, double ay, double az, double dt) = 0;
  virtual void Update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz,
                      double dt)                                                                   = 0;

  virtual void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt) = 0;
  virtual void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag,
                      double dt)                                                            = 0;

  // 状态查询
  virtual const Eigen::Quaterniond& Quaternion() const                                         = 0;
  virtual void                      EulerAngle(double& roll, double& pitch, double& yaw) const = 0;
  virtual void                      Reset()                                                    = 0;

  // 可选：切换轴模式
  virtual void SetAxisMode(AxisMode mode) {
    axis_mode_ = mode;
  }
  AxisMode GetAxisMode() const {
    return axis_mode_;
  }

  const char* GetAxisModeName() const {
    return (axis_mode_ == AxisMode::SIX_AXIS) ? "6-axis" : "9-axis";
  }

  // 算法名称（由子类返回）
  virtual const char* GetAlgorithmName() const = 0;

  static AxisMode AxisModeFromString(const std::string& name);

private:
  AxisMode axis_mode_;
};

} // namespace imu_algorithm
