#pragma once

#include "algorithm/attitude_estimator.h"
#include "algorithm/quaternion.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace imu_algorithm {

/// @brief Madgwick 梯度下降姿态解算算法（基于 Eigen 实现）
class MadgwickFilter : public AttitudeEstimator {
public:
  /// @brief 构造函数
  /// @param mode 轴数模式
  /// @param beta 梯度下降步长/融合系数 (rad/s)
  /// @param zeta 陀螺仪零偏补偿系数
  MadgwickFilter(AttitudeEstimator::AxisMode mode = AxisMode::NINE_AXIS, double beta = 0.5, double zeta = 0.0)
      : AttitudeEstimator(mode), q_(Eigen::Quaterniond::Identity()), mode_(mode), beta_(beta), zeta_(zeta),
        accel_reject_threshold_(0.3), gyro_bias_(Eigen::Vector3d::Zero()), mag_ref_(0.0, 0.0, 1.0),
        mag_ref_initialized_(false) {
  }

  // 接口实现
  void Update(double gx, double gy, double gz, double ax, double ay, double az, double dt) override;
  void Update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz,
              double dt) override;
  void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt) override;
  void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag,
              double dt) override;

  const Eigen::Quaterniond& Quaternion() const override {
    return q_;
  }
  void EulerAngle(double& roll, double& pitch, double& yaw) const override;
  void Reset() override;

  void SetBeta(double beta) {
    beta_ = beta;
  }
  double GetBeta() const {
    return beta_;
  }
  void SetZeta(double zeta) {
    zeta_ = zeta;
  }
  double GetZeta() const {
    return zeta_;
  }

  void SetAxisMode(AttitudeEstimator::AxisMode mode) override {
    mode_ = mode;
  }
  AttitudeEstimator::AxisMode GetAxisMode() const {
    return mode_;
  }

  void SetAccelRejectionThreshold(double threshold) {
    accel_reject_threshold_ = threshold;
  }

  const Eigen::Vector3d& GetGyroBias() const {
    return gyro_bias_;
  }

  const char* GetAlgorithmName() const {
    return "madgwick";
  }

private:
  Eigen::Quaterniond accelGradientDescent(const Eigen::Vector3d& accel) const;
  Eigen::Quaterniond accelMagGradientDescent(const Eigen::Vector3d& accel, const Eigen::Vector3d& mag) const;
  void               updateGyroBias(const Eigen::Quaterniond& q_gradient, double dt);
  bool               isAccelValid(const Eigen::Vector3d& accel) const;

private:
  Eigen::Quaterniond          q_;
  AttitudeEstimator::AxisMode mode_;
  double                      beta_;
  double                      zeta_;
  double                      accel_reject_threshold_;

  Eigen::Vector3d gyro_bias_;
  Eigen::Vector3d mag_ref_;
  bool            mag_ref_initialized_;

  static constexpr double GRAVITY = 9.80665;
};

} // namespace imu_algorithm
