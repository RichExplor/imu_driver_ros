#pragma once

#include "algorithm/attitude_estimator.h"
#include "algorithm/quaternion.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace imu_algorithm {

/// @brief Mahony 互补式姿态估计算法（基于 Eigen 实现）
class MahonyFilter : public AttitudeEstimator {
public:
  MahonyFilter(AttitudeEstimator::AxisMode mode = AxisMode::NINE_AXIS, double kp = 5.0, double ki = 0.05)
      : AttitudeEstimator(mode), q_(Eigen::Quaterniond::Identity()), mode_(mode), kp_(kp), ki_(ki),
        accel_reject_threshold_(0.2), integral_error_(Eigen::Vector3d::Zero()), mag_ref_(0.0, 0.0, 1.0),
        mag_ref_initialized_(false) {
  }

  void Update(double gx, double gy, double gz, double ax, double ay, double az, double dt) override;
  void Update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz,
              double dt) override;

  void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt) override;
  void Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag,
              double dt) override;

  const Eigen::Quaterniond& Quaternion() const override {
    return q_;
  }

  void SetQuaternion(const Eigen::Quaterniond& q) {
    q_ = q.normalized();
  }

  void EulerAngle(double& roll, double& pitch, double& yaw) const override;
  void Reset() override;

  void SetKp(double kp) {
    kp_ = kp;
  }
  double GetKp() const {
    return kp_;
  }
  void SetKi(double ki) {
    ki_ = ki;
  }
  double GetKi() const {
    return ki_;
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

  const Eigen::Vector3d& GetIntegralError() const {
    return integral_error_;
  }

  const char* GetAlgorithmName() const {
    return "mahony";
  }

private:
  bool isAccelValid(const Eigen::Vector3d& accel) const;
  bool isMagValid(const Eigen::Vector3d& mag) const;

private:
  Eigen::Quaterniond          q_;
  AttitudeEstimator::AxisMode mode_;
  double                      kp_;
  double                      ki_;
  double                      accel_reject_threshold_;

  Eigen::Vector3d integral_error_; ///< 积分误差累积项 (Mahony 论文中的 e_int)
  Eigen::Vector3d mag_ref_;
  bool            mag_ref_initialized_;

  static constexpr double GRAVITY              = 9.80665;
  static constexpr double INTEGRAL_ERROR_LIMIT = 1.0; ///< 积分误差限幅 (rad/s)
};

} // namespace imu_algorithm
