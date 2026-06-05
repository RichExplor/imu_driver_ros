#include "algorithm/rk4_integration.h"
#include <cmath>

namespace imu_algorithm {

RK4Integration::RK4Integration(AxisMode mode, double alpha_acc, double alpha_mag)
    : q_(Eigen::Quaterniond::Identity()), mode_(mode), alpha_acc_(alpha_acc), alpha_mag_(alpha_mag),
      accel_reject_threshold_(0.3) {
}

void RK4Integration::Update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
  Update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), dt);
}

void RK4Integration::Update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my,
                            double mz, double dt) {
  Update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(mx, my, mz), dt);
}

void RK4Integration::Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt) {
  rk4Step(gyro, dt);

  correctWithAccel(accel);
}

void RK4Integration::Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag,
                            double dt) {
  rk4Step(gyro, dt);

  correctWithAccel(accel);

  if (mode_ == AxisMode::NINE_AXIS) {
    correctWithMag(mag);
  }
}

void RK4Integration::rk4Step(const Eigen::Vector3d& gyro, double dt) {
  // 四元数微分方程: dq/dt = 0.5 * q ⊗ ω
  // RK4 积分

  // k1 = f(t, q)
  Eigen::Quaterniond k1 = QuaternionUtils::Derivative(q_, gyro);

  // k2 = f(t + dt/2, q + k1*dt/2)
  Eigen::Quaterniond q2;
  q2.coeffs() = q_.coeffs() + k1.coeffs() * (dt * 0.5);
  q2.normalize();
  Eigen::Quaterniond k2 = QuaternionUtils::Derivative(q2, gyro);

  // k3 = f(t + dt/2, q + k2*dt/2)
  Eigen::Quaterniond q3;
  q3.coeffs() = q_.coeffs() + k2.coeffs() * (dt * 0.5);
  q3.normalize();
  Eigen::Quaterniond k3 = QuaternionUtils::Derivative(q3, gyro);

  // k4 = f(t + dt, q + k3*dt)
  Eigen::Quaterniond q4;
  q4.coeffs() = q_.coeffs() + k3.coeffs() * dt;
  q4.normalize();
  Eigen::Quaterniond k4 = QuaternionUtils::Derivative(q4, gyro);

  // q(t+dt) = q(t) + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
  q_.coeffs() += (dt / 6.0) * (k1.coeffs() + 2.0 * k2.coeffs() + 2.0 * k3.coeffs() + k4.coeffs());

  q_.normalize();
}

void RK4Integration::correctWithAccel(const Eigen::Vector3d& accel) {
  if (!isAccelValid(accel)) {
    return;
  }

  // 归一化加速度计测量值
  double norm = accel.norm();
  if (norm < 1e-10)
    return;
  Eigen::Vector3d a_norm = accel / norm;

  // 参考重力方向在机体系的表示
  Eigen::Vector3d gravity_ref = q_ * Eigen::Vector3d::UnitZ();

  // 计算误差：叉积
  Eigen::Vector3d error = a_norm.cross(gravity_ref);

  // 互补修正
  double factor = alpha_acc_ * 0.5;
  q_.x() += factor * error.x();
  q_.y() += factor * error.y();
  q_.z() += factor * error.z();

  q_.normalize();
}

void RK4Integration::correctWithMag(const Eigen::Vector3d& mag) {
  double norm = mag.norm();
  if (norm < 1e-10)
    return;
  Eigen::Vector3d m_norm = mag / norm;

  Eigen::Vector3d mag_ref = QuaternionUtils::InverseRotateVector(q_, m_norm);

  double bx = std::sqrt(mag_ref.x() * mag_ref.x() + mag_ref.y() * mag_ref.y());
  double bz = mag_ref.z();

  Eigen::Vector3d mag_body = QuaternionUtils::RotateVector(q_, Eigen::Vector3d(bx, 0.0, bz));

  Eigen::Vector3d error = m_norm.cross(mag_body);

  double factor = alpha_mag_ * 0.5;
  q_.x() += factor * error.x();
  q_.y() += factor * error.y();
  q_.z() += factor * error.z();

  q_.normalize();
}

bool RK4Integration::isAccelValid(const Eigen::Vector3d& accel) const {
  double norm  = accel.norm();
  double ratio = std::abs(norm - GRAVITY) / GRAVITY;
  return ratio < accel_reject_threshold_;
}

void RK4Integration::EulerAngle(double& roll, double& pitch, double& yaw) const {
  QuaternionUtils::ToEuler(q_, roll, pitch, yaw);
}

void RK4Integration::Reset() {
  q_ = Eigen::Quaterniond::Identity();
}

} // namespace imu_algorithm
