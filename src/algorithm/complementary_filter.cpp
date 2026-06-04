#include "algorithm/complementary_filter.h"
#include <cmath>

namespace imu_algorithm {

ComplementaryFilter::ComplementaryFilter(AxisMode mode, double alpha_acc, double alpha_mag)
    : q_(Eigen::Quaterniond::Identity()), mode_(mode), alpha_acc_(alpha_acc), alpha_mag_(alpha_mag),
      accel_reject_threshold_(0.3) {
}

void ComplementaryFilter::Update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
  Update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), dt);
}

void ComplementaryFilter::Update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my,
                                 double mz, double dt) {
  Update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(mx, my, mz), dt);
}

void ComplementaryFilter::Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt) {
  // 1. 陀螺仪积分：一阶近似 q_new = q + dq/dt * dt
  Eigen::Quaterniond dq = QuaternionUtils::Derivative(q_, gyro);
  // 四元数加法 + 缩放：q_ += dq * dt
  q_.coeffs() += dq.coeffs() * dt;
  q_.normalize();

  // 2. 加速度计修正 roll/pitch
  correctWithAccel(accel);
}

void ComplementaryFilter::Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag,
                                 double dt) {
  // 1. 陀螺仪积分
  Eigen::Quaterniond dq = QuaternionUtils::Derivative(q_, gyro);
  q_.coeffs() += dq.coeffs() * dt;
  q_.normalize();

  // 2. 加速度计修正 roll/pitch
  correctWithAccel(accel);

  // 3. 磁力计修正 yaw（仅 9 轴模式）
  if (mode_ == AxisMode::NINE_AXIS) {
    correctWithMag(mag);
  }
}

void ComplementaryFilter::correctWithAccel(const Eigen::Vector3d& accel) {
  // 检查加速度计数据有效性
  if (!isAccelValid(accel)) {
    return;
  }

  // 归一化加速度计测量值（重力方向在机体系的投影）
  double norm = accel.norm();
  if (norm < 1e-10)
    return;
  Eigen::Vector3d a_norm = accel / norm;

  // 参考重力方向在机体系的表示：q * [0,0,1] * q^-1
  // 即将参考系 z 轴旋转到机体系
  Eigen::Vector3d gravity_ref = q_ * Eigen::Vector3d::UnitZ();

  // 计算误差：测量方向 × 参考方向（叉积）
  Eigen::Vector3d error = a_norm.cross(gravity_ref);

  // 互补滤波修正：将误差以比例系数加到四元数上
  // δq ≈ (1, 0.5*ex*k, 0.5*ey*k, 0.5*ez*k)
  double factor = alpha_acc_ * 0.5;
  q_.x() += factor * error.x();
  q_.y() += factor * error.y();
  q_.z() += factor * error.z();

  q_.normalize();
}

void ComplementaryFilter::correctWithMag(const Eigen::Vector3d& mag) {
  // 归一化磁力计测量值
  double norm = mag.norm();
  if (norm < 1e-10)
    return;
  Eigen::Vector3d m_norm = mag / norm;

  // 将磁力计测量值从机体系转到参考系（水平面）
  Eigen::Vector3d mag_ref = QuaternionUtils::InverseRotateVector(q_, m_norm);

  // 在水平面上投影，忽略 z 分量，得到磁场水平方向
  double bx = std::sqrt(mag_ref.x() * mag_ref.x() + mag_ref.y() * mag_ref.y());
  double bz = mag_ref.z();

  // 参考磁场方向在机体系的表示：将 (bx, 0, bz) 从参考系旋转回机体系
  Eigen::Vector3d mag_body = QuaternionUtils::RotateVector(q_, Eigen::Vector3d(bx, 0.0, bz));

  // 计算误差：测量方向 × 参考方向
  Eigen::Vector3d error = m_norm.cross(mag_body);

  // 互补滤波修正
  double factor = alpha_mag_ * 0.5;
  q_.x() += factor * error.x();
  q_.y() += factor * error.y();
  q_.z() += factor * error.z();

  q_.normalize();
}

bool ComplementaryFilter::isAccelValid(const Eigen::Vector3d& accel) const {
  double norm  = accel.norm();
  double ratio = std::abs(norm - GRAVITY) / GRAVITY;
  return ratio < accel_reject_threshold_;
}

void ComplementaryFilter::EulerAngle(double& roll, double& pitch, double& yaw) const {
  QuaternionUtils::ToEuler(q_, roll, pitch, yaw);
}

void ComplementaryFilter::Reset() {
  q_ = Eigen::Quaterniond::Identity();
}

} // namespace imu_algorithm
