#include "algorithm/mahony_filter.h"
#include <cmath>

namespace imu_algorithm {

void MahonyFilter::Update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
  Update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), dt);
}

void MahonyFilter::Update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my,
                          double mz, double dt) {
  Update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(mx, my, mz), dt);
}

/// @brief 6轴 Mahony 更新（仅加速度计辅助）
///
/// 标准 Mahony 互补滤波算法（NED/ENU 均适用）：
///   1. 归一化加速度计测量
///   2. 将惯性系重力方向 [0,0,1] 旋转到机体系，得到估计重力方向 g_est
///      注意：q * v 表示将惯性系向量 v 旋转到机体系
///   3. 计算误差 e = g_est × a（估计值叉乘测量值）
///   4. 积分误差 e_int += ki * e * dt
///   5. 修正角速度 omega = gyro + kp * e + ki * e_int
///   6. 四元数一阶积分 q = q + 0.5 * q ⊗ [0, omega] * dt
void MahonyFilter::Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt) {
  Eigen::Vector3d error = Eigen::Vector3d::Zero();

  if (isAccelValid(accel)) {
    Eigen::Vector3d a = accel.normalized();

    // 将惯性系重力方向 [0,0,1] 旋转到机体系，得到估计的重力方向
    Eigen::Vector3d g_est = q_ * Eigen::Vector3d::UnitZ();
    error = g_est.cross(a);
  }

  if (ki_ > 0.0 && isAccelValid(accel)) {
    integral_error_ += ki_ * error * dt;
    integral_error_ = integral_error_.cwiseMax(-INTEGRAL_ERROR_LIMIT).cwiseMin(INTEGRAL_ERROR_LIMIT);
  }

  Eigen::Vector3d omega = gyro + kp_ * error + integral_error_;
  Eigen::Quaterniond dq(1.0, 0.5 * omega.x() * dt, 0.5 * omega.y() * dt, 0.5 * omega.z() * dt);
  q_ = (q_ * dq).normalized();
}

/// @brief 9轴 Mahony 更新（加速度计 + 磁力计辅助）
///
/// 在6轴基础上增加磁力计航向修正：
///   1. 计算加速度计误差（与6轴相同）
///   2. 将磁力计测量转到惯性系，建立水平面磁场参考方向 mag_ref_
///   3. 将 mag_ref_ 转回机体系，与实际磁力计测量计算叉乘误差
///   4. 磁力计误差贡献在水平面(XY)起作用，修正航向
///   5. 融合加速度计和磁力计误差，统一更新
void MahonyFilter::Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag,
                          double dt) {
  if (mode_ != AxisMode::NINE_AXIS || !isMagValid(mag)) {
    Update(gyro, accel, dt);
    return;
  }

  Eigen::Vector3d error               = Eigen::Vector3d::Zero();
  bool            has_valid_reference = false;

  if (isAccelValid(accel)) {
    Eigen::Vector3d a = accel.normalized();
    Eigen::Vector3d g_est = q_ * Eigen::Vector3d::UnitZ();
    error += g_est.cross(a);
    has_valid_reference = true;
  }

  Eigen::Vector3d m = mag.normalized();

  // 初始化磁场参考方向（仅在首次有效磁力计数据时）
  if (!mag_ref_initialized_) {
    Eigen::Vector3d m_inertial = q_.conjugate() * m;
    double bx = std::sqrt(m_inertial.x() * m_inertial.x() + m_inertial.y() * m_inertial.y());
    if (bx > 1e-6) {
      mag_ref_             = Eigen::Vector3d(bx, 0.0, m_inertial.z()).normalized();
      mag_ref_initialized_ = true;
    }
  }

  if (mag_ref_initialized_) {
    Eigen::Vector3d m_ref_body = q_ * mag_ref_;
    Eigen::Vector3d error_mag = m_ref_body.cross(m);
    error.x() += error_mag.x();
    error.y() += error_mag.y();
    has_valid_reference = true;
  }

  if (ki_ > 0.0 && has_valid_reference) {
    integral_error_ += ki_ * error * dt;
    integral_error_ = integral_error_.cwiseMax(-INTEGRAL_ERROR_LIMIT).cwiseMin(INTEGRAL_ERROR_LIMIT);
  }

  Eigen::Vector3d omega = gyro + kp_ * error + integral_error_;

  Eigen::Quaterniond dq(1.0, 0.5 * omega.x() * dt, 0.5 * omega.y() * dt, 0.5 * omega.z() * dt);
  q_ = (q_ * dq).normalized();
}

bool MahonyFilter::isAccelValid(const Eigen::Vector3d& accel) const {
  double norm = accel.norm();
  if (norm < 1e-10) {
    return false;
  }

  double ratio = std::abs(norm - GRAVITY) / GRAVITY;
  return ratio < accel_reject_threshold_;
}

bool MahonyFilter::isMagValid(const Eigen::Vector3d& mag) const {
  double norm = mag.norm();
  return norm > 1e-10;
}

void MahonyFilter::EulerAngle(double& roll, double& pitch, double& yaw) const {
  QuaternionUtils::ToEuler(q_, roll, pitch, yaw);
}

void MahonyFilter::Reset() {
  q_                   = Eigen::Quaterniond::Identity();
  integral_error_      = Eigen::Vector3d::Zero();
  mag_ref_initialized_ = false;
}

} // namespace imu_algorithm
