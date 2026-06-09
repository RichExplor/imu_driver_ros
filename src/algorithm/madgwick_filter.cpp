#include "algorithm/madgwick_filter.h"
#include <cmath>

namespace imu_algorithm {

void MadgwickFilter::Update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
  Update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), dt);
}

void MadgwickFilter::Update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my,
                            double mz, double dt) {
  Update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(mx, my, mz), dt);
}

void MadgwickFilter::Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt) {
  // 1. 补偿陀螺仪零偏
  Eigen::Vector3d gyro_corrected = gyro - gyro_bias_;

  // 2. 陀螺仪积分：一阶近似 q_new = q + dq/dt * dt
  Eigen::Quaterniond dq = QuaternionUtils::Derivative(q_, gyro_corrected);
  q_.coeffs() += dq.coeffs() * dt;

  // 3. 加速度计梯度下降修正
  if (isAccelValid(accel)) {
    Eigen::Quaterniond q_grad = accelGradientDescent(accel);
    // 加权融合：q = q + (1 - γ) * dq_gyro * dt + γ * (-beta * ∇f)
    // 等价于：q = q + dq_gyro*dt - beta * ∇f * dt
    // ∇f 即 q_grad（已归一化的梯度下降方向）
    q_.coeffs() -= beta_ * dt * q_grad.coeffs();

    // 4. 更新陀螺仪零偏（当有加速度计修正时）
    if (zeta_ > 0.0) {
      updateGyroBias(q_grad, dt);
    }
  }

  q_.normalize();
}

void MadgwickFilter::Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag,
                            double dt) {
  // 1. 补偿陀螺仪零偏
  Eigen::Vector3d gyro_corrected = gyro - gyro_bias_;

  // 2. 陀螺仪积分
  Eigen::Quaterniond dq = QuaternionUtils::Derivative(q_, gyro_corrected);
  q_.coeffs() += dq.coeffs() * dt;

  // 3. 加速度计 + 磁力计梯度下降修正
  if (isAccelValid(accel)) {
    if (mode_ == AttitudeEstimator::AxisMode::NINE_AXIS) {
      // 初始化磁场参考方向（第一次收到有效磁力计数据时）
      if (!mag_ref_initialized_) {
        // 将磁力计测量转到参考系水平面
        Eigen::Vector3d mag_inertial = QuaternionUtils::InverseRotateVector(q_, mag);
        double          mag_norm = std::sqrt(mag_inertial.x() * mag_inertial.x() + mag_inertial.y() * mag_inertial.y());
        if (mag_norm > 1e-10) {
          // 参考方向：水平面磁场方向 + 垂直分量
          mag_ref_ = Eigen::Vector3d(mag_norm, 0.0, mag_inertial.z());
          mag_ref_.normalize();
          mag_ref_initialized_ = true;
        }
      }

      if (mag_ref_initialized_) {
        Eigen::Quaterniond q_grad = accelMagGradientDescent(accel, mag);
        q_.coeffs() -= beta_ * dt * q_grad.coeffs();

        if (zeta_ > 0.0) {
          updateGyroBias(q_grad, dt);
        }
      } else {
        // 磁场参考未初始化，仅使用加速度计
        Eigen::Quaterniond q_grad = accelGradientDescent(accel);
        q_.coeffs() -= beta_ * dt * q_grad.coeffs();

        if (zeta_ > 0.0) {
          updateGyroBias(q_grad, dt);
        }
      }
    } else {
      // 6轴模式：仅加速度计
      Eigen::Quaterniond q_grad = accelGradientDescent(accel);
      q_.coeffs() -= beta_ * dt * q_grad.coeffs();

      if (zeta_ > 0.0) {
        updateGyroBias(q_grad, dt);
      }
    }
  }

  q_.normalize();
}

Eigen::Quaterniond MadgwickFilter::accelGradientDescent(const Eigen::Vector3d& accel) const {
  // 归一化加速度计
  double norm = accel.norm();
  if (norm < 1e-10) {
    return Eigen::Quaterniond::Identity();
  }
  Eigen::Vector3d a = accel / norm;

  // 目标函数：重力在机体系的方向
  // 参考重力在机体系的投影 = q * [0, 0, 1] * q^-1
  // 用四元数直接计算：
  //   g_body = [2*(q1*q3 - q0*q2),
  //             2*(q0*q1 + q2*q3),
  //             q0^2 - q1^2 - q2^2 + q3^2]
  double q0 = q_.w(), q1 = q_.x(), q2 = q_.y(), q3 = q_.z();

  // 目标函数 f(q, a) = g_body - a
  // f = [2*(q1*q3 - q0*q2) - ax,
  //      2*(q0*q1 + q2*q3) - ay,
  //      q0^2 - q1^2 - q2^2 + q3^2 - az]

  // 雅可比矩阵 J = df/dq (3x4)
  // 对 q0: [-2*q2,     2*q1,    2*q0   ]
  // 对 q1: [ 2*q3,     2*q0,   -2*q1   ]
  // 对 q2: [-2*q0,     2*q3,   -2*q2   ]
  // 对 q3: [ 2*q1,     2*q2,    2*q3   ]

  // 计算目标函数值
  double f1 = 2.0 * (q1 * q3 - q0 * q2) - a.x();
  double f2 = 2.0 * (q0 * q1 + q2 * q3) - a.y();
  double f3 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3 - a.z();

  // 计算梯度 = J^T * f
  double grad_w = -2.0 * q2 * f1 + 2.0 * q1 * f2 + 2.0 * q0 * f3;
  double grad_x = 2.0 * q3 * f1 + 2.0 * q0 * f2 - 2.0 * q1 * f3;
  double grad_y = -2.0 * q0 * f1 + 2.0 * q3 * f2 - 2.0 * q2 * f3;
  double grad_z = 2.0 * q1 * f1 + 2.0 * q2 * f2 + 2.0 * q3 * f3;

  // 归一化梯度（梯度下降步长由 beta 控制）
  double grad_norm = std::sqrt(grad_w * grad_w + grad_x * grad_x + grad_y * grad_y + grad_z * grad_z);
  if (grad_norm < 1e-10) {
    return Eigen::Quaterniond::Identity();
  }

  return Eigen::Quaterniond(grad_w / grad_norm, grad_x / grad_norm, grad_y / grad_norm, grad_z / grad_norm);
}

Eigen::Quaterniond MadgwickFilter::accelMagGradientDescent(const Eigen::Vector3d& accel,
                                                           const Eigen::Vector3d& mag) const {
  // 归一化加速度计
  double a_norm = accel.norm();
  if (a_norm < 1e-10) {
    return Eigen::Quaterniond::Identity();
  }
  Eigen::Vector3d a = accel / a_norm;

  // 归一化磁力计
  double m_norm = mag.norm();
  if (m_norm < 1e-10) {
    // 磁力计无效，退化为仅加速度计
    return accelGradientDescent(accel);
  }
  Eigen::Vector3d m = mag / m_norm;

  double q0 = q_.w(), q1 = q_.x(), q2 = q_.y(), q3 = q_.z();

  // 参考磁场方向（在参考系水平面）
  double bx = mag_ref_.x();
  double bz = mag_ref_.z();

  // 目标函数（6个方程）：
  // 加速度计部分（3个）：
  //   f1 = 2*(q1*q3 - q0*q2) - ax
  //   f2 = 2*(q0*q1 + q2*q3) - ay
  //   f3 = q0^2 - q1^2 - q2^2 + q3^2 - az
  //
  // 磁力计部分（3个）：
  //   f4 = 2*bx*(0.5 - q2^2 - q3^2) + 2*bz*(q1*q3 - q0*q2) - mx
  //   f5 = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3) - my
  //   f6 = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5 - q1^2 - q2^2) - mz

  double f1 = 2.0 * (q1 * q3 - q0 * q2) - a.x();
  double f2 = 2.0 * (q0 * q1 + q2 * q3) - a.y();
  double f3 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3 - a.z();

  double f4 = 2.0 * bx * (0.5 - q2 * q2 - q3 * q3) + 2.0 * bz * (q1 * q3 - q0 * q2) - m.x();
  double f5 = 2.0 * bx * (q1 * q2 - q0 * q3) + 2.0 * bz * (q0 * q1 + q2 * q3) - m.y();
  double f6 = 2.0 * bx * (q0 * q2 + q1 * q3) + 2.0 * bz * (0.5 - q1 * q1 - q2 * q2) - m.z();

  // 雅可比矩阵 J^T (6x4) 的转置 * f
  // 对 q0:
  double grad_w = -2.0 * q2 * f1 + 2.0 * q1 * f2 + 2.0 * q0 * f3 + (-2.0 * bz * q2) * f4 +
                  (-2.0 * bx * q3 + 2.0 * bz * q1) * f5 + (2.0 * bx * q2) * f6;

  // 对 q1:
  double grad_x = 2.0 * q3 * f1 + 2.0 * q0 * f2 - 2.0 * q1 * f3 + (2.0 * bz * q3) * f4 +
                  (2.0 * bx * q2 + 2.0 * bz * q0) * f5 + (2.0 * bx * q3 - 4.0 * bz * q1) * f6;

  // 对 q2:
  double grad_y = -2.0 * q0 * f1 + 2.0 * q3 * f2 - 2.0 * q2 * f3 + (-4.0 * bx * q2 - 2.0 * bz * q0) * f4 +
                  (2.0 * bx * q1 - 4.0 * bz * q2) * f5 + (2.0 * bx * q0 - 4.0 * bz * q2) * f6;

  // 对 q3:
  double grad_z = 2.0 * q1 * f1 + 2.0 * q2 * f2 + 2.0 * q3 * f3 + (-4.0 * bx * q3) * f4 +
                  (-2.0 * bx * q0 + 2.0 * bz * q2) * f5 + (2.0 * bx * q1) * f6;

  // 归一化梯度
  double grad_norm = std::sqrt(grad_w * grad_w + grad_x * grad_x + grad_y * grad_y + grad_z * grad_z);
  if (grad_norm < 1e-10) {
    return Eigen::Quaterniond::Identity();
  }

  return Eigen::Quaterniond(grad_w / grad_norm, grad_x / grad_norm, grad_y / grad_norm, grad_z / grad_norm);
}

void MadgwickFilter::updateGyroBias(const Eigen::Quaterniond& q_gradient, double dt) {
  // 梯度下降方向对应角速率误差
  // 误差角速率 ω_error = 2 * q_gradient 的虚部（近似）
  // 更新零偏：bias += zeta * ω_error * dt
  // 注意：q_gradient 是归一化的梯度下降方向四元数
  // 修正量 = 2 * q_gradient.xyz 表示需要修正的角速率方向
  Eigen::Vector3d omega_error;
  omega_error.x() = 2.0 * q_gradient.x();
  omega_error.y() = 2.0 * q_gradient.y();
  omega_error.z() = 2.0 * q_gradient.z();

  gyro_bias_ += zeta_ * omega_error * dt;

  // 限制零偏范围，防止漂移过大
  const double max_bias = 0.05; // 最大零偏 0.05 rad/s ≈ 2.86 deg/s
  for (int i = 0; i < 3; ++i) {
    if (gyro_bias_(i) > max_bias)
      gyro_bias_(i) = max_bias;
    if (gyro_bias_(i) < -max_bias)
      gyro_bias_(i) = -max_bias;
  }
}

bool MadgwickFilter::isAccelValid(const Eigen::Vector3d& accel) const {
  double norm  = accel.norm();
  double ratio = std::abs(norm - GRAVITY) / GRAVITY;
  return ratio < accel_reject_threshold_;
}

void MadgwickFilter::EulerAngle(double& roll, double& pitch, double& yaw) const {
  QuaternionUtils::ToEuler(q_, roll, pitch, yaw);
}

void MadgwickFilter::Reset() {
  q_                   = Eigen::Quaterniond::Identity();
  gyro_bias_           = Eigen::Vector3d::Zero();
  mag_ref_initialized_ = false;
}

} // namespace imu_algorithm
