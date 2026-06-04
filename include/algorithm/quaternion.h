#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace imu_algorithm {

/// @brief 基于 Eigen 的四元数工具类
///
/// 封装 Eigen::Quaterniond，提供姿态解算所需的四元数运算工具函数。
/// Eigen::Quaterniond 内部存储顺序为 (w, x, y, z)，构造函数参数也是 (w, x, y, z)。
/// 注意：Eigen::Quaterniond::coeffs() 返回顺序为 (x, y, z, w)，与构造函数不同。
///
/// 旋转关系：q 表示从参考系（NED/ENU）到机体系的旋转
class QuaternionUtils {
public:
  /// @brief 从四元数提取欧拉角（ZYX 旋转顺序）
  /// @param q 四元数
  /// @param roll  横滚角（弧度）
  /// @param pitch 俯仰角（弧度）
  /// @param yaw   偏航角（弧度）
  static void ToEuler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
    Eigen::Matrix3d R = q.toRotationMatrix();

    double sinp = -R(2, 0);
    if (std::abs(sinp) >= 1.0) {
      pitch = std::copysign(M_PI / 2.0, sinp);
      roll  = std::atan2(-R(0, 1), R(1, 1));
      yaw   = 0.0;
    } else {
      pitch = std::asin(sinp);
      roll  = std::atan2(R(2, 1), R(2, 2));
      yaw   = std::atan2(R(1, 0), R(0, 0));
    }
  }

  /// @brief 计算四元数导数 dq/dt = 0.5 * q ⊗ ω
  /// @param q 当前四元数
  /// @param gyro 机体系角速度（rad/s）
  /// @return 四元数导数（非单位四元数，仅用于积分）
  static Eigen::Quaterniond Derivative(const Eigen::Quaterniond& q, const Eigen::Vector3d& gyro) {
    // dq/dt = 0.5 * q * (0, ω)
    // 手动展开 Hamilton 乘积：
    // q = (q0, q1, q2, q3), ω_quat = (0, ωx, ωy, ωz)
    double q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
    double wx = gyro.x(), wy = gyro.y(), wz = gyro.z();

    double dw = 0.5 * (-q1 * wx - q2 * wy - q3 * wz);
    double dx = 0.5 * (q0 * wx + q2 * wz - q3 * wy);
    double dy = 0.5 * (q0 * wy - q1 * wz + q3 * wx);
    double dz = 0.5 * (q0 * wz + q1 * wy - q2 * wx);

    return Eigen::Quaterniond(dw, dx, dy, dz);
  }

  /// @brief 计算四元数导数 dq/dt = 0.5 * q ⊗ ω（标量参数版本）
  static Eigen::Quaterniond Derivative(const Eigen::Quaterniond& q, double gx, double gy, double gz) {
    return Derivative(q, Eigen::Vector3d(gx, gy, gz));
  }

  /// @brief 使用四元数旋转向量（q * v * q^-1）
  static Eigen::Vector3d RotateVector(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) {
    return q * v;
  }

  /// @brief 使用四元数的逆旋转向量（q^-1 * v * q）
  static Eigen::Vector3d InverseRotateVector(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) {
    return q.conjugate() * v;
  }

public:
  static constexpr double PI = 3.14159265358979323846;
};

} // namespace imu_algorithm
