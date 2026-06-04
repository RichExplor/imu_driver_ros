#include "algorithm/attitude_estimator.h"
#include <algorithm>
#include <cstring>

namespace imu_algorithm {

AttitudeEstimator::AttitudeEstimator(AlgorithmType algo, AxisMode axis_mode, double alpha_acc, double alpha_mag)
    : algo_(algo), axis_mode_(axis_mode), alpha_acc_(alpha_acc), alpha_mag_(alpha_mag) {
  // 根据算法类型创建对应的滤波器实例
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ptr_ = std::make_unique<ComplementaryFilter>(toCompAxisMode(axis_mode_), alpha_acc_, alpha_mag_);
  } else {
    rk4_filter_ptr_ = std::make_unique<RK4Integration>(toRK4AxisMode(axis_mode_), alpha_acc_, alpha_mag_);
  }
}

void AttitudeEstimator::Update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
  Update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), dt);
}

void AttitudeEstimator::Update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my,
                               double mz, double dt) {
  Update(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(ax, ay, az), Eigen::Vector3d(mx, my, mz), dt);
}

void AttitudeEstimator::Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, double dt) {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ptr_->Update(gyro, accel, dt);
  } else {
    rk4_filter_ptr_->Update(gyro, accel, dt);
  }
}

void AttitudeEstimator::Update(const Eigen::Vector3d& gyro, const Eigen::Vector3d& accel, const Eigen::Vector3d& mag,
                               double dt) {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ptr_->Update(gyro, accel, mag, dt);
  } else {
    rk4_filter_ptr_->Update(gyro, accel, mag, dt);
  }
}

const Eigen::Quaterniond& AttitudeEstimator::Quaternion() const {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    return comp_filter_ptr_->Quaternion();
  } else {
    return rk4_filter_ptr_->Quaternion();
  }
}

void AttitudeEstimator::EulerAngle(double& roll, double& pitch, double& yaw) const {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ptr_->EulerAngle(roll, pitch, yaw);
  } else {
    rk4_filter_ptr_->EulerAngle(roll, pitch, yaw);
  }
}

void AttitudeEstimator::Reset() {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ptr_->Reset();
  } else {
    rk4_filter_ptr_->Reset();
  }
}

void AttitudeEstimator::SetAlgorithm(AlgorithmType algo) {
  if (algo == algo_)
    return;

  algo_ = algo;

  // 释放旧算法，创建新算法
  comp_filter_ptr_.reset();
  rk4_filter_ptr_.reset();

  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ptr_ = std::make_unique<ComplementaryFilter>(toCompAxisMode(axis_mode_), alpha_acc_, alpha_mag_);
  } else {
    rk4_filter_ptr_ = std::make_unique<RK4Integration>(toRK4AxisMode(axis_mode_), alpha_acc_, alpha_mag_);
  }
}

void AttitudeEstimator::SetAxisMode(AxisMode mode) {
  axis_mode_ = mode;

  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ptr_->SetAxisMode(toCompAxisMode(mode));
  } else {
    rk4_filter_ptr_->SetAxisMode(toRK4AxisMode(mode));
  }
}

void AttitudeEstimator::SetAlphaAcc(double alpha) {
  alpha_acc_ = alpha;
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ptr_->SetAlphaAcc(alpha);
  } else {
    rk4_filter_ptr_->SetAlphaAcc(alpha);
  }
}

void AttitudeEstimator::SetAlphaMag(double alpha) {
  alpha_mag_ = alpha;
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ptr_->SetAlphaMag(alpha);
  } else {
    rk4_filter_ptr_->SetAlphaMag(alpha);
  }
}

double AttitudeEstimator::GetAlphaAcc() const {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    return comp_filter_ptr_->GetAlphaAcc();
  } else {
    return rk4_filter_ptr_->GetAlphaAcc();
  }
}

double AttitudeEstimator::GetAlphaMag() const {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    return comp_filter_ptr_->GetAlphaMag();
  } else {
    return rk4_filter_ptr_->GetAlphaMag();
  }
}

void AttitudeEstimator::SetAccelRejectionThreshold(double threshold) {
  if (algo_ == AlgorithmType::COMPLEMENTARY) {
    comp_filter_ptr_->SetAccelRejectionThreshold(threshold);
  } else {
    rk4_filter_ptr_->SetAccelRejectionThreshold(threshold);
  }
}

const char* AttitudeEstimator::GetAlgorithmName() const {
  switch (algo_) {
  case AlgorithmType::COMPLEMENTARY:
    return "complementary";
  case AlgorithmType::RK4:
    return "rk4";
  default:
    return "unknown";
  }
}

const char* AttitudeEstimator::GetAxisModeName() const {
  switch (axis_mode_) {
  case AxisMode::SIX_AXIS:
    return "6-axis";
  case AxisMode::NINE_AXIS:
    return "9-axis";
  default:
    return "unknown";
  }
}

AttitudeEstimator::AlgorithmType AttitudeEstimator::AlgorithmFromString(const std::string& name) {
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "rk4" || lower == "runge_kutta" || lower == "rungekutta") {
    return AlgorithmType::RK4;
  }
  // 默认返回互补滤波
  return AlgorithmType::COMPLEMENTARY;
}

AttitudeEstimator::AxisMode AttitudeEstimator::AxisModeFromString(const std::string& name) {
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "6" || lower == "6axis" || lower == "six_axis" || lower == "six" || lower == "6-axis" ||
      lower == "6_axis") {
    return AxisMode::SIX_AXIS;
  }
  // 默认返回 9 轴
  return AxisMode::NINE_AXIS;
}

ComplementaryFilter::AxisMode AttitudeEstimator::toCompAxisMode(AxisMode mode) {
  return (mode == AxisMode::SIX_AXIS) ? ComplementaryFilter::AxisMode::SIX_AXIS
                                      : ComplementaryFilter::AxisMode::NINE_AXIS;
}

RK4Integration::AxisMode AttitudeEstimator::toRK4AxisMode(AxisMode mode) {
  return (mode == AxisMode::SIX_AXIS) ? RK4Integration::AxisMode::SIX_AXIS : RK4Integration::AxisMode::NINE_AXIS;
}

} // namespace imu_algorithm
