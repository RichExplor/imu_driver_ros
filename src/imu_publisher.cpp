#include "imu_publisher.h"

ImuPublisher::ImuPublisher(ros::NodeHandle& nh, bool publish_custom, bool publish_sensor_msgs,
                           const std::string& frame_id, std::shared_ptr<imu_algorithm::AttitudeEstimator> estimator)
    : publish_custom_(publish_custom), publish_sensor_msgs_(publish_sensor_msgs), frame_id_(frame_id),
      estimator_ptr_(estimator), first_frame_(true) {

  if (publish_custom_) {
    pub_custom_ = nh.advertise<imu_driver_ros::ImuData>("/A0110E/imu/data_raw", 10);
    ROS_INFO("Publishing custom ImuData on /A0110E/imu/data_raw");
  }

  if (publish_sensor_msgs) {
    pub_imu_ = nh.advertise<sensor_msgs::Imu>("/A0110E/imu/data", 10);
    pub_mag_ = nh.advertise<sensor_msgs::MagneticField>("/A0110E/imu/mag", 10);
    ROS_INFO("Publishing sensor_msgs/Imu on /A0110E/imu/data, "
             "sensor_msgs/MagneticField on /A0110E/imu/mag");
  }

  if (estimator_ptr_) {
    ROS_INFO("Attitude estimation enabled: algorithm=%s, axis_mode=%s", estimator_ptr_->GetAlgorithmName(),
             estimator_ptr_->GetAxisModeName());
  }
}

void ImuPublisher::Publish(const ImuRawData& raw, const ros::Time& stamp) {
  // 1. 将原始数据转换为 SI 单位，并应用缩放系数
  geometry_msgs::Vector3    linear_acceleration;
  geometry_msgs::Vector3    angular_velocity;
  geometry_msgs::Vector3    magnetic_field;
  geometry_msgs::Quaternion orientation;

  // 2. 原始数据单位量级转换
  convertRawData(raw, stamp, linear_acceleration, angular_velocity, magnetic_field, orientation);

  // 3. 发布自定义消息和标准消息
  if (publish_custom_) {
    imu_driver_ros::ImuData custom_msg;
    custom_msg.header.stamp    = stamp;
    custom_msg.header.frame_id = frame_id_;

    custom_msg.orientation         = orientation;
    custom_msg.linear_acceleration = linear_acceleration;
    custom_msg.angular_velocity    = angular_velocity;
    custom_msg.magnetic_field      = magnetic_field;

    custom_msg.valid = raw.valid;
    pub_custom_.publish(custom_msg);
  }

  if (publish_sensor_msgs_) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp    = stamp;
    imu_msg.header.frame_id = frame_id_;

    imu_msg.orientation = orientation;
    fillCovariance(imu_msg.orientation_covariance.data(), !estimator_ptr_);

    imu_msg.linear_acceleration = linear_acceleration;
    fillCovariance(imu_msg.linear_acceleration_covariance.data(), false);

    imu_msg.angular_velocity = angular_velocity;
    fillCovariance(imu_msg.angular_velocity_covariance.data(), false);

    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.stamp    = stamp;
    mag_msg.header.frame_id = frame_id_;
    mag_msg.magnetic_field  = magnetic_field;
    fillCovariance(mag_msg.magnetic_field_covariance.data(), false);

    // 发布 sensor_msgs/Imu
    pub_imu_.publish(imu_msg);

    // 发布 sensor_msgs/MagneticField
    pub_mag_.publish(mag_msg);
  }
}

void ImuPublisher::fillCovariance(double cov[9], bool unknown) {
  // 全部置零
  std::fill(cov, cov + 9, COV_ZERO);

  if (unknown) {
    cov[0] = COV_UNKNOWN;
  }
}

void ImuPublisher::convertRawData(const ImuRawData& raw, const ros::Time& stamp, geometry_msgs::Vector3& accel,
                                  geometry_msgs::Vector3& gyro, geometry_msgs::Vector3& mag,
                                  geometry_msgs::Quaternion& quat) {
  // 1. 线性加速度
  if (raw.has_accel) {
    accel.x = static_cast<double>(raw.ax) * SCALE_ACCEL;
    accel.y = static_cast<double>(raw.ay) * SCALE_ACCEL;
    accel.z = static_cast<double>(raw.az) * SCALE_ACCEL;
  } else {
    accel.x = accel.y = accel.z = 0.0;
  }

  // 2. 角速度（原数据为 deg/s * 1e-6，转换为 rad/s）
  if (raw.has_gyro) {
    gyro.x = static_cast<double>(raw.wx) * SCALE_GYRO * Deg2Rad;
    gyro.y = static_cast<double>(raw.wy) * SCALE_GYRO * Deg2Rad;
    gyro.z = static_cast<double>(raw.wz) * SCALE_GYRO * Deg2Rad;
  } else {
    gyro.x = gyro.y = gyro.z = 0.0;
  }

  // 3. 磁场：优先使用强度（0x31），否则使用归一化（0x30）
  if (raw.has_mag_strength) {
    mag.x = static_cast<double>(raw.mx) * SCALE_MAG_STRENGTH_TO_TESLA * SCALE_MAG;
    mag.y = static_cast<double>(raw.my) * SCALE_MAG_STRENGTH_TO_TESLA * SCALE_MAG;
    mag.z = static_cast<double>(raw.mz) * SCALE_MAG_STRENGTH_TO_TESLA * SCALE_MAG;
  } else if (raw.has_mag_norm) {
    mag.x = static_cast<double>(raw.hx) * SCALE_MAG;
    mag.y = static_cast<double>(raw.hy) * SCALE_MAG;
    mag.z = static_cast<double>(raw.hz) * SCALE_MAG;
  } else {
    mag.x = mag.y = mag.z = 0.0;
  }

  // 4. 如果有四元数直接使用；否则进行姿态解算（需要 accel + gyro）
  quat.x = quat.y = quat.z = 0.0;
  quat.w                   = 1.0;

  if (raw.has_quat) {
    quat.x = static_cast<double>(raw.q1) * SCALE_NOT_DIMENSIONAL;
    quat.y = static_cast<double>(raw.q2) * SCALE_NOT_DIMENSIONAL;
    quat.z = static_cast<double>(raw.q3) * SCALE_NOT_DIMENSIONAL;
    quat.w = static_cast<double>(raw.q0) * SCALE_NOT_DIMENSIONAL;
  } else if (estimator_ptr_ && raw.has_accel && raw.has_gyro) {
    attitudeEstimate(accel, gyro, mag, quat, stamp);
  } else if (raw.has_euler) {
    double             pitch = static_cast<double>(raw.pitch) * SCALE_NOT_DIMENSIONAL * Deg2Rad;
    double             roll  = static_cast<double>(raw.roll) * SCALE_NOT_DIMENSIONAL * Deg2Rad;
    double             yaw   = static_cast<double>(raw.yaw) * SCALE_NOT_DIMENSIONAL * Deg2Rad;
    Eigen::AngleAxisd  rx(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd  ry(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd  rz(yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = rz * ry * rx;
    quat.x               = q.x();
    quat.y               = q.y();
    quat.z               = q.z();
    quat.w               = q.w();
  }
}

void ImuPublisher::attitudeEstimate(const geometry_msgs::Vector3& accel, const geometry_msgs::Vector3& gyro,
                                    const geometry_msgs::Vector3& mag, geometry_msgs::Quaternion& orientation,
                                    const ros::Time& stamp) {
  if (!estimator_ptr_) {
    // 无姿态解算，使用默认单位四元数
    orientation.x = orientation.y = orientation.z = 0.0;
    orientation.w                                 = 1.0;
    return;
  }

  // 0. 计算 dt
  double dt = 0.0;
  if (first_frame_) {
    first_frame_ = false;
    dt           = 0.01; // 首帧默认 10ms
  } else {
    dt = (stamp - last_stamp_).toSec();

    if (dt <= 0.0 || dt > 1.0) {
      dt = 0.01; // 异常 dt，使用默认值
    }
  }
  last_stamp_ = stamp;

  // 1. 将 geometry_msgs 转换为 Eigen 向量
  Eigen::Vector3d gyro_vec(gyro.x, gyro.y, gyro.z);
  Eigen::Vector3d accel_vec(accel.x, accel.y, accel.z);
  Eigen::Vector3d mag_vec(mag.x, mag.y, mag.z);

  // 2. 更新姿态解算器
  if (estimator_ptr_->GetAxisMode() == imu_algorithm::AttitudeEstimator::AxisMode::NINE_AXIS) {
    estimator_ptr_->Update(gyro_vec, accel_vec, mag_vec, dt);
  } else {
    estimator_ptr_->Update(gyro_vec, accel_vec, dt);
  }

  // 3. 从姿态解算器获取四元数
  const Eigen::Quaterniond& q = estimator_ptr_->Quaternion();
  orientation.x               = q.x();
  orientation.y               = q.y();
  orientation.z               = q.z();
  orientation.w               = q.w();
}
