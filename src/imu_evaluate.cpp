#include "algorithm/attitude_estimator.h"
#include "algorithm/madgwick_filter.h"
#include "algorithm/mahony_filter.h"
#include <cmath>
#include <fstream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <tf/tf.h>

class ImuMagSyncProcessor {
private:
  ros::NodeHandle nh_;

  // 订阅者
  message_filters::Subscriber<sensor_msgs::Imu>           imu_sub_;
  message_filters::Subscriber<sensor_msgs::MagneticField> mag_sub_;

  // 同步策略：使用近似时间同步（允许微小时间差）
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::MagneticField> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy>                                                             sync_;

  std::ofstream csv_file_;
  ros::Time     last_time_;

  std::shared_ptr<imu_algorithm::AttitudeEstimator> attitude_estimator_ptr_;

  // 姿态解算参数（从 ROS 参数服务器读取）
  std::string algorithm_type_ = "madgwick"; ///< 算法类型："mahony" | "madgwick"
  std::string axis_mode_      = "9";        ///< 轴数模式："6" 或 "9"
  double      beta_           = 0.35;       ///< Madgwick 梯度下降步长 (rad/s)
  double      zeta_           = 0.01;       ///< Madgwick 陀螺仪零偏补偿系数
  double      kp_             = 5.0;        ///< Mahony 比例增益
  double      ki_             = 0.05;       ///< Mahony 积分增益

public:
  ImuMagSyncProcessor()
      : imu_sub_(nh_, "/imu/data", 10), mag_sub_(nh_, "/imu/mag", 10), sync_(SyncPolicy(10), imu_sub_, mag_sub_) {
    // 从参数服务器读取配置
    nh_.param<std::string>("algorithm_type", algorithm_type_, "madgwick");
    nh_.param<std::string>("axis_mode", axis_mode_, "6");
    nh_.param<double>("beta", beta_, 0.3);
    nh_.param<double>("zeta", zeta_, 0.0);
    nh_.param<double>("kp", kp_, 5.0);
    nh_.param<double>("ki", ki_, 0.05);

    // 打开 CSV 文件
    std::string csv_path;
    nh_.param("csv_file", csv_path, std::string("/home/guofeng/imu_data/imu_filter_output.csv"));
    csv_file_.open(csv_path);
    if (!csv_file_.is_open()) {
      ROS_ERROR("Cannot open CSV file: %s", csv_path.c_str());
      ros::shutdown();
      return;
    }
    // 写入 CSV 表头
    csv_file_ << "timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,"
              << "roll_raw,pitch_raw,yaw_raw,"
              << "roll_filter,pitch_filter,yaw_filter\n";
    csv_file_.flush();

    sync_.registerCallback(boost::bind(&ImuMagSyncProcessor::callback, this, _1, _2));

    auto axis_mode = imu_algorithm::AttitudeEstimator::AxisModeFromString(axis_mode_);

    std::string algo_lower = algorithm_type_;
    std::transform(algo_lower.begin(), algo_lower.end(), algo_lower.begin(), ::tolower);

    if (algo_lower.find("madgwick") != std::string::npos) {
      attitude_estimator_ptr_ = std::make_shared<imu_algorithm::MadgwickFilter>(axis_mode, beta_, zeta_);
    } else if (algo_lower.find("mahony") != std::string::npos) {
      attitude_estimator_ptr_ = std::make_shared<imu_algorithm::MahonyFilter>(axis_mode, kp_, ki_);
    } else {
      ROS_WARN("Unknown algorithm '%s', defaulting to madgwick", algorithm_type_.c_str());
      attitude_estimator_ptr_ = std::make_shared<imu_algorithm::MadgwickFilter>(axis_mode, beta_, zeta_);
    }

    ROS_INFO("IMU/Mag synchronizer started. Algorithm: %s, Axis: %s, "
             "beta=%.4f, zeta=%.4f, kp=%.4f, ki=%.4f",
             attitude_estimator_ptr_->GetAlgorithmName(), attitude_estimator_ptr_->GetAxisModeName(), beta_, zeta_, kp_,
             ki_);
  }

  // 同步回调函数：当 imu 和 mag 的时间戳对齐后调用
  void callback(const sensor_msgs::Imu::ConstPtr& imu_msg, const sensor_msgs::MagneticField::ConstPtr& mag_msg) {
    ROS_INFO("Received synchronized data: imu_stamp=%.3f, mag_stamp=%.3f", imu_msg->header.stamp.toSec(),
             mag_msg->header.stamp.toSec());

    // 1. 提取数据
    double ax = imu_msg->linear_acceleration.x;
    double ay = imu_msg->linear_acceleration.y;
    double az = imu_msg->linear_acceleration.z;
    double gx = imu_msg->angular_velocity.x;
    double gy = imu_msg->angular_velocity.y;
    double gz = imu_msg->angular_velocity.z;
    double mx = mag_msg->magnetic_field.x;
    double my = mag_msg->magnetic_field.y;
    double mz = mag_msg->magnetic_field.z;

    // 2. 提取原始姿态（如果有四元数）
    double roll_raw = 0.0, pitch_raw = 0.0, yaw_raw = 0.0;
    if (!std::isnan(imu_msg->orientation.x) && !std::isnan(imu_msg->orientation.w)) {
      tf::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
      tf::Matrix3x3(q).getRPY(roll_raw, pitch_raw, yaw_raw);
    }

    // 计算 dt
    ros::Time now = imu_msg->header.stamp;
    float     dt  = 0.0;
    if (last_time_.isZero()) {
      dt = 1.0 / 50.0; // 默认 50 Hz
    } else {
      dt = (now - last_time_).toSec();
      if (dt <= 0.0 || dt > 0.1)
        dt = 0.02; // 限制异常值
    }
    last_time_ = now;

    // 姿态解算更新
    Eigen::Vector3d gyro_vec(gx, gy, gz);
    Eigen::Vector3d accel_vec(ax, ay, az);
    Eigen::Vector3d mag_vec(mx, my, mz);
    if (attitude_estimator_ptr_->GetAxisMode() == imu_algorithm::AttitudeEstimator::AxisMode::NINE_AXIS) {
      attitude_estimator_ptr_->Update(gyro_vec, accel_vec, mag_vec, dt);
    } else {
      attitude_estimator_ptr_->Update(gyro_vec, accel_vec, dt);
    }

    // 获取滤波后的欧拉角
    double roll_filt, pitch_filt, yaw_filt;
    attitude_estimator_ptr_->EulerAngle(roll_filt, pitch_filt, yaw_filt);

    // 写入 CSV
    csv_file_ << std::fixed << std::setprecision(9) << now.toSec() << "," << ax << "," << ay << "," << az << "," << gx
              << "," << gy << "," << gz << "," << mx << "," << my << "," << mz << "," << roll_raw * 180.0 / M_PI << ","
              << pitch_raw * 180.0 / M_PI << "," << yaw_raw * 180.0 / M_PI << "," << roll_filt * 180.0 / M_PI << ","
              << pitch_filt * 180.0 / M_PI << "," << yaw_filt * 180.0 / M_PI << "\n";

    // 可选：每100行 flush 一次
    static int count = 0;
    if (++count % 100 == 0)
      csv_file_.flush();

    // 3. 输出调试信息
    ROS_INFO_THROTTLE(1,
                      "Acc: %.2f,%.2f,%.2f | Mag: %.2f,%.2f,%.2f | "
                      "RPY: %.1f,%.1f,%.1f deg",
                      ax, ay, az, mx, my, mz, roll_filt * 180.0 / M_PI, pitch_filt * 180.0 / M_PI,
                      yaw_filt * 180.0 / M_PI);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_mag_sync_processor");
  ImuMagSyncProcessor processor;
  ros::spin();
  return 0;
}
