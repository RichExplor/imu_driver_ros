/**
 * @file imu_simulator.cpp
 * @brief IMU 仿真模拟器节点 —— 生成模拟 IMU 数据并发布标准 ROS topic，
 *        用于在 RViz 中订阅显示 IMU 姿态模型。
 *
 * 发布 topic：
 *   - /imu/data          (sensor_msgs/Imu)         含四元数姿态、角速度、线性加速度
 *   - /imu/mag           (sensor_msgs/MagneticField) 磁场矢量
 *
 * 支持参数：
 *   - publish_rate       (double, 默认 50.0)  发布频率 Hz
 *   - frame_id           (string, 默认 "imu_link") 坐标系 ID
 *   - motion_type        (string, 默认 "sin") 运动模式："static" | "sin" | "rotate"
 *   - gravity            (double, 默认 9.80665) 重力加速度 m/s²
 *   - noise_accel        (double, 默认 0.01)   加速度计噪声标准差 m/s²
 *   - noise_gyro         (double, 默认 0.001)  陀螺仪噪声标准差 rad/s
 *   - noise_mag          (double, 默认 0.0001) 磁力计噪声标准差 T
 *   - sin_freq           (double, 默认 0.5)    正弦运动频率 Hz（sin 模式）
 *   - sin_amplitude      (double, 默认 0.3)    正弦运动幅度 rad（sin 模式）
 *   - rotate_speed       (double, 默认 0.5)    旋转速度 rad/s（rotate 模式）
 */

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/Vector3Stamped.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

/// @brief IMU 仿真模拟器类
class ImuSimulator {
public:
  ImuSimulator(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh), time_(0.0), first_tick_(true) {
    loadParams();
    initPublishers();
    initRNG();
  }

  void run() {
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &ImuSimulator::timerCallback, this);
    ROS_INFO("IMU Simulator started: rate=%.1f Hz, motion=%s, frame=%s", publish_rate_, motion_type_.c_str(),
             frame_id_.c_str());
    ros::spin();
  }

private:
  void loadParams() {
    pnh_.param<double>("publish_rate", publish_rate_, 50.0);
    pnh_.param<std::string>("frame_id", frame_id_, "imu_link");
    pnh_.param<std::string>("motion_type", motion_type_, "sin");
    pnh_.param<double>("gravity", gravity_, 9.80665);
    pnh_.param<double>("noise_accel", noise_accel_, 0.01);
    pnh_.param<double>("noise_gyro", noise_gyro_, 0.001);
    pnh_.param<double>("noise_mag", noise_mag_, 0.0001);
    pnh_.param<double>("sin_freq", sin_freq_, 0.5);
    pnh_.param<double>("sin_amplitude", sin_amplitude_, 0.3);
    pnh_.param<double>("rotate_speed", rotate_speed_, 0.5);

    // 限制 publish_rate 合理范围
    if (publish_rate_ <= 0.0)
      publish_rate_ = 50.0;
    if (publish_rate_ > 500.0)
      publish_rate_ = 500.0;
  }

  void initPublishers() {
    pub_imu_    = nh_.advertise<sensor_msgs::Imu>("/imu/data", 10);
    pub_mag_    = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  }

  void initRNG() {
    rng_.seed(std::random_device{}());
    // 使用零均值的正态分布
    dist_accel_ = std::normal_distribution<double>(0.0, noise_accel_);
    dist_gyro_  = std::normal_distribution<double>(0.0, noise_gyro_);
    dist_mag_   = std::normal_distribution<double>(0.0, noise_mag_);
  }

  /// @brief 根据运动模式计算当前姿态（欧拉角 -> 四元数）和角速度
  void computeMotion(double t, Eigen::Quaterniond& orientation, Eigen::Vector3d& angular_velocity) {
    double roll, pitch, yaw;
    double droll, dpitch, dyaw;

    if (motion_type_ == "static") {
      // 静止模式：水平放置
      roll = pitch = yaw = 0.0;
      droll = dpitch = dyaw = 0.0;
    } else if (motion_type_ == "rotate") {
      // 绕 Z 轴匀速旋转
      roll   = 0.0;
      pitch  = 0.0;
      yaw    = rotate_speed_ * t;
      droll  = 0.0;
      dpitch = 0.0;
      dyaw   = rotate_speed_;
    } else {
      // 默认 sin 模式：三轴正弦摇摆
      double omega = 2.0 * M_PI * sin_freq_;
      double A     = sin_amplitude_;

      roll  = A * std::sin(omega * t);
      pitch = A * std::sin(omega * t * 0.7 + M_PI / 3.0);
      yaw   = A * std::sin(omega * t * 0.3 + M_PI / 6.0);

      droll  = A * omega * std::cos(omega * t);
      dpitch = A * omega * 0.7 * std::cos(omega * t * 0.7 + M_PI / 3.0);
      dyaw   = A * omega * 0.3 * std::cos(omega * t * 0.3 + M_PI / 6.0);
    }

    // 欧拉角 (ZYX) -> 四元数
    Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
    orientation = rz * ry * rx;

    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cr = std::cos(roll);
    double sr = std::sin(roll);

    double gyro_x = droll - dyaw * sp;
    double gyro_y = dpitch * cr + dyaw * sr * cp;
    double gyro_z = -dpitch * sr + dyaw * cr * cp;

    angular_velocity = Eigen::Vector3d(gyro_x, gyro_y, gyro_z);
  }

  /// @brief 根据姿态计算体坐标系下的线性加速度（含重力补偿）
  Eigen::Vector3d computeAccel(const Eigen::Quaterniond& q) {
    // 体坐标系下的重力分量：a_gravity = R^T * [0, 0, -g]
    // 静止时加速度计读数 = -g 在体坐标系下的投影
    Eigen::Vector3d gravity_world(0.0, 0.0, -gravity_);
    Eigen::Vector3d a_body = q.conjugate() * gravity_world;
    // 加速度计实际读数为 -a_body（因为加速度计感受的是反作用力）
    // 但标准 IMU 模型中，静止时 z 轴读数为 +g
    return -a_body;
  }

  /// @brief 计算地磁场在体坐标系下的投影（简化模型）
  Eigen::Vector3d computeMag(const Eigen::Quaterniond& q) {
    // 假设地磁场在 NED 坐标系下: 北向 ~0.00002 T, 东向 0, 垂直 ~0.00004 T
    // 简化为水平向北 + 垂直向下
    Eigen::Vector3d mag_world(0.00002, 0.0, 0.00004);
    return q.conjugate() * mag_world;
  }

  double noiseAccel() {
    return dist_accel_(rng_);
  }
  double noiseGyro() {
    return dist_gyro_(rng_);
  }
  double noiseMag() {
    return dist_mag_(rng_);
  }

  /// @brief 填充 3x3 协方差矩阵（对角线为 variance，其余为 0）
  void fillCovariance(boost::array<double, 9>& cov, double variance) {
    cov[0] = variance;
    cov[1] = 0.0;
    cov[2] = 0.0;
    cov[3] = 0.0;
    cov[4] = variance;
    cov[5] = 0.0;
    cov[6] = 0.0;
    cov[7] = 0.0;
    cov[8] = variance;
  }

  void timerCallback(const ros::TimerEvent& event) {
    double dt = 1.0 / publish_rate_;
    if (first_tick_) {
      dt          = 1.0 / publish_rate_;
      first_tick_ = false;
    } else {
      dt = (event.current_real - last_time_).toSec();
      if (dt <= 0.0 || dt > 1.0)
        dt = 1.0 / publish_rate_;
    }
    last_time_ = event.current_real;
    time_ += dt;

    // 1. 计算姿态和角速度
    Eigen::Quaterniond orientation;
    Eigen::Vector3d    angular_velocity;
    computeMotion(time_, orientation, angular_velocity);
    orientation.normalize();

    // 2. 计算体坐标系下的加速度和磁场
    Eigen::Vector3d accel = computeAccel(orientation);
    Eigen::Vector3d mag   = computeMag(orientation);

    // 3. 添加高斯噪声
    Eigen::Vector3d accel_noisy(accel.x() /*+ noiseAccel()*/, accel.y() /*+ noiseAccel()*/,
                                accel.z() /*+ noiseAccel()*/);
    Eigen::Vector3d gyro_noisy(angular_velocity.x() /*+ noiseGyro()*/, angular_velocity.y() /*+ noiseGyro()*/,
                               angular_velocity.z() /*+ noiseGyro()*/);
    Eigen::Vector3d mag_noisy(mag.x() /*+ noiseMag()*/, mag.y() /*+ noiseMag()*/, mag.z() /*+ noiseMag()*/);

    // 4. 构建 sensor_msgs/Imu
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp    = ros::Time::now();
    imu_msg.header.frame_id = frame_id_;

    imu_msg.orientation.x = orientation.x();
    imu_msg.orientation.y = orientation.y();
    imu_msg.orientation.z = orientation.z();
    imu_msg.orientation.w = orientation.w();
    fillCovariance(imu_msg.orientation_covariance, noise_gyro_ * noise_gyro_);

    imu_msg.angular_velocity.x = gyro_noisy.x();
    imu_msg.angular_velocity.y = gyro_noisy.y();
    imu_msg.angular_velocity.z = gyro_noisy.z();
    fillCovariance(imu_msg.angular_velocity_covariance, noise_gyro_ * noise_gyro_);

    imu_msg.linear_acceleration.x = accel_noisy.x();
    imu_msg.linear_acceleration.y = accel_noisy.y();
    imu_msg.linear_acceleration.z = accel_noisy.z();
    fillCovariance(imu_msg.linear_acceleration_covariance, noise_accel_ * noise_accel_);

    // 5. 构建 sensor_msgs/MagneticField
    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.stamp     = imu_msg.header.stamp;
    mag_msg.header.frame_id  = frame_id_;
    mag_msg.magnetic_field.x = mag_noisy.x();
    mag_msg.magnetic_field.y = mag_noisy.y();
    mag_msg.magnetic_field.z = mag_noisy.z();
    fillCovariance(mag_msg.magnetic_field_covariance, noise_mag_ * noise_mag_);

    // 6. 发布
    pub_imu_.publish(imu_msg);
    pub_mag_.publish(mag_msg);

    visualization_msgs::Marker cube_marker_;
    cube_marker_.header.frame_id = frame_id_;
    cube_marker_.header.stamp    = ros::Time::now();
    cube_marker_.lifetime        = ros::Duration();
    cube_marker_.ns              = "imu_cube";
    cube_marker_.id              = 0;
    cube_marker_.type            = visualization_msgs::Marker::CUBE;
    cube_marker_.action          = visualization_msgs::Marker::ADD;

    cube_marker_.scale.x = 0.5;
    cube_marker_.scale.y = 0.5;
    cube_marker_.scale.z = 0.5;
    cube_marker_.color.r = 1.0;
    cube_marker_.color.g = 0.0;
    cube_marker_.color.b = 0.0;
    cube_marker_.color.a = 0.7;

    cube_marker_.pose.position.x    = 0.0;
    cube_marker_.pose.position.y    = 0.0;
    cube_marker_.pose.position.z    = 0.5;
    cube_marker_.pose.orientation.x = orientation.x();
    cube_marker_.pose.orientation.y = orientation.y();
    cube_marker_.pose.orientation.z = orientation.z();
    cube_marker_.pose.orientation.w = orientation.w();

    marker_pub_.publish(cube_marker_);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher  pub_imu_;
  ros::Publisher  pub_mag_;
  ros::Publisher  marker_pub_;
  ros::Timer      timer_;

  // 参数
  double      publish_rate_;
  std::string frame_id_;
  std::string motion_type_;
  double      gravity_;
  double      noise_accel_;
  double      noise_gyro_;
  double      noise_mag_;
  double      sin_freq_;
  double      sin_amplitude_;
  double      rotate_speed_;

  // 状态
  double    time_;
  bool      first_tick_;
  ros::Time last_time_;

  // 随机数生成
  std::mt19937                     rng_;
  std::normal_distribution<double> dist_accel_;
  std::normal_distribution<double> dist_gyro_;
  std::normal_distribution<double> dist_mag_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_simulator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ImuSimulator sim(nh, pnh);
  sim.run();

  return 0;
}
