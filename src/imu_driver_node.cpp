#include "imu_driver_node.h"
#include "algorithm/madgwick_filter.h"
#include "algorithm/mahony_filter.h"
#include <ros/ros.h>

ImuDriverNode::ImuDriverNode(ros::NodeHandle& nh)
    : nh_(nh), total_frames_(0), failed_frames_(0), last_diag_time_(ros::Time::now()) {
  loadParams();
}

void ImuDriverNode::loadParams() {
  // 串口参数
  nh_.param<std::string>("port", port_, "/dev/ttyACM0");
  nh_.param<int>("baud", baud_, 115200);
  nh_.param<int>("timeout_ms", timeout_ms_, 100);

  // 发布参数
  nh_.param<bool>("publish_custom", publish_custom_, true);
  nh_.param<bool>("publish_sensor_msgs", publish_sensor_msgs_, false);
  nh_.param<std::string>("frame_id", frame_id_, "imu_link");

  // 姿态解算参数（仅支持 madgwick 与 mahony）
  nh_.param<bool>("enable_attitude_estimation", enable_attitude_estimation_, true);
  nh_.param<std::string>("algorithm_type", algorithm_type_, "madgwick");
  nh_.param<std::string>("axis_mode", axis_mode_, "9");
  nh_.param<double>("beta", beta_, 0.5);
  nh_.param<double>("zeta", zeta_, 0.0);
  nh_.param<double>("kp", kp_, 5.0);
  nh_.param<double>("ki", ki_, 0.05);

  ROS_INFO("Parameters loaded:");
  ROS_INFO("  port = %s", port_.c_str());
  ROS_INFO("  baud = %d", baud_);
  ROS_INFO("  timeout_ms = %d", timeout_ms_);
  ROS_INFO("  publish_custom = %s", publish_custom_ ? "true" : "false");
  ROS_INFO("  publish_sensor_msgs = %s", publish_sensor_msgs_ ? "true" : "false");
  ROS_INFO("  frame_id = %s", frame_id_.c_str());
  ROS_INFO("  enable_attitude_estimation = %s", enable_attitude_estimation_ ? "true" : "false");
  ROS_INFO("  algorithm_type = %s", algorithm_type_.c_str());
  ROS_INFO("  axis_mode = %s", axis_mode_.c_str());
  ROS_INFO("  beta = %.4f (rad/s)", beta_);
  ROS_INFO("  zeta = %.4f", zeta_);
}

bool ImuDriverNode::Init() {
  serial_ptr_ = std::make_unique<SerialPort>(port_, baud_, timeout_ms_);
  if (!serial_ptr_->Open()) {
    ROS_ERROR("Failed to initialize serial port");
    return false;
  }

  parser_ptr_ = std::make_unique<ImuParser>();

  if (enable_attitude_estimation_) {
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

    ROS_INFO("Attitude estimator created: algorithm=%s, axis_mode=%s, "
             "beta=%.4f, zeta=%.4f, kp=%.4f, ki=%.4f",
             attitude_estimator_ptr_->GetAlgorithmName(), attitude_estimator_ptr_->GetAxisModeName(), beta_, zeta_, kp_,
             ki_);
  } else {
    attitude_estimator_ptr_ = nullptr;
    ROS_INFO("Attitude estimation disabled, publishing raw data with identity quaternion");
  }

  publisher_ptr_ =
      std::make_unique<ImuPublisher>(nh_, publish_custom_, publish_sensor_msgs_, frame_id_, attitude_estimator_ptr_);

  last_diag_time_ = ros::Time::now();
  return true;
}

void ImuDriverNode::Run() {
  uint8_t data_buffer[ImuParser::READ_BUF_SIZE];

  while (ros::ok()) {
    size_t n = serial_ptr_->Read(data_buffer, sizeof(data_buffer));
    if (n == 0) {
      ros::spinOnce();
      continue;
    }

    parser_ptr_->Feed(data_buffer, n);

    ImuRawData raw;
    while (parser_ptr_->Parse(raw)) {
      if (total_frames_ == std::numeric_limits<size_t>::max())
        total_frames_ = 0;
      else
        ++total_frames_;

      publisher_ptr_->Publish(raw, ros::Time::now());
    }

    // 定期发布诊断信息
    publishDiagnostics();

    ros::spinOnce();
  }
}

void ImuDriverNode::Shutdown() {
  if (serial_ptr_) {
    serial_ptr_->Close();
  }
  ROS_INFO("IMU driver node shutdown. Total frames: %zu, Checksum fails: %zu", total_frames_,
           parser_ptr_ ? parser_ptr_->ChecksumFailCount() : 0);
}

void ImuDriverNode::publishDiagnostics() {
  const ros::Time now           = ros::Time::now();
  const double    diag_interval = 60.0; // 每 60 秒输出一次诊断

  if ((now - last_diag_time_).toSec() < diag_interval) {
    return;
  }

  size_t fails = parser_ptr_ ? parser_ptr_->ChecksumFailCount() : 0;
  ROS_INFO("Diagnostics: frames_ok=%zu, checksum_fails=%zu", total_frames_, fails);

  last_diag_time_ = now;
}
