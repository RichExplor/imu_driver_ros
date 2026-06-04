#include "imu_driver_node.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_ros_publisher");
  ros::NodeHandle nh("~");

  ImuDriverNode imu_driver_node(nh);

  if (!imu_driver_node.Init()) {
    ROS_ERROR("Failed to initialize IMU driver node");
    return 1;
  }

  ROS_INFO("IMU driver node started");
  imu_driver_node.Run();
  imu_driver_node.Shutdown();

  return 0;
}
