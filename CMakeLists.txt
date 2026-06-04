cmake_minimum_required(VERSION 3.8)

project(imu_driver_ros)

# 构建类型与 C++ 标准
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
)

# Find Boost
find_package(Boost REQUIRED COMPONENTS system)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# include directories
include_directories(
  "include"
  "src"
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 自定义消息
add_message_files(
  FILES
  ImuData.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs geometry_msgs
  INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR}
)

# 算法库源文件
set(ALGORITHM_SOURCES
  src/algorithm/complementary_filter.cpp
  src/algorithm/rk4_integration.cpp
  src/algorithm/attitude_estimator.cpp
)

# 主节点：多源文件
add_executable(imu_ros_publisher
  src/imu_ros_publisher.cpp
  src/serial_port.cpp
  src/imu_parser.cpp
  src/imu_publisher.cpp
  src/imu_driver_node.cpp
  ${ALGORITHM_SOURCES}
)

add_dependencies(imu_ros_publisher
  ${PROJECT_NAME}_EXPORTED_TARGETS
  ${catkin_EXPORTED_TARGETS}
)

target_compile_features(imu_ros_publisher PUBLIC cxx_std_14)

target_link_libraries(imu_ros_publisher
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
