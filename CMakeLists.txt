cmake_minimum_required(VERSION 3.8)

project(imu_driver_ros)

# 构建类型与 C++ 标准
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_filters
)

# Find Boost
find_package(Boost REQUIRED COMPONENTS system)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# 包含目录（规范：只添加 include 和依赖包的目录）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 自定义消息
add_message_files(
  FILES
  ImuData.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# Catkin 打包配置（将自身 include 和 Eigen3 目录传递给依赖包）
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime sensor_msgs geometry_msgs visualization_msgs message_filters
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
)

# 算法库源文件
set(ALGORITHM_SOURCES
src/algorithm/madgwick_filter.cpp
src/algorithm/mahony_filter.cpp
src/algorithm/attitude_estimator.cpp
)

# 可执行文件
add_executable(imu_ros_publisher
  src/imu_ros_publisher.cpp
  src/serial_port.cpp
  src/imu_parser.cpp
  src/imu_publisher.cpp
  src/imu_driver_node.cpp
  ${ALGORITHM_SOURCES}
)

add_executable(imu_simulator
  src/imu_simulator.cpp
  ${ALGORITHM_SOURCES}
)

add_executable(imu_evaluate
  src/imu_evaluate.cpp
  ${ALGORITHM_SOURCES}
)

target_compile_features(imu_ros_publisher PUBLIC cxx_std_17)
target_compile_features(imu_simulator PUBLIC cxx_std_17)
target_compile_features(imu_evaluate PUBLIC cxx_std_17)

target_link_libraries(imu_ros_publisher
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(imu_simulator
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(imu_evaluate
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(imu_ros_publisher
  ${PROJECT_NAME}_EXPORTED_TARGETS
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(imu_simulator
  ${PROJECT_NAME}_EXPORTED_TARGETS
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(imu_evaluate
  ${PROJECT_NAME}_EXPORTED_TARGETS
  ${catkin_EXPORTED_TARGETS}
)