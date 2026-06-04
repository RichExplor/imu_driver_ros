# imu_ros_driver

ROS 1 (roscpp) IMU 串口驱动节点：从串口读取二进制 IMU 数据，经协议解析、单位转换和姿态解算后，发布为结构化 ROS 消息。

## 功能特性

- 🔌 串口通信封装（Boost.Asio），支持可配置超时的同步读取
- 📦 二进制协议解析，自动帧同步、双重累加校验和验证、TLV 子包提取
- 🧭 内置姿态解算：互补滤波 / RK4 四阶龙格库塔，6 轴 / 9 轴可配置
- 📡 双通道发布：自定义 `ImuData` 消息 + 标准 `sensor_msgs/Imu` / `MagneticField`
- 🛡️ 健壮的缓冲区管理，防止越界访问
- 📊 内置诊断统计（帧计数、校验失败计数，每 60 秒输出一次）

## 架构

```
main → ImuDriverNode
├── SerialPort          （串口通信 + 超时读取）
├── ImuParser           （协议解析 + 校验）
└── ImuPublisher        （消息构建 + 发布）
    └── AttitudeEstimator（姿态解算管理器）
        ├── ComplementaryFilter（互补滤波）
        └── RK4Integration     （RK4 积分）
            └── QuaternionUtils（四元数工具）
```

| 类 | 文件 | 职责 |
|---|------|------|
| `SerialPort` | [`inc/serial_port.h`](inc/serial_port.h) / [`src/serial_port.cpp`](src/serial_port.cpp) | 串口打开/关闭、带超时的同步读取（Boost.Asio） |
| `ImuParser` | [`inc/imu_parser.h`](inc/imu_parser.h) / [`src/imu_parser.cpp`](src/imu_parser.cpp) | 字节流缓冲、帧同步、双重累加校验和、TLV 子包提取 |
| `ImuPublisher` | [`inc/imu_publisher.h`](inc/imu_publisher.h) / [`src/imu_publisher.cpp`](src/imu_publisher.cpp) | 原始数据单位转换、姿态解算调用、ROS 消息构建与发布 |
| `ImuDriverNode` | [`inc/imu_driver_node.h`](inc/imu_driver_node.h) / [`src/imu_driver_node.cpp`](src/imu_driver_node.cpp) | 参数读取、模块组合、主循环、诊断输出 |
| `AttitudeEstimator` | [`inc/algorithm/attitude_estimator.h`](inc/algorithm/attitude_estimator.h) / [`src/algorithm/attitude_estimator.cpp`](src/algorithm/attitude_estimator.cpp) | 策略模式管理器，委托给互补滤波或 RK4 |
| `ComplementaryFilter` | [`inc/algorithm/complementary_filter.h`](inc/algorithm/complementary_filter.h) / [`src/algorithm/complementary_filter.cpp`](src/algorithm/complementary_filter.cpp) | 互补滤波姿态解算（6 轴/9 轴） |
| `RK4Integration` | [`inc/algorithm/rk4_integration.h`](inc/algorithm/rk4_integration.h) / [`src/algorithm/rk4_integration.cpp`](src/algorithm/rk4_integration.cpp) | 四阶龙格-库塔高精度积分（6 轴/9 轴） |
| `QuaternionUtils` | [`inc/algorithm/quaternion.h`](inc/algorithm/quaternion.h) | 四元数工具：欧拉角转换、导数计算、向量旋转 |

> 📋 完整架构设计文档见 [`plans/architecture_design.md`](plans/architecture_design.md)

## 构建

在 catkin 工作区中构建：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**依赖**：roscpp, std_msgs, sensor_msgs, geometry_msgs, Boost (system), Eigen3

**编译失败**：自定义消息编译问题，仅首次编译会出现，重新catkin_make即可

## 运行

### 使用 launch 启动（推荐）

```bash
roslaunch imu_ros_driver imu_ros_publisher.launch
```

### 使用 rosrun 启动并传参

```bash
rosrun imu_ros_driver imu_ros_publisher _port:=/dev/ttyUSB0 _baud:=115200
```

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `baud` | int | `115200` | 波特率 |
| `timeout_ms` | int | `100` | 串口读取超时（毫秒），0=阻塞（内部强制 100ms） |
| `publish_custom` | bool | `true` | 是否发布自定义 `ImuData` 消息 |
| `publish_sensor_msgs` | bool | `false` | 是否发布标准 `sensor_msgs/Imu` 和 `MagneticField` |
| `frame_id` | string | `imu_link` | TF 坐标系 ID |
| `enable_attitude_estimation` | bool | `true` | 是否启用姿态解算 |
| `algorithm_type` | string | `complementary` | 算法类型：`complementary`（互补滤波）或 `rk4`（四阶龙格库塔） |
| `axis_mode` | string | `9` | 轴数模式：`6`（加速度计+陀螺仪）或 `9`（加速度计+陀螺仪+磁力计） |
| `alpha_acc` | double | `0.02` | 加速度计融合/修正系数（0~1），越大越信任加速度计 |
| `alpha_mag` | double | `0.01` | 磁力计融合/修正系数（0~1），越大越信任磁力计 |

## 话题

| 话题 | 消息类型 | 发布条件 | 队列 |
|------|----------|----------|------|
| `~/imu/data_serial` | `imu_ros_driver/ImuData` | `publish_custom=true` | 1 |
| `~/imu/data_raw` | `sensor_msgs/Imu` | `publish_sensor_msgs=true` | 10 |
| `~/imu/mag` | `sensor_msgs/MagneticField` | `publish_sensor_msgs=true` | 10 |

## 消息格式

### `imu_ros_driver/ImuData`

```
std_msgs/Header header
geometry_msgs/Quaternion orientation        # 姿态四元数
geometry_msgs/Vector3 linear_acceleration   # 线性加速度 (m/s²)
geometry_msgs/Vector3 angular_velocity      # 角速度 (rad/s)
geometry_msgs/Vector3 magnetic_field        # 磁场 (Tesla)
bool valid                                  # 校验是否通过
```

- `header`：ROS Header，含时间戳和 `frame_id`
- `orientation`：姿态四元数，优先级：硬件四元数 > 姿态解算 > 欧拉角转换 > 单位四元数
- `valid`：数据有效状态，校验通过时为 `true`

## 姿态解算

### 算法对比

| 特性 | 互补滤波 | RK4 |
|------|----------|-----|
| 积分精度 | O(dt²) 一阶欧拉 | O(dt⁵) 四阶 |
| 计算量 | 低 | 中 |
| 适用场景 | 一般动态、嵌入式 | 高动态、低采样率 |

### 轴数模式

| 模式 | 传感器 | Roll/Pitch | Yaw | 适用场景 |
|------|--------|------------|-----|----------|
| 6 轴 | 加速度计 + 陀螺仪 | 无漂移 | 会漂移 | 无磁场环境 |
| 9 轴 | 加速度计 + 陀螺仪 + 磁力计 | 无漂移 | 无漂移 | 全姿态需求 |

### 姿态四元数优先级

1. **硬件四元数**（`has_quat=true`）：直接使用 IMU 内置四元数
2. **姿态解算**（`estimator` 启用 + `has_accel` + `has_gyro`）：调用 `AttitudeEstimator`
3. **欧拉角转换**（`has_euler=true`）：欧拉角 → 四元数（ZYX 旋转顺序）
4. **单位四元数**：默认 `(0, 0, 0, 1)`

## 串口协议格式

帧头 `0x59 0x53`，小端字节序，TLV 子包结构：

| 偏移 | 长度 | 内容 |
|------|------|------|
| 0-1 | 2 | 帧头 `0x59 0x53` |
| 2-3 | 2 | 事务 ID (TID) |
| 4 | 1 | MESSAGE 长度 (LEN) |
| 5~5+LEN-1 | LEN | TLV 子包序列 |
| 5+LEN | 1 | 校验和 CK1 |
| 5+LEN+1 | 1 | 校验和 CK2 |

### TLV 子包 ID

| Data ID | 载荷长度 | 内容 | 单位 |
|---------|----------|------|------|
| 0x01 | 2 | 温度 | 0.01 °C |
| 0x10 | 12 | 加速度 X/Y/Z | 1e-6 m/s² |
| 0x20 | 12 | 角速度 X/Y/Z | 1e-6 deg/s |
| 0x30 | 12 | 磁场归一化 X/Y/Z | 1e-6 (无量纲) |
| 0x31 | 12 | 磁场强度 X/Y/Z | 0.001 mGauss |
| 0x40 | 12 | 欧拉角 Pitch/Roll/Yaw | 1e-6 deg |
| 0x41 | 16 | 四元数 q0/q1/q2/q3 | 1e-6 |

### 校验和算法

双重累加校验，累加范围从 TID 到 MESSAGE 结束（含 LEN 字节）：

```
CK1 = Σ(byte[i]) & 0xFF
CK2 = Σ(CK1_running_sum) & 0xFF
```

## 故障排除

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 自定义消息编译失败 | 仅首次编译会出现 | 重新catkin_make即可 |
| `Failed to open serial port` | 设备不存在或权限不足 | 检查设备路径，添加用户到 `dialout` 组：`sudo usermod -aG dialout $USER` |
| 无数据发布 | 波特率不匹配 | 确认 IMU 模块波特率与 `baud` 参数一致 |
| 大量 `Checksum mismatch` | 串口数据丢失或波特率错误 | 检查串口线缆，降低波特率重试 |
| 节点启动后卡住 | `timeout_ms=0` 且串口无数据 | 设置 `timeout_ms` 为非零值 |
| `ImuData` 话题无数据 | `publish_custom=false` | 设置 `publish_custom` 为 `true` |
| 姿态四元数始终为单位四元数 | 姿态解算未启用或缺少传感器数据 | 检查 `enable_attitude_estimation` 和传感器数据 |
| Yaw 角漂移 | 使用 6 轴模式 | 切换为 9 轴模式 `axis_mode:=9` |
| 加速度计修正失效 | 加速度模长偏离重力 | 检查 `alpha_acc` 和加速度计异常检测阈值 |

## 格式化代码

```bash
find . \( -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) -exec clang-format -i {} \;
```

## 许可证

MIT
