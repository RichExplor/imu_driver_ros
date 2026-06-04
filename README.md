# imu_driver_ros

ROS (roscpp) 节点：从串口读取二进制 IMU 数据并发布为结构化消息。

## 功能特性

- 🔌 串口通信封装，支持可配置超时
- 📦 二进制协议解析，自动帧同步与校验和验证
- 📡 支持自定义 `ImuData` 消息和标准 `sensor_msgs/Imu` + `MagneticField` 双通道发布
- 🛡️ 健壮的缓冲区管理，防止越界访问
- 📊 内置诊断统计（帧计数、校验失败计数）

## 架构

```
main → ImuDriverNode
         ├── SerialPort    （串口通信 + 超时读取）
         ├── ImuParser     （协议解析 + 校验）
         └── ImuPublisher  （消息构建 + 发布）
```

| 类 | 文件 | 职责 |
|---|------|------|
| `SerialPort` | `include/serial_port.h` / `src/serial_port.cpp` | 串口打开/关闭、带超时的同步读取 |
| `ImuParser` | `include/imu_parser.h` / `src/imu_parser.cpp` | 字节流缓冲、帧同步、校验和验证、字段提取 |
| `ImuPublisher` | `include/imu_publisher.h` / `src/imu_publisher.cpp` | ROS Publisher 管理、消息构建与缩放 |
| `ImuDriverNode` | `include/imu_driver_node.h` / `src/imu_driver_node.cpp` | 参数读取、模块组合、主循环、诊断 |

## 构建

在 catkin 工作区中构建：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行

### 使用 launch 启动（推荐）

```bash
roslaunch imu_driver_ros imu_ros_publisher.launch
```

### 使用 rosrun 启动并传参

```bash
rosrun imu_driver_ros imu_ros_publisher _port:=/dev/ttyUSB0 _baud:=115200
```

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `port` | string | `/dev/ttyUSB0` | 串口设备路径 |
| `baud` | int | `115200` | 波特率 |
| `timeout_ms` | int | `100` | 串口读取超时（毫秒），0=阻塞 |
| `publish_custom` | bool | `true` | 是否发布自定义 `ImuData` 消息 |
| `publish_sensor_msgs` | bool | `false` | 是否发布标准 `sensor_msgs/Imu` 和 `MagneticField` |
| `frame_id` | string | `imu_link` | TF 坐标系 ID |

## 话题

| 话题 | 消息类型 | 发布条件 |
|------|----------|----------|
| `~/imu_serial` | `imu_driver_ros/ImuData` | `publish_custom=true` |
| `~/imu/data_raw` | `sensor_msgs/Imu` | `publish_sensor_msgs=true` |
| `~/imu/mag` | `sensor_msgs/MagneticField` | `publish_sensor_msgs=true` |

## 消息格式

### `imu_driver_ros/ImuData`

```
std_msgs/Header header
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 linear_acceleration
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 magnetic_field
bool valid
```

- `header`: ROS Header，含时间戳和 `frame_id`
- `orientation`: 四元数，当前默认设为单位四元数（方向未知）
- `valid`: 数据有效状态，校验通过时为 `true`
- `linear_acceleration`: 线性加速度（原始 int16 值转 double）
- `angular_velocity`: 角速度（原始 int16 值转 double）
- `magnetic_field`: 磁场（原始 int16 值转 double）

## 串口协议格式

每帧 24 字节，小端字节序：

| 偏移 | 长度 | 内容 |
|------|------|------|
| 0-3 | 4 | 帧头 `0x4E 0x4A 0x13 0x01` |
| 4-5 | 2 | 加速度 X (int16) |
| 6-7 | 2 | 加速度 Y (int16) |
| 8-9 | 2 | 加速度 Z (int16) |
| 10-11 | 2 | 角速度 X (int16) |
| 12-13 | 2 | 角速度 Y (int16) |
| 14-15 | 2 | 角速度 Z (int16) |
| 16-17 | 2 | 磁场 X (int16) |
| 18-19 | 2 | 磁场 Y (int16) |
| 20-21 | 2 | 磁场 Z (int16) |
| 22-23 | 2 | 校验和 (uint16，前 22 字节累加和) |

## 故障排除

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| `Failed to open serial port` | 设备不存在或权限不足 | 检查设备路径，添加用户到 `dialout` 组：`sudo usermod -aG dialout $USER` |
| 无数据发布 | 波特率不匹配 | 确认 IMU 模块波特率与 `baud` 参数一致 |
| 大量 `Checksum mismatch` | 串口数据丢失或波特率错误 | 检查串口线缆，降低波特率重试 |
| 节点启动后卡住 | `timeout_ms=0` 且串口无数据 | 设置 `timeout_ms` 为非零值 |
| `ImuData` 话题无数据 | `publish_custom=false` | 设置 `publish_custom` 为 `true` |

## 许可证

MIT

## 格式化代码
find . \( -name "*.c" -o -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) -exec clang-format -i {} \;
