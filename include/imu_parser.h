#pragma once

#include <array>
#include <cstdint>
#include <vector>

/// @brief IMU 原始数据（解析后、缩放前）
struct ImuRawData {
  // 原始数据均为小端 32-bit（除温度为 16-bit）
  int32_t ax, ay, az; ///< 线性加速度原始值（每轴 4 字节，Value = DATA * 1e-6 m/s^2）
  int32_t wx, wy, wz; ///< 角速度原始值（每轴 4 字节，Value = DATA * 1e-6 deg/s）
  int32_t hx, hy, hz; ///< 磁场归一化原始值（ID 0x30，每轴 4 字节，Value = DATA * 1e-6）
  int32_t mx, my, mz; ///< 磁场强度原始值（ID 0x31，每轴 4 字节，Value = DATA * 0.001 mGauss）

  // 可选的姿态/欧拉/四元数
  int32_t q0, q1, q2, q3; ///< 四元数分量（每分量 4 字节，Value = DATA * 1e-6）
  int32_t pitch, roll, yaw; ///< 欧拉角（ID 0x40 顺序：Pitch, Roll, Yaw，每轴 4 字节，Value = DATA * 1e-6 deg）

  int16_t temp; ///< 温度（2 字节，Value = DATA * 0.01 °C）

  // 字段存在标志
  bool has_accel;
  bool has_gyro;
  bool has_mag_norm;     // 0x30
  bool has_mag_strength; // 0x31
  bool has_quat;         // 0x41
  bool has_euler;        // 0x40
  bool has_temp;         // 0x01

  bool valid; ///< 校验是否通过

  ImuRawData()
      : ax(0), ay(0), az(0), wx(0), wy(0), wz(0), hx(0), hy(0), hz(0), mx(0), my(0), mz(0), q0(0), q1(0), q2(0), q3(0),
        pitch(0), roll(0), yaw(0), temp(0), has_accel(false), has_gyro(false), has_mag_norm(false),
        has_mag_strength(false), has_quat(false), has_euler(false), has_temp(false), valid(false) {
  }
};

/// @brief IMU 二进制协议解析器（新协议：header=0x59,0x53）
class ImuParser {
public:
  ImuParser();

  /// @brief 向解析器输入新数据
  void Feed(const uint8_t* data, size_t len);

  /// @brief 尝试解析一帧
  /// @return 成功解析返回 true（out 填充），否则返回 false
  bool Parse(ImuRawData& out);

  /// @brief 获取校验失败计数
  size_t ChecksumFailCount() const {
    return checksum_fails_;
  }

  // 协议常量 — 公开以便测试
  static constexpr uint8_t HEADER_BYTE0  = 0x59;
  static constexpr uint8_t HEADER_BYTE1  = 0x53;
  static constexpr size_t  HEADER_SIZE   = 2;
  static constexpr size_t  READ_BUF_SIZE = 512; ///< 单次读取缓冲区大小

private:
  /// @brief 验证从 pos 开始的帧（pos 指向帧头第一个字节）的校验和
  bool validateChecksum(size_t pos) const;

  /// @brief 从缓冲区读取小端 int16 / int32
  int16_t readI16(size_t offset) const;
  int32_t readI32(size_t offset) const;

  std::vector<uint8_t>                buf_;
  static const std::array<uint8_t, 2> HEADER_;
  size_t                              checksum_fails_;
  size_t                              frame_count_;
};
