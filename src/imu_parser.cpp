#include "imu_parser.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <cstddef>

const std::array<uint8_t, 2> ImuParser::HEADER_ = {ImuParser::HEADER_BYTE0, ImuParser::HEADER_BYTE1};

ImuParser::ImuParser() : checksum_fails_(0), frame_count_(0) {
  buf_.reserve(READ_BUF_SIZE);
}

void ImuParser::Feed(const uint8_t* data, size_t len) {
  buf_.insert(buf_.end(), data, data + len);
}

bool ImuParser::Parse(ImuRawData& out) {
  // 需要至少 header(2) + TID(2) + LEN(1) + CK(2) = 7 字节
  while (buf_.size() >= HEADER_SIZE + 2 + 1 + 2) {

    // 搜索帧头
    auto it = std::search(buf_.begin(), buf_.end(), HEADER_.begin(), HEADER_.end());
    if (it == buf_.end()) {
      if (buf_.size() > HEADER_SIZE - 1) {
        buf_.erase(buf_.begin(), buf_.end() - (HEADER_SIZE - 1));
      }
      return false;
    }

    size_t pos = static_cast<size_t>(std::distance(buf_.begin(), it));

    // 如果剩余数据不足以读取 TID+LEN，等待更多数据
    if (buf_.size() - pos < HEADER_SIZE + 3) {
      return false;
    }

    // 读取 LEN 字段（pos+2, pos+3: TID; pos+4: LEN）
    uint8_t len = buf_[pos + HEADER_SIZE + 2];

    size_t full_frame_len = HEADER_SIZE + 2 + 1 + static_cast<size_t>(len) + 2; // header + TID + LEN + MESSAGE + CK

    if (buf_.size() - pos < full_frame_len) {
      // 等待更多字节
      return false;
    }

    // 校验和验证（从 TID 开始累加到 MESSAGE 结束，包含 LEN 字节）
    if (!validateChecksum(pos)) {
      if (checksum_fails_ == std::numeric_limits<size_t>::max()) {
        checksum_fails_ = 0;
      } else {
        ++checksum_fails_;
      }

      // 跳过当前帧头的第一个字节，重新搜索
      buf_.erase(buf_.begin() + pos);
      continue;
    }

    // 解析 MESSAGE 内的数据包
    size_t msg_start = pos + HEADER_SIZE + 2 + 1; // header + TID + LEN
    size_t msg_end   = msg_start + len;

    // 重置输出
    out       = ImuRawData();
    out.valid = true;

    size_t i = msg_start;
    while (i + 2 <= msg_end) {
      uint8_t data_id = buf_[i];
      uint8_t pkt_len = buf_[i + 1];
      if (i + 2 + pkt_len > msg_end) {
        // 包长度超出，视为解析失败（但帧校验已通过，仍保留帧）
        break;
      }

      const uint8_t* payload = &buf_[i + 2];

      auto read_i32 = [&](size_t off) -> int32_t {
        size_t   o = off;
        uint32_t v = static_cast<uint32_t>(payload[o]) | (static_cast<uint32_t>(payload[o + 1]) << 8) |
                     (static_cast<uint32_t>(payload[o + 2]) << 16) | (static_cast<uint32_t>(payload[o + 3]) << 24);
        return static_cast<int32_t>(v);
      };

      auto read_i16 = [&](size_t off) -> int16_t {
        size_t   o = off;
        uint16_t v = static_cast<uint16_t>(payload[o]) | (static_cast<uint16_t>(payload[o + 1]) << 8);
        return static_cast<int16_t>(v);
      };

      switch (data_id) {
      case 0x01: // 温度（2 字节，单位 0.01 °C）
        if (pkt_len >= 2) {
          out.temp     = read_i16(0);
          out.has_temp = true;
        }
        break;
      case 0x10: // 加速度 (x,y,z) 每轴 4 字节，单位 1e-6 m/s^2
        if (pkt_len >= 12) {
          out.ax        = read_i32(0);
          out.ay        = read_i32(4);
          out.az        = read_i32(8);
          out.has_accel = true;
        }
        break;
      case 0x20: // 角速度 (x,y,z) 每轴 4 字节，单位 1e-6 deg/s
        if (pkt_len >= 12) {
          out.wx       = read_i32(0);
          out.wy       = read_i32(4);
          out.wz       = read_i32(8);
          out.has_gyro = true;
        }
        break;
      case 0x30: // 磁场归一化 (x,y,z) 每轴 4 字节，Value = DATA * 1e-6 (无量纲)
        if (pkt_len >= 12) {
          out.hx           = read_i32(0);
          out.hy           = read_i32(4);
          out.hz           = read_i32(8);
          out.has_mag_norm = true;
        }
        break;
      case 0x31: // 磁场强度 (x,y,z) 每轴 4 字节，Value = DATA * 0.001 mGauss
        if (pkt_len >= 12) {
          out.mx               = read_i32(0);
          out.my               = read_i32(4);
          out.mz               = read_i32(8);
          out.has_mag_strength = true;
        }
        break;
      case 0x40: // 欧拉角 (Pitch, Roll, Yaw) 每轴 4 字节，Value = DATA * 1e-6 deg
        if (pkt_len >= 12) {
          out.pitch     = read_i32(0);
          out.roll      = read_i32(4);
          out.yaw       = read_i32(8);
          out.has_euler = true;
        }
        break;
      case 0x41: // 四元数 q0..q3 每分量 4 字节，单位 1e-6
        if (pkt_len >= 16) {
          out.q0       = read_i32(0);
          out.q1       = read_i32(4);
          out.q2       = read_i32(8);
          out.q3       = read_i32(12);
          out.has_quat = true;
        }
        break;
      default:
        break;
      }

      i += 2 + pkt_len;
    }

    // 移除已处理的帧
    buf_.erase(buf_.begin(), buf_.begin() + full_frame_len + pos);

    // 成功帧计数（wrap to zero）
    if (frame_count_ == std::numeric_limits<size_t>::max())
      frame_count_ = 0;
    else
      ++frame_count_;

    return true;
  }

  return false;
}

bool ImuParser::validateChecksum(size_t pos) const {
  // 双重累加校验：从 TID 开始累加到 MESSAGE 结束（包含 LEN 字节）
  // CK1 = 所有累加字节的低 8 位之和的低 8 位
  // CK2 = 所有累加字节的高 8 位之和的低 8 位（即 CK1 累加过程中的进位累加）
  size_t  tid_pos = pos + HEADER_SIZE;
  uint8_t len     = buf_[pos + HEADER_SIZE + 2];
  size_t  msg_end = pos + HEADER_SIZE + 2 + 1 + static_cast<size_t>(len); // first byte after MESSAGE

  uint8_t ck1 = 0, ck2 = 0;
  for (size_t i = tid_pos; i < msg_end; ++i) {
    ck1 += buf_[i];
    ck2 += ck1;
  }

  // 校验码 CK1, CK2 紧跟在 MESSAGE 之后（小端：CK1 在低地址，CK2 在高地址）
  uint8_t expected_ck1 = buf_[msg_end];
  uint8_t expected_ck2 = buf_[msg_end + 1];
  return (ck1 == expected_ck1) && (ck2 == expected_ck2);
}

int16_t ImuParser::readI16(size_t offset) const {
  return static_cast<int16_t>(static_cast<uint16_t>(buf_[offset]) | (static_cast<uint16_t>(buf_[offset + 1]) << 8));
}

int32_t ImuParser::readI32(size_t offset) const {
  return static_cast<int32_t>(static_cast<uint32_t>(buf_[offset]) | (static_cast<uint32_t>(buf_[offset + 1]) << 8) |
                              (static_cast<uint32_t>(buf_[offset + 2]) << 16) |
                              (static_cast<uint32_t>(buf_[offset + 3]) << 24));
}