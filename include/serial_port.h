#pragma once

#include <boost/asio.hpp>
#include <cstdint>
#include <memory>
#include <string>

/// @brief 串口通信封装，支持带超时的同步读取
class SerialPort {
public:
  /// @brief 构造函数
  /// @param port 串口设备路径，如 /dev/ttyUSB0
  /// @param baud 波特率
  /// @param timeout_ms 读取超时时间（毫秒），0 表示阻塞
  SerialPort(const std::string& port, int baud, int timeout_ms = 100);

  ~SerialPort();

  bool Open();

  void Close();

  bool IsOpen() const;

  /// @brief 带超时的同步读取
  /// @param buf 接收缓冲区
  /// @param max_len 缓冲区最大长度
  /// @return 实际读取的字节数，超时或错误返回 0
  size_t Read(uint8_t* buf, size_t max_len);

private:
  boost::asio::io_service                   io_;
  std::unique_ptr<boost::asio::serial_port> serial_ptr_;
  std::string                               port_;
  int                                       baud_;
  int                                       timeout_ms_;
};
