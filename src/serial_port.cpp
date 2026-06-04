#include "serial_port.h"
#include <boost/asio/deadline_timer.hpp>
#include <boost/bind.hpp>
#include <ros/ros.h>

SerialPort::SerialPort(const std::string& port, int baud, int timeout_ms)
    : port_(port), baud_(baud), timeout_ms_(timeout_ms), serial_ptr_(nullptr) {
}

SerialPort::~SerialPort() {
  Close();
}

bool SerialPort::Open() {
  try {
    serial_ptr_ = std::make_unique<boost::asio::serial_port>(io_, port_);
    serial_ptr_->set_option(boost::asio::serial_port_base::baud_rate(baud_));
    serial_ptr_->set_option(boost::asio::serial_port_base::character_size(8));
    serial_ptr_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial_ptr_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial_ptr_->set_option(
        boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    ROS_INFO("Serial port opened: %s @ %d bps", port_.c_str(), baud_);
    return true;
  } catch (const boost::system::system_error& e) {
    ROS_ERROR("Failed to open serial port %s: %s", port_.c_str(), e.what());
    serial_ptr_.reset();
    return false;
  }
}

void SerialPort::Close() {
  if (serial_ptr_ && serial_ptr_->is_open()) {
    boost::system::error_code ec;
    serial_ptr_->close(ec);
    if (ec) {
      ROS_WARN("Error closing serial port: %s", ec.message().c_str());
    } else {
      ROS_INFO("Serial port closed: %s", port_.c_str());
    }
  }
  serial_ptr_.reset();
}

bool SerialPort::IsOpen() const {
  return serial_ptr_ && serial_ptr_->is_open();
}

size_t SerialPort::Read(uint8_t* buf, size_t max_len) {
  if (!IsOpen())
    return 0;

  int effective_timeout = timeout_ms_;
  if (effective_timeout <= 0) {
    ROS_WARN_THROTTLE(60, "Serial timeout_ms <= 0. For stability, using 100 ms effective timeout.");
    effective_timeout = 100;
  }

  // 带超时的异步读取
  size_t                    bytes_read = 0;
  boost::system::error_code read_ec;

  boost::asio::deadline_timer timer(io_, boost::posix_time::milliseconds(effective_timeout));

  // 设置超时回调：取消串口读取
  timer.async_wait([this](const boost::system::error_code& ec_timer) {
    if (!ec_timer && serial_ptr_) {
      serial_ptr_->cancel();
    }
  });

  // 启动异步读取
  serial_ptr_->async_read_some(boost::asio::buffer(buf, max_len),
                               [&bytes_read, &read_ec](const boost::system::error_code& ec, size_t n) {
                                 read_ec = ec;
                                 if (!ec) {
                                   bytes_read = n;
                                 }
                               });

  // 运行 I/O 服务直到其中一个操作完成
  io_.restart();
  io_.run();

  // 取消可能仍在等待的定时器
  timer.cancel();

  // 处理读取错误（operation_aborted 是超时导致的，不算错误）
  if (read_ec && read_ec != boost::asio::error::operation_aborted) {
    ROS_ERROR("Serial read error: %s", read_ec.message().c_str());
    return 0;
  }

  return bytes_read;
}
