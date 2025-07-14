#include "motor/motor_control.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>

namespace motor_control {

MotorControl::MotorControl(const std::string& port, int baud_rate, int timeout)
  : port_(port), baud_rate_(baud_rate), timeout_(timeout), connected_(false)
{
}

MotorControl::~MotorControl()
{
  close();
}

bool MotorControl::init()
{
  try {
    // 配置串口
    serial_.setPort(port_);
    serial_.setBaudrate(baud_rate_);
    serial::Timeout timeout(serial::Timeout::simpleTimeout(timeout_));
    serial_.setTimeout(timeout);
    
    // 打开串口
    serial_.open();
    connected_ = serial_.isOpen();
    
    if (connected_) {
      RCLCPP_INFO(rclcpp::get_logger("motor_control"), "Connected to %s at %d baud", 
                 port_.c_str(), baud_rate_);
      
      // 清空缓冲区
      serial_.flush();
      
      return true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Failed to connect to %s", 
                  port_.c_str());
      return false;
    }
  }
  catch (const serial::IOException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Exception: %s", e.what());
    connected_ = false;
    return false;
  }
}

void MotorControl::close()
{
  if (connected_) {
    // 尝试停止所有电机
    try {
      // 发送停止命令
      for (int i = 1; i <= 10; ++i) {  // 假设最多10个电机站
        setVelocity(i, 0);
      }
    } catch(...) {
      // 忽略异常，确保关闭串口
    }
    
    serial_.close();
    connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("motor_control"), "Serial port closed");
  }
}

bool MotorControl::setPosition(uint8_t station_num, int32_t position, uint16_t threshold)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Not connected to motor controller");
    return false;
  }
  
  // 构造位置控制命令
  std::stringstream cmd;
  cmd << "#" << static_cast<int>(station_num) 
      << "P" << position 
      << "T" << threshold 
      << "\r\n";
  
  std::string response = sendCommand(cmd.str());
  
  // 检查响应是否成功
  return response.find("OK") != std::string::npos;
}

bool MotorControl::setVelocity(uint8_t station_num, int16_t velocity, uint16_t acc, uint16_t dec)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Not connected to motor controller");
    return false;
  }
  
  // 构造速度控制命令
  std::stringstream cmd;
  cmd << "#" << static_cast<int>(station_num) 
      << "V" << velocity
      << "A" << acc
      << "D" << dec
      << "\r\n";
  
  std::string response = sendCommand(cmd.str());
  
  // 检查响应是否成功
  return response.find("OK") != std::string::npos;
}

bool MotorControl::executeOrder(const motor::msg::MotorOrder::SharedPtr order)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Not connected to motor controller");
    return false;
  }

  bool success = true;
  
  // 依次执行每个电机的命令
  for (size_t i = 0; i < order->station_num.size(); ++i) {
    // 检查数组长度匹配
    if (i >= order->form.size() || i >= order->vel.size() || 
        i >= order->vel_ac.size() || i >= order->vel_de.size() ||
        i >= order->pos_mode.size() || i >= order->pos.size() || 
        i >= order->pos_thr.size()) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Motor order arrays size mismatch");
      return false;
    }
    
    // 根据控制模式执行不同的命令
    if (order->pos_mode[i]) {
      // 位置控制模式
      if (!setPosition(order->station_num[i], order->pos[i], order->pos_thr[i])) {
        success = false;
      }
    } else {
      // 速度控制模式
      if (!setVelocity(order->station_num[i], order->vel[i], 
                      order->vel_ac[i], order->vel_de[i])) {
        success = false;
      }
    }
    
    // 短暂延时，避免命令发送太快
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  return success;
}

std::string MotorControl::getStatus(uint8_t station_num)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Not connected to motor controller");
    return "";
  }
  
  // 构造状态查询命令
  std::stringstream cmd;
  cmd << "#" << static_cast<int>(station_num) << "S\r\n";
  
  return sendCommand(cmd.str());
}

bool MotorControl::isConnected() const
{
  return connected_ && serial_.isOpen();
}

std::string MotorControl::sendCommand(const std::string& command, bool wait_response)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Not connected to motor controller");
    return "";
  }
  
  // 清空接收缓冲区
  serial_.flushInput();
  
  // 发送命令
  try {
    RCLCPP_DEBUG(rclcpp::get_logger("motor_control"), "Sending: %s", command.c_str());
    serial_.write(command);
  }
  catch (const serial::IOException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Error writing to serial port: %s", e.what());
    return "";
  }
  
  // 如果不需要等待响应，直接返回
  if (!wait_response) {
    return "";
  }
  
  // 等待并读取响应
  std::string response = "";
  try {
    // 等待合理的时间让设备响应
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    if (serial_.available()) {
      response = serial_.readline();
      RCLCPP_DEBUG(rclcpp::get_logger("motor_control"), "Received: %s", response.c_str());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("motor_control"), "No response received");
    }
  }
  catch (const serial::IOException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Error reading from serial port: %s", e.what());
  }
  
  return response;
}

bool MotorControl::validateCommand(const std::string& command)
{
  // 检查命令格式是否正确
  // 此处可以根据具体的电机协议添加更详细的校验
  if (command.empty() || command[0] != '#') {
    return false;
  }
  
  // 检查是否以回车换行符结束
  if (command.size() < 2 || command.substr(command.size() - 2) != "\r\n") {
    return false;
  }
  
  return true;
}

} // namespace motor_control 