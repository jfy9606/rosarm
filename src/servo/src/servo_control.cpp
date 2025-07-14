#include "servo/servo_control.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace servo_control {

// SCServo舵机协议常量
namespace sc_protocol {
  constexpr uint8_t HEADER = 0xFF;           // 包头
  constexpr uint8_t CMD_PING = 0x01;         // PING命令
  constexpr uint8_t CMD_READ = 0x02;         // 读命令
  constexpr uint8_t CMD_WRITE = 0x03;        // 写命令
  constexpr uint8_t REG_POSITION = 0x2A;     // 位置寄存器地址
  constexpr uint8_t REG_SPEED = 0x30;        // 速度寄存器地址
  constexpr uint8_t REG_LOAD = 0x34;         // 负载寄存器地址
  constexpr uint8_t REG_VOLTAGE = 0x3E;      // 电压寄存器地址
  constexpr uint8_t REG_TEMPERATURE = 0x3F;  // 温度寄存器地址
  constexpr uint8_t REG_TORQUE_ENABLE = 0x28; // 扭矩使能寄存器地址
}

ServoControl::ServoControl(const std::string& port, int baud_rate, int timeout)
  : port_(port), baud_rate_(baud_rate), timeout_(timeout), connected_(false)
{
}

ServoControl::~ServoControl()
{
  close();
}

bool ServoControl::init()
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
      RCLCPP_INFO(rclcpp::get_logger("servo_control"), "Connected to %s at %d baud", 
                 port_.c_str(), baud_rate_);
      
      // 清空缓冲区
      serial_.flush();
      
      return true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Failed to connect to %s", 
                  port_.c_str());
      return false;
    }
  }
  catch (const serial::IOException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Exception: %s", e.what());
    connected_ = false;
    return false;
  }
}

void ServoControl::close()
{
  if (connected_) {
    // 尝试关闭所有舵机扭矩
    try {
      for (uint8_t id = 1; id <= 10; ++id) {
        setTorque(id, false);
      }
    } catch(...) {
      // 忽略异常，确保关闭串口
    }
    
    serial_.close();
    connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("servo_control"), "Serial port closed");
  }
}

bool ServoControl::setPosition(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return false;
  }
  
  // 构建位置控制数据包
  std::vector<uint8_t> packet;
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(id);
  packet.push_back(7);  // 长度: 命令(1) + 寄存器地址(1) + 位置(2) + 时间(2) + 校验和(1)
  packet.push_back(sc_protocol::CMD_WRITE);
  packet.push_back(sc_protocol::REG_POSITION);
  packet.push_back(position & 0xFF);
  packet.push_back((position >> 8) & 0xFF);
  packet.push_back(time & 0xFF);
  packet.push_back((time >> 8) & 0xFF);
  packet.push_back(calculateChecksum(packet));
  
  return sendPacket(packet);
}

bool ServoControl::setSpeed(uint8_t id, int16_t speed)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return false;
  }
  
  // 构建速度控制数据包
  std::vector<uint8_t> packet;
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(id);
  packet.push_back(5);  // 长度: 命令(1) + 寄存器地址(1) + 速度(2) + 校验和(1)
  packet.push_back(sc_protocol::CMD_WRITE);
  packet.push_back(sc_protocol::REG_SPEED);
  packet.push_back(speed & 0xFF);
  packet.push_back((speed >> 8) & 0xFF);
  packet.push_back(calculateChecksum(packet));
  
  return sendPacket(packet);
}

int ServoControl::getPosition(uint8_t id)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return -1;
  }
  
  // 构建位置读取数据包
  std::vector<uint8_t> packet;
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(id);
  packet.push_back(4);  // 长度: 命令(1) + 寄存器地址(1) + 数据长度(1) + 校验和(1)
  packet.push_back(sc_protocol::CMD_READ);
  packet.push_back(sc_protocol::REG_POSITION);
  packet.push_back(2);  // 读取2个字节
  packet.push_back(calculateChecksum(packet));
  
  if (!sendPacket(packet)) {
    return -1;
  }
  
  // 等待响应并解析
  std::vector<uint8_t> response = readPacket();
  if (response.size() >= 8) {
    return response[5] + (response[6] << 8);
  }
  
  return -1;
}

int ServoControl::getTemperature(uint8_t id)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return -1;
  }
  
  // 构建温度读取数据包
  std::vector<uint8_t> packet;
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(id);
  packet.push_back(4);  // 长度: 命令(1) + 寄存器地址(1) + 数据长度(1) + 校验和(1)
  packet.push_back(sc_protocol::CMD_READ);
  packet.push_back(sc_protocol::REG_TEMPERATURE);
  packet.push_back(1);  // 读取1个字节
  packet.push_back(calculateChecksum(packet));
  
  if (!sendPacket(packet)) {
    return -1;
  }
  
  // 等待响应并解析
  std::vector<uint8_t> response = readPacket();
  if (response.size() >= 7) {
    return response[5];
  }
  
  return -1;
}

float ServoControl::getVoltage(uint8_t id)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return -1.0f;
  }
  
  // 构建电压读取数据包
  std::vector<uint8_t> packet;
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(id);
  packet.push_back(4);  // 长度: 命令(1) + 寄存器地址(1) + 数据长度(1) + 校验和(1)
  packet.push_back(sc_protocol::CMD_READ);
  packet.push_back(sc_protocol::REG_VOLTAGE);
  packet.push_back(1);  // 读取1个字节
  packet.push_back(calculateChecksum(packet));
  
  if (!sendPacket(packet)) {
    return -1.0f;
  }
  
  // 等待响应并解析
  std::vector<uint8_t> response = readPacket();
  if (response.size() >= 7) {
    return response[5] / 10.0f;  // 电压单位为0.1V
  }
  
  return -1.0f;
}

bool ServoControl::setTorque(uint8_t id, bool enable)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return false;
  }
  
  // 构建扭矩使能数据包
  std::vector<uint8_t> packet;
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(sc_protocol::HEADER);
  packet.push_back(id);
  packet.push_back(4);  // 长度: 命令(1) + 寄存器地址(1) + 使能值(1) + 校验和(1)
  packet.push_back(sc_protocol::CMD_WRITE);
  packet.push_back(sc_protocol::REG_TORQUE_ENABLE);
  packet.push_back(enable ? 1 : 0);
  packet.push_back(calculateChecksum(packet));
  
  return sendPacket(packet);
}

bool ServoControl::isConnected() const
{
  return connected_ && serial_.isOpen();
}

bool ServoControl::sendPacket(const std::vector<uint8_t>& packet)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return false;
  }
  
  // 清空接收缓冲区
  serial_.flushInput();
  
  try {
    // 发送数据包
    serial_.write(packet);
    
    // 等待数据发送完成
    serial_.flush();
    return true;
  }
  catch (const serial::IOException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Error writing to serial port: %s", e.what());
    return false;
  }
}

std::vector<uint8_t> ServoControl::readPacket()
{
  std::vector<uint8_t> response;
  
  try {
    // 等待响应的合理时间
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_));
    
    // 检查是否有数据可读
    if (serial_.available()) {
      // 读取可用的所有字节
      response = serial_.read(serial_.available());
    } else {
      RCLCPP_WARN(rclcpp::get_logger("servo_control"), "No response received");
    }
  }
  catch (const serial::IOException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Error reading from serial port: %s", e.what());
  }
  
  return response;
}

uint8_t ServoControl::calculateChecksum(const std::vector<uint8_t>& data)
{
  uint8_t sum = 0;
  // 计算从ID开始，到校验和前一个字节的所有字节的和
  for (size_t i = 2; i < data.size(); i++) {
    sum += data[i];
  }
  return ~sum;  // 取反
}

} // namespace servo_control 