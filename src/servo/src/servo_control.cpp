#include "servo/servo_control.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace servo_control {

// SCServo舵机协议常量 (用于传统方式)
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
  // 创建Feetech适配器
  ft_adapter_ = std::make_unique<FeeTechAdapter>(port, baud_rate);
}

ServoControl::~ServoControl()
{
  close();
}

bool ServoControl::init()
{
  try {
    // 初始化Feetech适配器
    if (ft_adapter_->init()) {
      connected_ = true;
      RCLCPP_INFO(rclcpp::get_logger("servo_control"), "Connected to %s at %d baud using Feetech SDK", 
                 port_.c_str(), baud_rate_);
      return true;
    }
    
    // 如果Feetech适配器初始化失败，尝试使用传统方式
    RCLCPP_WARN(rclcpp::get_logger("servo_control"), 
               "Failed to initialize with Feetech SDK, falling back to traditional method");
    
    // 配置串口
    serial_.setPort(port_);
    serial_.setBaudrate(baud_rate_);
    serial::Timeout timeout(serial::Timeout::simpleTimeout(timeout_));
    serial_.setTimeout(timeout);
    
    // 打开串口
    serial_.open();
    connected_ = serial_.isOpen();
    
    if (connected_) {
      RCLCPP_INFO(rclcpp::get_logger("servo_control"), "Connected to %s at %d baud using traditional method", 
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
    
    // 关闭Feetech适配器
    if (ft_adapter_) {
      ft_adapter_->close();
    }
    
    // 关闭传统串口
    if (serial_.isOpen()) {
      serial_.close();
    }
    
    connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("servo_control"), "Serial port closed");
  }
}

bool ServoControl::loadServoConfigs(const std::string& config_file)
{
  try {
    YAML::Node config = YAML::LoadFile(config_file);
    
    if (!config["servo_config"] || !config["servo_config"]["wrist_servos"]) {
      RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Invalid config file format");
      return false;
    }
    
    std::vector<FTServoConfig> servo_configs;
    
    // 解析舵机配置
    auto servos = config["servo_config"]["wrist_servos"];
    for (size_t i = 0; i < servos.size(); i++) {
      auto servo = servos[i];
      
      FTServoConfig config;
      config.id = servo["id"].as<uint8_t>();
      config.name = servo["name"].as<std::string>();
      
      // 解析舵机类型
      std::string type_str = servo["type"].as<std::string>();
      if (type_str == "STS_TYPE1") {
        config.type = FTServoType::STS_TYPE1;
        servo_types_[config.id] = FTServoType::STS_TYPE1;
      } else if (type_str == "STS_TYPE2") {
        config.type = FTServoType::STS_TYPE2;
        servo_types_[config.id] = FTServoType::STS_TYPE2;
      } else {
        RCLCPP_WARN(rclcpp::get_logger("servo_control"), 
                   "Unknown servo type: %s, defaulting to STS_TYPE1", type_str.c_str());
        config.type = FTServoType::STS_TYPE1;
        servo_types_[config.id] = FTServoType::STS_TYPE1;
      }
      
      config.min_position = servo["min_pos"].as<uint16_t>();
      config.max_position = servo["max_pos"].as<uint16_t>();
      
      servo_configs.push_back(config);
      
      RCLCPP_INFO(rclcpp::get_logger("servo_control"), 
                 "Loaded config for servo ID %d, name %s, type %s", 
                 config.id, config.name.c_str(), type_str.c_str());
    }
    
    // 将配置加载到Feetech适配器
    if (ft_adapter_) {
      ft_adapter_->loadServoConfigs(servo_configs);
    }
    
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), 
                "Failed to load servo configs: %s", e.what());
    return false;
  }
}

bool ServoControl::setPosition(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
  (void)speed; // 标记参数为有意未使用
  
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return false;
  }
  
  // 尝试使用Feetech适配器
  if (ft_adapter_ && ft_adapter_->isConnected()) {
    return ft_adapter_->setPosition(id, position, time, speed);
  }
  
  // 传统方式
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
  
  // 尝试使用Feetech适配器
  if (ft_adapter_ && ft_adapter_->isConnected()) {
    return ft_adapter_->setSpeed(id, speed);
  }
  
  // 传统方式
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
  
  // 尝试使用Feetech适配器
  if (ft_adapter_ && ft_adapter_->isConnected()) {
    return ft_adapter_->getPosition(id);
  }
  
  // 传统方式
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
  
  // 尝试使用Feetech适配器
  if (ft_adapter_ && ft_adapter_->isConnected()) {
    return ft_adapter_->getTemperature(id);
  }
  
  // 传统方式
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
  
  // 尝试使用Feetech适配器
  if (ft_adapter_ && ft_adapter_->isConnected()) {
    return ft_adapter_->getVoltage(id);
  }
  
  // 传统方式
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
  
  // 尝试使用Feetech适配器
  if (ft_adapter_ && ft_adapter_->isConnected()) {
    return ft_adapter_->setTorque(id, enable);
  }
  
  // 传统方式
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
  // 首先检查Feetech适配器
  if (ft_adapter_ && ft_adapter_->isConnected()) {
    return true;
  }
  
  // 然后检查传统串口
  return connected_ && serial_.isOpen();
}

std::map<uint8_t, int> ServoControl::syncReadPositions(const std::vector<uint8_t>& ids)
{
  std::map<uint8_t, int> positions;
  
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return positions;
  }
  
  // 尝试使用Feetech适配器
  if (ft_adapter_ && ft_adapter_->isConnected()) {
    return ft_adapter_->syncReadPositions(ids);
  }
  
  // 传统方式不支持同步读取，使用顺序读取
  for (auto id : ids) {
    int position = getPosition(id);
    if (position >= 0) {
      positions[id] = position;
    }
  }
  
  return positions;
}

bool ServoControl::syncWritePositions(const std::map<uint8_t, uint16_t>& positions)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return false;
  }
  
  // 尝试使用Feetech适配器
  if (ft_adapter_ && ft_adapter_->isConnected()) {
    return ft_adapter_->syncWritePositions(positions);
  }
  
  // 传统方式不支持同步写入，使用顺序写入
  bool all_success = true;
  for (const auto& pos : positions) {
    if (!setPosition(pos.first, pos.second)) {
      all_success = false;
    }
  }
  
  return all_success;
}

bool ServoControl::sendPacket(const std::vector<uint8_t>& packet)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return false;
  }
  
  try {
    // 发送数据包
    size_t bytes_written = serial_.write(packet);
    
    if (bytes_written != packet.size()) {
      RCLCPP_ERROR(rclcpp::get_logger("servo_control"), 
                  "Failed to write all bytes: %zu of %zu written", 
                  bytes_written, packet.size());
      return false;
    }
    
    return true;
  }
  catch (const serial::IOException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), 
                "IOException: %s", e.what());
    return false;
  }
  catch (const serial::SerialException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), 
                "SerialException: %s", e.what());
    return false;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), 
                "Exception: %s", e.what());
    return false;
  }
}

std::vector<uint8_t> ServoControl::readPacket()
{
  std::vector<uint8_t> response;
  
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), "Not connected to servo controller");
    return response;
  }
  
  try {
    // 等待响应
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    
    // 检查是否有数据可读
    size_t bytes_available = serial_.available();
    if (bytes_available == 0) {
      RCLCPP_WARN(rclcpp::get_logger("servo_control"), "No data available to read");
      return response;
    }
    
    // 读取所有可用数据
    response = serial_.read(bytes_available);
    
    return response;
  }
  catch (const serial::IOException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), 
                "IOException: %s", e.what());
    return response;
  }
  catch (const serial::SerialException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), 
                "SerialException: %s", e.what());
    return response;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("servo_control"), 
                "Exception: %s", e.what());
    return response;
  }
}

uint8_t ServoControl::calculateChecksum(const std::vector<uint8_t>& data)
{
  uint8_t checksum = 0;
  
  // 校验和计算: 从ID开始，到校验和前一个字节
  for (size_t i = 2; i < data.size() - 1; ++i) {
    checksum += data[i];
  }
  
  // 取反
  checksum = ~checksum;
  
  return checksum;
}

} // namespace servo_control 