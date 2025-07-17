#include "servo/servo_control.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace servo_control {

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
    // 尝试使用Feetech SDK初始化
    ft_adapter_ = std::make_unique<FeeTechAdapter>(port_, baud_rate_);
    
    if (ft_adapter_->init()) {
      connected_ = true;
      std::cout << "Connected to " << port_ << " at " << baud_rate_ << " baud using Feetech SDK" << std::endl;
      return true;
    } else {
      std::cerr << "Failed to initialize with Feetech SDK, falling back to traditional serial method" << std::endl;
      ft_adapter_.reset();
    }
    
    // 尝试使用传统串口方式初始化
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout_);
    serial_.setPort(port_);
    serial_.setBaudrate(baud_rate_);
    serial_.setTimeout(to);
    
    serial_.open();
    
    if (serial_.isOpen()) {
      connected_ = true;
      std::cout << "Connected to " << port_ << " at " << baud_rate_ << " baud using traditional method" << std::endl;
      return true;
    } else {
      std::cerr << "Failed to connect to " << port_ << std::endl;
      connected_ = false;
      return false;
    }
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    connected_ = false;
    return false;
  }
}

void ServoControl::close()
{
    if (ft_adapter_) {
      ft_adapter_->close();
    }
    
    if (serial_.isOpen()) {
    serial_.close();
    }
    
    connected_ = false;
  std::cout << "Serial port closed" << std::endl;
}

bool ServoControl::loadServoConfigs(const std::string& config_file)
{
  try {
    // 打开配置文件
    std::ifstream file(config_file);
    if (!file.is_open()) {
      std::cerr << "Failed to open config file: " << config_file << std::endl;
      return false;
    }
    
    // 解析JSON
    json config_json;
    file >> config_json;
    
    if (!config_json.is_array()) {
      std::cerr << "Invalid config file format" << std::endl;
      return false;
    }
    
    // 清除旧配置
    servo_types_.clear();
    
    // 读取舵机配置
    std::vector<FTServoConfig> ft_configs;
    
    for (const auto& servo : config_json) {
      uint8_t id = servo["id"];
      std::string name = servo["name"];
      std::string type_str = servo["type"];
      
      FTServoType type;
      if (type_str == "STS_TYPE1") {
        type = FTServoType::STS_TYPE1;
      } else if (type_str == "STS_TYPE2") {
        type = FTServoType::STS_TYPE2;
      } else {
        std::cerr << "Unknown servo type: " << type_str << " for ID: " << static_cast<int>(id) << std::endl;
        continue;
      }
      
      // 保存类型映射
      servo_types_[id] = type;
      
      // 创建FT舵机配置
      FTServoConfig ft_config;
      ft_config.id = id;
      ft_config.name = name;
      ft_config.type = type;
      ft_config.min_position = servo.value("min_position", 0);
      ft_config.max_position = servo.value("max_position", 4095);
      
      ft_configs.push_back(ft_config);
      
      std::cout << "Loaded config for servo ID " << static_cast<int>(id) 
                << ", type " << type_str 
                << ", name " << name << std::endl;
    }
    
    // 如果使用FT适配器，加载配置
    if (ft_adapter_) {
      ft_adapter_->loadServoConfigs(ft_configs);
    }
    
    return true;
  } catch (const std::exception& e) {
    std::cerr << "Error loading servo configs: " << e.what() << std::endl;
    return false;
  }
}

bool ServoControl::setPosition(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return false;
  }
  
  // 如果使用FT适配器
  if (ft_adapter_) {
    return ft_adapter_->setPosition(id, position, time, speed);
  }
  
  // 传统方式实现
  // 这里需要根据具体舵机协议实现
  // 这是一个简化的示例
  std::vector<uint8_t> packet;
  packet.push_back(0xFF);  // 头
  packet.push_back(0xFF);  // 头
  packet.push_back(id);    // ID
  packet.push_back(0x07);  // 长度
  packet.push_back(0x03);  // 写命令
  packet.push_back(0x2A);  // 位置寄存器地址
  packet.push_back(position & 0xFF);        // 位置低字节
  packet.push_back((position >> 8) & 0xFF); // 位置高字节
  packet.push_back(time & 0xFF);            // 时间低字节
  packet.push_back((time >> 8) & 0xFF);     // 时间高字节
  
  // 计算校验和
  uint8_t checksum = calculateChecksum(packet);
  packet.push_back(checksum);
  
  return sendPacket(packet);
}

bool ServoControl::setSpeed(uint8_t id, int16_t speed)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return false;
  }
  
  // 如果使用FT适配器
  if (ft_adapter_) {
    return ft_adapter_->setSpeed(id, speed);
  }
  
  // 传统方式实现
  // 这里需要根据具体舵机协议实现
  // 这是一个简化的示例
  std::vector<uint8_t> packet;
  packet.push_back(0xFF);  // 头
  packet.push_back(0xFF);  // 头
  packet.push_back(id);    // ID
  packet.push_back(0x05);  // 长度
  packet.push_back(0x03);  // 写命令
  packet.push_back(0x20);  // 速度寄存器地址
  packet.push_back(speed & 0xFF);        // 速度低字节
  packet.push_back((speed >> 8) & 0xFF); // 速度高字节
  
  // 计算校验和
  uint8_t checksum = calculateChecksum(packet);
  packet.push_back(checksum);
  
  return sendPacket(packet);
}

int ServoControl::getPosition(uint8_t id)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return -1;
  }
  
  // 如果使用FT适配器
  if (ft_adapter_) {
    return ft_adapter_->getPosition(id);
  }
  
  // 传统方式实现
  // 这里需要根据具体舵机协议实现
  // 这是一个简化的示例
  std::vector<uint8_t> packet;
  packet.push_back(0xFF);  // 头
  packet.push_back(0xFF);  // 头
  packet.push_back(id);    // ID
  packet.push_back(0x04);  // 长度
  packet.push_back(0x02);  // 读命令
  packet.push_back(0x38);  // 位置寄存器地址
  packet.push_back(0x02);  // 读取长度(2字节)
  
  // 计算校验和
  uint8_t checksum = calculateChecksum(packet);
  packet.push_back(checksum);
  
  if (!sendPacket(packet)) {
    return -1;
  }
  
  // 读取响应
  std::vector<uint8_t> response = readPacket();
  if (response.size() < 8) {
    return -1;
  }
  
  // 解析位置
  return (response[6] | (response[7] << 8));
}

int ServoControl::getTemperature(uint8_t id)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return -1;
  }
  
  // 如果使用FT适配器
  if (ft_adapter_) {
    return ft_adapter_->getTemperature(id);
  }
  
  // 传统方式实现
  // 这里需要根据具体舵机协议实现
  // 这是一个简化的示例
  std::vector<uint8_t> packet;
  packet.push_back(0xFF);  // 头
  packet.push_back(0xFF);  // 头
  packet.push_back(id);    // ID
  packet.push_back(0x04);  // 长度
  packet.push_back(0x02);  // 读命令
  packet.push_back(0x2B);  // 温度寄存器地址
  packet.push_back(0x01);  // 读取长度(1字节)
  
  // 计算校验和
  uint8_t checksum = calculateChecksum(packet);
  packet.push_back(checksum);
  
  if (!sendPacket(packet)) {
    return -1;
  }
  
  // 读取响应
  std::vector<uint8_t> response = readPacket();
  if (response.size() < 7) {
    return -1;
  }
  
  // 解析温度
  return response[6];
}

float ServoControl::getVoltage(uint8_t id)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return -1.0f;
  }
  
  // 如果使用FT适配器
  if (ft_adapter_) {
    return ft_adapter_->getVoltage(id);
  }
  
  // 传统方式实现
  // 这里需要根据具体舵机协议实现
  // 这是一个简化的示例
  std::vector<uint8_t> packet;
  packet.push_back(0xFF);  // 头
  packet.push_back(0xFF);  // 头
  packet.push_back(id);    // ID
  packet.push_back(0x04);  // 长度
  packet.push_back(0x02);  // 读命令
  packet.push_back(0x2A);  // 电压寄存器地址
  packet.push_back(0x01);  // 读取长度(1字节)
  
  // 计算校验和
  uint8_t checksum = calculateChecksum(packet);
  packet.push_back(checksum);
  
  if (!sendPacket(packet)) {
    return -1.0f;
  }
  
  // 读取响应
  std::vector<uint8_t> response = readPacket();
  if (response.size() < 7) {
    return -1.0f;
  }
  
  // 解析电压 (假设单位为0.1V)
  return response[6] / 10.0f;
}

bool ServoControl::setTorque(uint8_t id, bool enable)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return false;
  }
  
  // 如果使用FT适配器
  if (ft_adapter_) {
    return ft_adapter_->setTorque(id, enable);
  }
  
  // 传统方式实现
  // 这里需要根据具体舵机协议实现
  // 这是一个简化的示例
  std::vector<uint8_t> packet;
  packet.push_back(0xFF);  // 头
  packet.push_back(0xFF);  // 头
  packet.push_back(id);    // ID
  packet.push_back(0x04);  // 长度
  packet.push_back(0x03);  // 写命令
  packet.push_back(0x18);  // 扭矩使能寄存器地址
  packet.push_back(enable ? 0x01 : 0x00);  // 使能/失能
  
  // 计算校验和
  uint8_t checksum = calculateChecksum(packet);
  packet.push_back(checksum);
  
  return sendPacket(packet);
}

bool ServoControl::isConnected() const
{
  return connected_;
}

std::map<uint8_t, int> ServoControl::syncReadPositions(const std::vector<uint8_t>& ids)
{
  std::map<uint8_t, int> positions;
  
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return positions;
  }
  
  // 如果使用FT适配器
  if (ft_adapter_) {
    return ft_adapter_->syncReadPositions(ids);
  }
  
  // 传统方式实现 - 简单地循环读取每个舵机
  for (auto id : ids) {
    int pos = getPosition(id);
    if (pos >= 0) {
      positions[id] = pos;
    }
  }
  
  return positions;
}

bool ServoControl::syncWritePositions(const std::map<uint8_t, uint16_t>& positions)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return false;
  }
  
  // 如果使用FT适配器
  if (ft_adapter_) {
    return ft_adapter_->syncWritePositions(positions);
  }
  
  // 传统方式实现 - 简单地循环设置每个舵机
  for (const auto& pos : positions) {
    if (!setPosition(pos.first, pos.second)) {
      return false;
    }
  }
  
  return true;
}

bool ServoControl::sendPacket(const std::vector<uint8_t>& packet)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return false;
  }
  
  try {
    size_t bytes_written = serial_.write(packet);
    if (bytes_written != packet.size()) {
      std::cerr << "Failed to write all bytes: wrote " << bytes_written 
                << " of " << packet.size() << std::endl;
      return false;
    }
    return true;
  } catch (const serial::SerialException& e) {
    std::cerr << "Serial exception: " << e.what() << std::endl;
    return false;
  } catch (const serial::IOException& e) {
    std::cerr << "IO exception: " << e.what() << std::endl;
    return false;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return false;
  }
}

std::vector<uint8_t> ServoControl::readPacket()
{
  std::vector<uint8_t> response;
  
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return response;
  }
  
  try {
    // 等待数据
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    // 检查可用数据
    size_t bytes_available = serial_.available();
    if (bytes_available == 0) {
      std::cerr << "No data available to read" << std::endl;
      return response;
    }
    
    // 读取数据
    std::string data = serial_.read(bytes_available);
    for (char c : data) {
      response.push_back(static_cast<uint8_t>(c));
    }
    
    return response;
  } catch (const serial::SerialException& e) {
    std::cerr << "Serial exception: " << e.what() << std::endl;
    return response;
  } catch (const serial::IOException& e) {
    std::cerr << "IO exception: " << e.what() << std::endl;
    return response;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
  return response;
  }
}

uint8_t ServoControl::calculateChecksum(const std::vector<uint8_t>& data)
{
  uint8_t checksum = 0;
  
  // 从ID开始计算校验和
  for (size_t i = 2; i < data.size(); i++) {
    checksum += data[i];
  }
  
  // 取反
  return ~checksum;
}

} // namespace servo_control 