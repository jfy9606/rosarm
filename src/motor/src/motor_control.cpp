#include "motor/motor_control.hpp"
#include <motor/msg/motor_order.hpp>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <fstream>

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
      for (const auto& motor : motor_configs_) {
        setVelocity(motor.first, 0);
      }
    } catch(...) {
      // 忽略异常，确保关闭串口
    }
    
    serial_.close();
    connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("motor_control"), "Serial port closed");
  }
}

bool MotorControl::loadMotorConfigs(const std::string& config_file)
{
  try {
    YAML::Node config = YAML::LoadFile(config_file);
    
    if (!config["motor_config"] || !config["motor_config"]["arm_motors"]) {
      RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Invalid config file format");
      return false;
    }
    
    // 解析电机配置
    auto motors = config["motor_config"]["arm_motors"];
    for (size_t i = 0; i < motors.size(); i++) {
      auto motor = motors[i];
      
      MotorConfig config;
      config.id = motor["id"].as<uint8_t>();
      config.name = motor["name"].as<std::string>();
      
      // 解析电机类型
      std::string type_str = motor["type"].as<std::string>();
      if (type_str == "AI_MOTOR") {
        config.type = MotorType::AI_MOTOR;
      } else if (type_str == "YF_MOTOR") {
        config.type = MotorType::YF_MOTOR;
      } else {
        RCLCPP_WARN(rclcpp::get_logger("motor_control"), 
                   "Unknown motor type: %s, defaulting to AI_MOTOR", type_str.c_str());
        config.type = MotorType::AI_MOTOR;
      }
      
      config.max_speed = motor["max_speed"].as<int>();
      config.max_acc = motor["max_acc"].as<int>();
      
      motor_configs_[config.id] = config;
      
      RCLCPP_INFO(rclcpp::get_logger("motor_control"), 
                 "Loaded config for motor ID %d, name %s, type %s", 
                 config.id, config.name.c_str(), type_str.c_str());
    }
    
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), 
                "Failed to load motor configs: %s", e.what());
    return false;
  }
}

bool MotorControl::setPosition(uint8_t station_num, int32_t position, uint16_t threshold)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Not connected to motor controller");
    return false;
  }
  
  // 根据电机类型调整参数
  int16_t velocity = 0;
  uint16_t acc = 100;
  uint16_t dec = 100;
  adjustParamsByType(station_num, position, velocity, acc, dec);
  
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
  
  // 根据电机类型调整参数
  int32_t position = 0;
  adjustParamsByType(station_num, position, velocity, acc, dec);
  
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

bool MotorControl::executeOrder(const std::shared_ptr<motor::msg::MotorOrder> order)
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
    
    uint8_t station_num = order->station_num[i];
    
    // 根据控制模式执行不同的命令
    if (order->pos_mode[i]) {
      // 位置控制模式
      int32_t position = order->pos[i];
      uint16_t threshold = order->pos_thr[i];
      
      // 根据电机类型调整参数
      int16_t velocity = 0;
      uint16_t acc = 100;
      uint16_t dec = 100;
      adjustParamsByType(station_num, position, velocity, acc, dec);
      
      if (!setPosition(station_num, position, threshold)) {
        success = false;
      }
    } else {
      // 速度控制模式
      int16_t velocity = order->vel[i];
      uint16_t acc = order->vel_ac[i];
      uint16_t dec = order->vel_de[i];
      
      // 根据电机类型调整参数
      int32_t position = 0;
      adjustParamsByType(station_num, position, velocity, acc, dec);
      
      if (!setVelocity(station_num, velocity, acc, dec)) {
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
  // 简单验证命令格式
  if (command.empty() || command[0] != '#') {
    return false;
  }
  
  // 验证命令结束符
  if (command.length() < 3 || 
      command.substr(command.length() - 2) != "\r\n") {
    return false;
  }
  
  return true;
}

void MotorControl::adjustParamsByType(uint8_t station_num, int32_t& position, int16_t& velocity, 
                                     uint16_t& acc, uint16_t& dec)
{
  MotorType type = getMotorType(station_num);
  
  // 根据电机类型调整参数
  switch (type) {
    case MotorType::AI_MOTOR:
      // AI电机参数调整
      if (velocity != 0) {
        // 速度限制
        if (velocity > 5000) velocity = 5000;
        if (velocity < -5000) velocity = -5000;
        
        // 加减速限制
        if (acc > 500) acc = 500;
        if (dec > 500) dec = 500;
      }
      break;
      
    case MotorType::YF_MOTOR:
      // YF电机参数调整
      if (velocity != 0) {
        // 速度限制
        if (velocity > 3000) velocity = 3000;
        if (velocity < -3000) velocity = -3000;
        
        // 加减速限制
        if (acc > 300) acc = 300;
        if (dec > 300) dec = 300;
      }
      break;
  }
  
  RCLCPP_DEBUG(rclcpp::get_logger("motor_control"), 
              "Adjusted parameters for motor %d (type %d): pos=%d, vel=%d, acc=%d, dec=%d",
              station_num, static_cast<int>(type), position, velocity, acc, dec);
}

MotorType MotorControl::getMotorType(uint8_t station_num) const
{
  auto it = motor_configs_.find(station_num);
  if (it != motor_configs_.end()) {
    return it->second.type;
  }
  
  // 如果找不到配置，默认返回AI_MOTOR类型
  RCLCPP_WARN(rclcpp::get_logger("motor_control"), 
             "No configuration found for motor %d, using default type", station_num);
  return MotorType::AI_MOTOR;
}

} // namespace motor_control 