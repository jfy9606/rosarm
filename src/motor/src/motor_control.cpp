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
  // 初始化命令列表
  initCommandList();
}

MotorControl::~MotorControl()
{
  close();
}

void MotorControl::initCommandList()
{
  // 初始化code_list大小
  code_list.resize(23);
  
  /*
  code list:
  0->DI1=enable/DI1端口关联使能;
  1->enable on/使能开;
  2->enable off/使能关;
  3->control model:vel/控制模式：速度控制;
  4->control model:pos/控制模式：位移控制;
  5->vel order from:internal vel/速度指令来源：内部速度指令;
  6->DI2=pos enable/DI2端口关联位移运行使能;
  7->pos enable on/位移运行使能开;
  8->pos enable off/位移运行使能关;
  9->DO1=pos arrived/DO1端口关联定位到达;
  10->DO1 positive/定位到达正逻辑;
  11->pso order from:internal pos/位置指令来源：内部位移指令;
  12->pos from:once/多段位置运行方式：单次运行;
  13->pos num:1/位移指令段数:1段;
  14->absolute pos/绝对位移模式;
  15->relative pos/相对位移模式;
  16->read vel/读取转速;
  17->read pos/读取位置;
  18->read input/读取输入端口情况;
  19->read output/读取输出端口情况;
  20->read current/读取相电流;
  21->read voltage/读取母线电压;
  22->read temperature/读取模块温度;
  */
  
  code_list[0] = {0x06, 0x03, 0x02, 0x00, 0x01};
  code_list[1] = {0x06, 0x03, 0x03, 0x00, 0x01};
  code_list[2] = {0x06, 0x03, 0x03, 0x00, 0x00};
  code_list[3] = {0x06, 0x02, 0x00, 0x00, 0x00};
  code_list[4] = {0x06, 0x02, 0x00, 0x00, 0x01};
  code_list[5] = {0x06, 0x06, 0x02, 0x00, 0x00};
  code_list[6] = {0x06, 0x03, 0x04, 0x00, 0x1c};
  code_list[7] = {0x06, 0x03, 0x05, 0x00, 0x01};
  code_list[8] = {0x06, 0x03, 0x05, 0x00, 0x00};
  code_list[9] = {0x06, 0x04, 0x00, 0x00, 0x05};
  code_list[10] = {0x06, 0x04, 0x01, 0x00, 0x00};
  code_list[11] = {0x06, 0x05, 0x00, 0x00, 0x02};
  code_list[12] = {0x06, 0x11, 0x00, 0x00, 0x00};
  code_list[13] = {0x06, 0x11, 0x01, 0x00, 0x01};
  code_list[14] = {0x06, 0x11, 0x04, 0x00, 0x01};
  code_list[15] = {0x06, 0x11, 0x04, 0x00, 0x00};
  code_list[16] = {0x03, 0x0b, 0x00, 0x00, 0x01};
  code_list[17] = {0x03, 0x0b, 0x07, 0x00, 0x02};
  code_list[18] = {0x03, 0x0b, 0x03, 0x00, 0x01};
  code_list[19] = {0x03, 0x0b, 0x05, 0x00, 0x01};
  code_list[20] = {0x03, 0x0b, 0x18, 0x00, 0x01};
  code_list[21] = {0x03, 0x0b, 0x1a, 0x00, 0x01};
  code_list[22] = {0x03, 0x0b, 0x1b, 0x00, 0x01};
  
  // 初始化motor_list大小
  motor_list.resize(2);
  
  /*
  motor_list:
  0-> 电机停止命令；
  1-> 电机关闭命令；
  */
  motor_list[0] = {0x3E, 0x01, 0x08, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  motor_list[1] = {0x3E, 0x01, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
}

bool MotorControl::init()
{
  try {
    auto try_open = [&](const std::string &p) -> bool {
      try {
        serial_.setPort(p);
        serial_.setBaudrate(baud_rate_);
        serial::Timeout timeout(serial::Timeout::simpleTimeout(timeout_));
        serial_.setTimeout(timeout);
        serial_.open();
        if (serial_.isOpen()) {
          connected_ = true;
          RCLCPP_INFO(rclcpp::get_logger("motor_control"), "Connected to %s at %d baud", p.c_str(), baud_rate_);
          return true;
        }
      } catch (const serial::IOException &e) {
        (void)e; // ignore and try next
      }
      return false;
    };

    bool ok = false;

    // 1) 优先尝试指定端口（且不为"auto"）
    if (!port_.empty() && port_ != "auto") {
      ok = try_open(port_);
    }

    // 2) 当未成功或端口为"auto"时，自动扫描常见端口
    if (!ok) {
      std::vector<std::string> candidates;
      for (int i = 0; i < 10; ++i) {
        candidates.push_back(std::string("/dev/ttyUSB") + std::to_string(i));
      }
      for (int i = 0; i < 10; ++i) {
        candidates.push_back(std::string("/dev/ttyACM") + std::to_string(i));
      }

      for (const auto &p : candidates) {
        if (try_open(p)) {
          port_ = p; // 记录成功端口
          ok = true;
          break;
        }
      }
    }

    if (ok) {
      serial_.flush();
      return true;
    } else {
      connected_ = false;
      RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Failed to auto-detect a valid serial port for motor controller. Last attempted: %s", port_.c_str());
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
      } else if (type_str == "YSF4_MOTOR" || type_str == "YSF4") {
        config.type = MotorType::YSF4_MOTOR;
      } else if (type_str == "YSF4_PRO_MOTOR" || type_str == "YSF4Pro" || type_str == "YSF4_PRO") {
        config.type = MotorType::YSF4_PRO_MOTOR;
      } else if (type_str == "YF_MOTOR") {
        // 兼容历史配置，将YF_MOTOR视作YSF4_MOTOR，并保留方向反转特性
        RCLCPP_WARN(rclcpp::get_logger("motor_control"), 
                   "YF_MOTOR is deprecated, automatically converting to YSF4_MOTOR");
        config.type = MotorType::YSF4_MOTOR;
      } else if (type_str == "YSF4_HAL_MOTOR") {
        config.type = MotorType::YSF4_HAL_MOTOR;
      } else {
        RCLCPP_WARN(rclcpp::get_logger("motor_control"), 
                   "Unknown motor type: %s, defaulting to AI_MOTOR", type_str.c_str());
        config.type = MotorType::AI_MOTOR;
      }
      
      // 解析协议类型
      std::string protocol_str = motor["protocol"].as<std::string>("TEXT_COMMAND");
      if (protocol_str == "MODBUS_RTU") {
        config.protocol = ProtocolType::MODBUS_RTU;
      } else {
        config.protocol = ProtocolType::TEXT_COMMAND;
      }
      
      config.max_speed = motor["max_speed"].as<int>();
      config.max_acc = motor["max_acc"].as<int>();
      // 新增安全与电机本体参数，存在则读取
      config.max_current = motor["max_current"] ? motor["max_current"].as<int>() : 5000; // mA
      config.rated_voltage = motor["rated_voltage"] ? motor["rated_voltage"].as<int>() : 24000; // mV
      config.pole_pairs = motor["pole_pairs"] ? motor["pole_pairs"].as<int>() : 7; // 14极=7极对
      config.encoder_resolution = motor["encoder_resolution"] ? motor["encoder_resolution"].as<int>() : 2048;
      config.hall_sensor = motor["hall_sensor"] ? motor["hall_sensor"].as<bool>() : true;
      config.temp_limit = motor["temp_limit"] ? motor["temp_limit"].as<int>() : 85;
      config.kv_rating = motor["kv_rating"] ? motor["kv_rating"].as<float>() : 100.0f;
      config.overcurrent_protection = motor["overcurrent_protection"] ? motor["overcurrent_protection"].as<int>() : config.max_current;
      config.thermal_protection = motor["thermal_protection"] ? motor["thermal_protection"].as<int>() : config.temp_limit;
      
      motor_configs_[config.id] = config;
      
      RCLCPP_INFO(rclcpp::get_logger("motor_control"), 
                 "Loaded config for motor ID %d, name %s, type %s, protocol %s", 
                 config.id, config.name.c_str(), type_str.c_str(), protocol_str.c_str());
    }
    
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), 
                "Failed to load motor configs: %s", e.what());
    return false;
  }
}


  // 电机安全检查，防止过载过流过热
  bool safetyClamped(uint8_t station_num, int16_t& velocity, uint16_t& acc) {
    auto it = motor_configs_.find(station_num);
    if (it != motor_configs_.end()) {
      auto& cfg = it->second;
      // YSF4系列电机有更严格的参数检查
    if (cfg.type == MotorType::YSF4_MOTOR || cfg.type == MotorType::YSF4_PRO_MOTOR || cfg.type == MotorType::YSF4_HAL_MOTOR) {
      if (cfg.type == MotorType::YSF4_PRO_MOTOR) {
        // YSF4Pro性能更好，可以使用更高的参数
        velocity = std::min(std::max(velocity, static_cast<int16_t>(-cfg.max_speed * 0.8)), 
                          static_cast<int16_t>(cfg.max_speed * 0.8)); // 使用80%最大速度
        acc = std::min(acc, static_cast<uint16_t>(cfg.max_acc * 0.8)); // 使用80%加速度
      } else {
        // YSF4和YSF4_HAL_MOTOR使用保守参数
        velocity = std::min(std::max(velocity, static_cast<int16_t>(-cfg.max_speed/2)), 
                          static_cast<int16_t>(cfg.max_speed/2)); // 降低50%最大速度避免过载
        acc = std::min(acc, static_cast<uint16_t>(cfg.max_acc/2)); // 降低50%加速度
      }
    } else {
      velocity = std::min(std::max(velocity, static_cast<int16_t>(-cfg.max_speed)), 
                        static_cast<int16_t>(cfg.max_speed));
      acc = std::min(acc, static_cast<uint16_t>(cfg.max_acc));
      }
    }
    return true;
  }
bool MotorControl::setPosition(uint8_t station_num, int32_t position, uint16_t threshold, 
                             uint16_t vel, uint16_t acc)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Not connected to motor controller");
    return false;
  }
  
  // 获取电机协议类型
  ProtocolType protocol = getProtocolType(station_num);
  
  if (protocol == ProtocolType::TEXT_COMMAND) {
    // 文本命令协议
    
    // 根据电机类型调整参数
    int16_t velocity = 0;
    uint16_t dec = acc;
    adjustParamsByType(station_num, position, velocity, acc, dec);
    safetyClamped(station_num, velocity, acc);
    
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
  else if (protocol == ProtocolType::MODBUS_RTU) {
    // ModBus RTU协议
    
    // 首先确保设置为位置控制模式
    if (!setControlMode(station_num, 0)) {
      return false;
    }
    
    // 构造位置控制指令（使用相对位置模式）
    std::vector<uint8_t> pos_data = {0x06, 0x12, 0x00};
    
    // 将position值添加到命令中（32位整数，高位在前）
    pos_data.push_back(static_cast<uint8_t>((position >> 24) & 0xFF));
    pos_data.push_back(static_cast<uint8_t>((position >> 16) & 0xFF));
    pos_data.push_back(static_cast<uint8_t>((position >> 8) & 0xFF));
    pos_data.push_back(static_cast<uint8_t>(position & 0xFF));
    
    // 添加速度限制
    pos_data.push_back(static_cast<uint8_t>((vel >> 8) & 0xFF));
    pos_data.push_back(static_cast<uint8_t>(vel & 0xFF));
    
    // 添加加速度
    pos_data.push_back(static_cast<uint8_t>((acc >> 8) & 0xFF));
    pos_data.push_back(static_cast<uint8_t>(acc & 0xFF));
    
    // 发送命令
    auto full_cmd = buildModBusCommand(station_num, pos_data);
    auto response = sendBinaryCommand(full_cmd);
    
    // 启动位置运行
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto enable_cmd = buildModBusCommand(station_num, code_list[7]);
    sendBinaryCommand(enable_cmd);
    
    return !response.empty();
  }
  
  return false;
}

bool MotorControl::setVelocity(uint8_t station_num, int16_t velocity, uint16_t acc, uint16_t dec)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Not connected to motor controller");
    return false;
  }
  
  // 获取电机协议类型
  ProtocolType protocol = getProtocolType(station_num);
  
  if (protocol == ProtocolType::TEXT_COMMAND) {
    // 文本命令协议
    
    // 根据电机类型调整参数
    int32_t position = 0;
    adjustParamsByType(station_num, position, velocity, acc, dec);
    safetyClamped(station_num, velocity, acc);
    
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
  else if (protocol == ProtocolType::MODBUS_RTU) {
    // ModBus RTU协议
    
    // 首先确保设置为速度控制模式
    if (!setControlMode(station_num, 1)) {
      return false;
    }
    
    // 构造速度控制指令
    std::vector<uint8_t> vel_data = {0x06, 0x06, 0x01};
    
    // 将velocity值添加到命令中（16位整数，高位在前）
    vel_data.push_back(static_cast<uint8_t>((velocity >> 8) & 0xFF));
    vel_data.push_back(static_cast<uint8_t>(velocity & 0xFF));
    
    // 添加加速度
    vel_data.push_back(static_cast<uint8_t>((acc >> 8) & 0xFF));
    vel_data.push_back(static_cast<uint8_t>(acc & 0xFF));
    
    // 添加减速度
    vel_data.push_back(static_cast<uint8_t>((dec >> 8) & 0xFF));
    vel_data.push_back(static_cast<uint8_t>(dec & 0xFF));
    
    // 发送命令
    auto full_cmd = buildModBusCommand(station_num, vel_data);
    auto response = sendBinaryCommand(full_cmd);
    
    return !response.empty();
  }
  
  return false;
}

bool MotorControl::enableMotor(uint8_t station_num, bool enable)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Not connected to motor controller");
    return false;
  }
  
  // 获取电机协议类型
  ProtocolType protocol = getProtocolType(station_num);
  
  if (protocol == ProtocolType::TEXT_COMMAND) {
    // 文本命令协议 - 使用M命令
    std::stringstream cmd;
    cmd << "#" << static_cast<int>(station_num)
        << "M" << (enable ? "1" : "0")
        << "\r\n";
    
    std::string response = sendCommand(cmd.str());
    return response.find("OK") != std::string::npos;
  }
  else if (protocol == ProtocolType::MODBUS_RTU) {
    // ModBus RTU协议 - 使用预定义命令
    auto cmd_idx = enable ? 1 : 2;  // 1=enable on, 2=enable off
    auto full_cmd = buildModBusCommand(station_num, code_list[cmd_idx]);
    auto response = sendBinaryCommand(full_cmd);
    return !response.empty();
  }
  
  return false;
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
    if (order->form[i] == 0) {  // 位置控制模式
      int32_t position = order->pos[i];
      uint16_t threshold = order->pos_thr[i];
      uint16_t vel = static_cast<uint16_t>(abs(order->vel[i])); // 使用绝对值作为位置模式下的速度限制
      
      if (!setPosition(station_num, position, threshold, vel, order->vel_ac[i])) {
        success = false;
      }
    } else {  // 速度控制模式
      int16_t velocity = order->vel[i];
      uint16_t acc = order->vel_ac[i];
      uint16_t dec = order->vel_de[i];
      
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
  
  // 获取电机协议类型
  ProtocolType protocol = getProtocolType(station_num);
  
  if (protocol == ProtocolType::TEXT_COMMAND) {
    // 文本命令协议
    std::stringstream cmd;
    cmd << "#" << static_cast<int>(station_num) << "S\r\n";
    return sendCommand(cmd.str());
  }
  else if (protocol == ProtocolType::MODBUS_RTU) {
    // ModBus RTU协议 - 读取位置和速度
    std::stringstream result;
    
    // 读取位置
    auto pos_cmd = buildModBusCommand(station_num, code_list[17]);
    auto pos_response = sendBinaryCommand(pos_cmd);
    
    // 读取速度
    auto vel_cmd = buildModBusCommand(station_num, code_list[16]);
    auto vel_response = sendBinaryCommand(vel_cmd);
    
    if (!pos_response.empty() && pos_response.size() >= 9) {
      int32_t pos = (pos_response[5] << 24) | (pos_response[6] << 16) | 
                    (pos_response[7] << 8) | pos_response[8];
      result << "POS:" << pos << " ";
    }
    
    if (!vel_response.empty() && vel_response.size() >= 7) {
      int16_t vel = (vel_response[5] << 8) | vel_response[6];
      result << "VEL:" << vel;
    }
    
    return result.str();
  }
  
  return "";
}

bool MotorControl::isConnected() const
{
  return connected_;
}

std::string MotorControl::sendCommand(const std::string& command, bool wait_response)
{
  if (!connected_) {
    return "";
  }
  
  try {
    // 清空接收缓冲区
    serial_.flushInput();
    
    // 发送命令
    size_t bytes_written = serial_.write(command);
    
    if (bytes_written != command.length()) {
      RCLCPP_WARN(rclcpp::get_logger("motor_control"), 
                 "Failed to write all bytes to serial port");
    }
    
    // 如果不需要等待响应，直接返回
    if (!wait_response) {
      return "";
    }
    
    // 等待响应
    std::string response;
    
    // 尝试读取响应，最多等待timeout_毫秒
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - start_time).count() < timeout_) {
      
      if (serial_.available()) {
        std::string chunk = serial_.read(serial_.available());
        response += chunk;
        
        // 如果收到完整响应，可以提前返回
        if (response.find("\r\n") != std::string::npos) {
          break;
        }
      }
      
      // 短暂休眠，避免CPU满载
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    return response;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), 
                "Failed to send command: %s", e.what());
    return "";
  }
}

std::vector<uint8_t> MotorControl::sendBinaryCommand(const std::vector<uint8_t>& command, bool wait_response)
{
  if (!connected_) {
    return {};
  }
  
  try {
    // 清空接收缓冲区
    serial_.flushInput();
    
    // 发送命令
    size_t bytes_written = serial_.write(command);
    
    if (bytes_written != command.size()) {
      RCLCPP_WARN(rclcpp::get_logger("motor_control"), 
                 "Failed to write all bytes to serial port");
    }
    
    // 如果不需要等待响应，直接返回
    if (!wait_response) {
      return {};
    }
    
    // 等待响应
    std::vector<uint8_t> response;
    
    // 尝试读取响应，最多等待timeout_毫秒
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now() - start_time).count() < timeout_) {
      
      if (serial_.available()) {
        std::vector<uint8_t> chunk;
        chunk.resize(serial_.available());
        serial_.read(chunk.data(), chunk.size());
        response.insert(response.end(), chunk.begin(), chunk.end());
        
        // ModBus响应通常至少有4个字节
        if (response.size() >= 4) {
          break;
        }
      }
      
      // 短暂休眠，避免CPU满载
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    return response;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), 
                "Failed to send binary command: %s", e.what());
    return {};
  }
}

bool MotorControl::validateCommand(const std::string& command)
{
  // 检查命令是否以#开头，以\r\n结尾
  if (command.empty() || command[0] != '#' || 
      command.size() < 4 || command.substr(command.size() - 2) != "\r\n") {
    return false;
  }
  
  return true;
}

bool MotorControl::setControlMode(uint8_t station_num, int8_t mode)
{
  // 检查当前模式，如果一致则不需要切换
  auto it = motor_mode_cache_.find(station_num);
  if (it != motor_mode_cache_.end() && it->second == mode) {
    return true;
  }
  
  int code_num = 0;
  std::vector<int> code_order;
  
  if (mode == 1) {  // 速度模式
    code_order = {3, 0, 2, 5, 6, 8};
    code_num = code_order.size();
  }
  else if (mode == 0) {  // 位置模式
    code_order = {4, 0, 2, 6, 8, 9, 10, 11, 12, 13};
    code_num = code_order.size();
  }
  else {
    RCLCPP_ERROR(rclcpp::get_logger("motor_control"), "Invalid control mode: %d", mode);
    return false;
  }
  
  bool success = true;
  for (int i = 0; i < code_num; i++) {
    auto cmd = buildModBusCommand(station_num, code_list[code_order[i]]);
    auto response = sendBinaryCommand(cmd);
    
    if (response.empty()) {
      success = false;
    }
    
    // 每条命令之间等待一小段时间
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  
  if (success) {
    // 更新模式缓存
    motor_mode_cache_[station_num] = mode;
  }
  
  return success;
}

void MotorControl::adjustParamsByType(uint8_t station_num, int32_t& position, int16_t& velocity, 
                                    uint16_t& acc, uint16_t& dec)
{
  auto it = motor_configs_.find(station_num);
  if (it == motor_configs_.end()) {
    return;
  }
  
  // 根据电机类型调整参数
  switch (it->second.type) {
    case MotorType::AI_MOTOR:
      // AImotor类型 - 参数无需特别调整
      break;
      
    case MotorType::YSF4_MOTOR:
      [[fallthrough]];
    case MotorType::YSF4_PRO_MOTOR:
      [[fallthrough]];
    case MotorType::YSF4_HAL_MOTOR:
      // YSF4系列电机 - 位置和速度值需要反向
      position = -position;
      velocity = -velocity;
      break;
  }
  
  // 限制最大速度和加速度
  velocity = std::min(std::max(velocity, static_cast<int16_t>(-it->second.max_speed)), 
                     static_cast<int16_t>(it->second.max_speed));
  acc = std::min(acc, static_cast<uint16_t>(it->second.max_acc));
  dec = std::min(dec, static_cast<uint16_t>(it->second.max_acc));
}

MotorType MotorControl::getMotorType(uint8_t station_num) const
{
  auto it = motor_configs_.find(station_num);
  if (it != motor_configs_.end()) {
    return it->second.type;
  }
  
  // 默认返回AImotor类型
  return MotorType::AI_MOTOR;
}

ProtocolType MotorControl::getProtocolType(uint8_t station_num) const
{
  auto it = motor_configs_.find(station_num);
  if (it != motor_configs_.end()) {
    return it->second.protocol;
  }
  
  // 默认返回文本协议
  return ProtocolType::TEXT_COMMAND;
}

std::vector<uint8_t> MotorControl::buildModBusCommand(uint8_t station_num, const std::vector<uint8_t>& data)
{
  return calculateCRC16(data, data.size(), station_num);
}

std::vector<uint8_t> MotorControl::calculateCRC16(const std::vector<uint8_t>& message, uint8_t length, uint8_t station_num)
{
  std::vector<uint8_t> modbusMsg;
  
  // 添加站地址
  modbusMsg.push_back(station_num);
  
  // 添加主体消息
  modbusMsg.insert(modbusMsg.end(), message.begin(), message.end());
  
  // 计算CRC16
  uint16_t crc = 0xFFFF;
  
  for (size_t i = 0; i < modbusMsg.size(); i++) {
    crc ^= static_cast<uint16_t>(modbusMsg[i]);
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  
  // 添加CRC（低位在前，高位在后）
  modbusMsg.push_back(static_cast<uint8_t>(crc & 0xFF));
  modbusMsg.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  
  return modbusMsg;
}

} // namespace motor_control