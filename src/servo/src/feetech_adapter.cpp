#include "servo/feetech_adapter.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace servo_control {

FeeTechAdapter::FeeTechAdapter(const std::string& port, int baud_rate)
  : port_(port), baud_rate_(baud_rate), connected_(false), protocol_end_(0)
{
}

FeeTechAdapter::~FeeTechAdapter()
{
  close();
}

bool FeeTechAdapter::init()
{
  try {
    // 创建端口处理器
    port_handler_ = std::shared_ptr<PortHandler>(new PortHandler(port_.c_str()));
    
    // 创建数据包处理器 (STS/SMS=0, SCS=1)
    packet_handler_ = std::shared_ptr<PacketHandler>(PacketHandler(protocol_end_));
    
    // 打开端口
    if (!port_handler_->openPort()) {
      RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Failed to open port %s", port_.c_str());
      return false;
    }
    
    // 设置波特率
    if (!port_handler_->setBaudRate(baud_rate_)) {
      RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Failed to set baudrate %d", baud_rate_);
      port_handler_->closePort();
      return false;
    }
    
    // 创建同步读取和写入处理器
    sync_read_ = std::make_shared<GroupSyncRead>(port_handler_.get(), packet_handler_.get(), 
                                                ft_address::PRESENT_POSITION, 4);
    sync_write_ = std::make_shared<GroupSyncWrite>(port_handler_.get(), packet_handler_.get(), 
                                                  ft_address::GOAL_POSITION, 2);
    
    connected_ = true;
    RCLCPP_INFO(rclcpp::get_logger("feetech_adapter"), 
               "Connected to %s at %d baud", port_.c_str(), baud_rate_);
    
    return true;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Exception: %s", e.what());
    connected_ = false;
    return false;
  }
}

void FeeTechAdapter::close()
{
  if (connected_) {
    // 尝试关闭所有舵机扭矩
    try {
      for (const auto& servo : servo_configs_) {
        setTorque(servo.first, false);
      }
    } catch(...) {
      // 忽略异常，确保关闭端口
    }
    
    port_handler_->closePort();
    connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("feetech_adapter"), "Port closed");
  }
}

void FeeTechAdapter::loadServoConfigs(const std::vector<FTServoConfig>& configs)
{
  servo_configs_.clear();
  for (const auto& config : configs) {
    servo_configs_[config.id] = config;
    RCLCPP_INFO(rclcpp::get_logger("feetech_adapter"), 
               "Loaded config for servo ID %d, type %d, name %s", 
               config.id, static_cast<int>(config.type), config.name.c_str());
  }
}

bool FeeTechAdapter::setPosition(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Not connected to servo controller");
    return false;
  }
  
  // 根据舵机类型调整参数
  adjustParamsByType(id, position, time, speed);
  
  // 限制位置在舵机范围内
  position = clampPosition(id, position);
  
  // 设置加速度
  uint8_t acc = 0; // 默认加速度
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  
  comm_result, error = packet_handler_->write1ByteTxRx(port_handler_.get(), id, ft_address::GOAL_ACC, acc);
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Failed to set acceleration: %s", 
                packet_handler_->getTxRxResult(comm_result));
    return false;
  }
  
  // 设置速度
  comm_result, error = packet_handler_->write2ByteTxRx(port_handler_.get(), id, ft_address::GOAL_SPEED, speed);
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Failed to set speed: %s", 
                packet_handler_->getTxRxResult(comm_result));
    return false;
  }
  
  // 设置位置
  comm_result, error = packet_handler_->write2ByteTxRx(port_handler_.get(), id, ft_address::GOAL_POSITION, position);
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Failed to set position: %s", 
                packet_handler_->getTxRxResult(comm_result));
    return false;
  }
  
  return true;
}

bool FeeTechAdapter::setSpeed(uint8_t id, int16_t speed)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Not connected to servo controller");
    return false;
  }
  
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  
  comm_result, error = packet_handler_->write2ByteTxRx(port_handler_.get(), id, ft_address::GOAL_SPEED, speed);
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Failed to set speed: %s", 
                packet_handler_->getTxRxResult(comm_result));
    return false;
  }
  
  return true;
}

int FeeTechAdapter::getPosition(uint8_t id)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Not connected to servo controller");
    return -1;
  }
  
  uint16_t position = 0;
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  
  position, comm_result, error = packet_handler_->read2ByteTxRx(port_handler_.get(), id, ft_address::PRESENT_POSITION);
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Failed to get position: %s", 
                packet_handler_->getTxRxResult(comm_result));
    return -1;
  }
  
  return position;
}

int FeeTechAdapter::getTemperature(uint8_t id)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Not connected to servo controller");
    return -1;
  }
  
  // 温度寄存器地址 (假设为0x3F，需要根据实际舵机型号调整)
  uint8_t temp_addr = 0x3F;
  uint8_t temperature = 0;
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  
  temperature, comm_result, error = packet_handler_->read1ByteTxRx(port_handler_.get(), id, temp_addr);
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Failed to get temperature: %s", 
                packet_handler_->getTxRxResult(comm_result));
    return -1;
  }
  
  return temperature;
}

float FeeTechAdapter::getVoltage(uint8_t id)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Not connected to servo controller");
    return -1.0f;
  }
  
  // 电压寄存器地址 (假设为0x3E，需要根据实际舵机型号调整)
  uint8_t volt_addr = 0x3E;
  uint8_t voltage = 0;
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  
  voltage, comm_result, error = packet_handler_->read1ByteTxRx(port_handler_.get(), id, volt_addr);
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Failed to get voltage: %s", 
                packet_handler_->getTxRxResult(comm_result));
    return -1.0f;
  }
  
  return voltage / 10.0f;  // 电压单位为0.1V
}

bool FeeTechAdapter::setTorque(uint8_t id, bool enable)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Not connected to servo controller");
    return false;
  }
  
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  
  comm_result, error = packet_handler_->write1ByteTxRx(port_handler_.get(), id, ft_address::TORQUE_ENABLE, enable ? 1 : 0);
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Failed to set torque: %s", 
                packet_handler_->getTxRxResult(comm_result));
    return false;
  }
  
  return true;
}

bool FeeTechAdapter::isConnected() const
{
  return connected_;
}

std::map<uint8_t, int> FeeTechAdapter::syncReadPositions(const std::vector<uint8_t>& ids)
{
  std::map<uint8_t, int> positions;
  
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Not connected to servo controller");
    return positions;
  }
  
  // 清除之前的参数
  sync_read_->clearParam();
  
  // 添加舵机ID
  for (auto id : ids) {
    if (!sync_read_->addParam(id)) {
      RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), 
                  "Failed to add param for sync read, ID: %d", id);
    }
  }
  
  // 执行同步读取
  int comm_result = sync_read_->txRxPacket();
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), 
                "Failed to sync read: %s", packet_handler_->getTxRxResult(comm_result));
    return positions;
  }
  
  // 获取每个舵机的位置
  for (auto id : ids) {
    if (sync_read_->isAvailable(id, ft_address::PRESENT_POSITION, 2)) {
      positions[id] = sync_read_->getData(id, ft_address::PRESENT_POSITION, 2);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("feetech_adapter"), 
                 "Sync read data not available for ID: %d", id);
    }
  }
  
  return positions;
}

bool FeeTechAdapter::syncWritePositions(const std::map<uint8_t, uint16_t>& positions)
{
  if (!connected_) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), "Not connected to servo controller");
    return false;
  }
  
  // 清除之前的参数
  sync_write_->clearParam();
  
  // 添加每个舵机的位置
  for (const auto& pos : positions) {
    uint8_t id = pos.first;
    uint16_t position = pos.second;
    
    // 根据舵机类型调整参数
    uint16_t time = 0;
    uint16_t speed = 0;
    adjustParamsByType(id, position, time, speed);
    
    // 限制位置在舵机范围内
    position = clampPosition(id, position);
    
    // 准备参数
    uint8_t param[2];
    param[0] = SCS_LOBYTE(position);
    param[1] = SCS_HIBYTE(position);
    
    if (!sync_write_->addParam(id, param)) {
      RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), 
                  "Failed to add param for sync write, ID: %d", id);
      return false;
    }
  }
  
  // 执行同步写入
  int comm_result = sync_write_->txPacket();
  if (comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("feetech_adapter"), 
                "Failed to sync write: %s", packet_handler_->getTxRxResult(comm_result));
    return false;
  }
  
  return true;
}

void FeeTechAdapter::adjustParamsByType(uint8_t id, uint16_t& position, uint16_t& time, uint16_t& speed)
{
  auto it = servo_configs_.find(id);
  if (it == servo_configs_.end()) {
    // 没有找到配置，使用默认值
    return;
  }
  
  // 根据舵机类型调整参数
  switch (it->second.type) {
    case FTServoType::STS_TYPE1:
      // 类型1舵机不需要特殊调整
      break;
    
    case FTServoType::STS_TYPE2:
      // 类型2舵机可能需要不同的参数调整
      // 例如，不同的速度限制
      if (speed > 1000) speed = 1000;  // 假设类型2舵机的速度上限为1000
      break;
  }
  
  RCLCPP_DEBUG(rclcpp::get_logger("feetech_adapter"), 
              "Adjusted parameters for servo ID %d (Type %d): position=%d, time=%d, speed=%d",
              id, static_cast<int>(it->second.type), position, time, speed);
}

uint16_t FeeTechAdapter::clampPosition(uint8_t id, uint16_t position)
{
  auto it = servo_configs_.find(id);
  if (it == servo_configs_.end()) {
    // 没有找到配置，返回原始值
    return position;
  }
  
  // 限制位置在舵机范围内
  if (position < it->second.min_position) {
    position = it->second.min_position;
  } else if (position > it->second.max_position) {
    position = it->second.max_position;
  }
  
  return position;
}

} // namespace servo_control 