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
    packet_handler_ = std::shared_ptr<PacketHandler>(new PacketHandler());
    
    // 打开端口
    if (!port_handler_->openPort()) {
      std::cerr << "Failed to open port " << port_ << std::endl;
      return false;
    }
    
    // 设置波特率
    if (!port_handler_->setBaudRate(baud_rate_)) {
      std::cerr << "Failed to set baudrate " << baud_rate_ << std::endl;
      port_handler_->closePort();
      return false;
    }
    
    // 创建同步读取和写入处理器
    sync_read_ = std::make_shared<GroupSyncRead>(port_handler_.get(), packet_handler_.get(), 
                                                ft_address::PRESENT_POSITION, 4);
    sync_write_ = std::make_shared<GroupSyncWrite>(port_handler_.get(), packet_handler_.get(), 
                                                  ft_address::GOAL_POSITION, 2);
    
    connected_ = true;
    std::cout << "Connected to " << port_ << " at " << baud_rate_ << " baud" << std::endl;
    
    return true;
  }
  catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
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
    std::cout << "Port closed" << std::endl;
  }
}

void FeeTechAdapter::loadServoConfigs(const std::vector<FTServoConfig>& configs)
{
  servo_configs_.clear();
  for (const auto& config : configs) {
    servo_configs_[config.id] = config;
    std::cout << "Loaded config for servo ID " << static_cast<int>(config.id) 
              << ", type " << static_cast<int>(config.type) 
              << ", name " << config.name << std::endl;
  }
}

bool FeeTechAdapter::setPosition(uint8_t id, uint16_t position, uint16_t time, uint16_t speed)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
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
  
  comm_result = packet_handler_->write1ByteTxRx(port_handler_.get(), id, ft_address::GOAL_ACC, acc, &error);
  if (comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to set acceleration: " << packet_handler_->getTxRxResult(comm_result) << std::endl;
    return false;
  }
  
  // 设置速度
  comm_result = packet_handler_->write2ByteTxRx(port_handler_.get(), id, ft_address::GOAL_SPEED, speed, &error);
  if (comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to set speed: " << packet_handler_->getTxRxResult(comm_result) << std::endl;
    return false;
  }
  
  // 设置位置
  comm_result = packet_handler_->write2ByteTxRx(port_handler_.get(), id, ft_address::GOAL_POSITION, position, &error);
  if (comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to set position: " << packet_handler_->getTxRxResult(comm_result) << std::endl;
    return false;
  }
  
  return true;
}

bool FeeTechAdapter::setSpeed(uint8_t id, int16_t speed)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return false;
  }
  
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  
  comm_result = packet_handler_->write2ByteTxRx(port_handler_.get(), id, ft_address::GOAL_SPEED, speed, &error);
  if (comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to set speed: " << packet_handler_->getTxRxResult(comm_result) << std::endl;
    return false;
  }
  
  return true;
}

int FeeTechAdapter::getPosition(uint8_t id)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return -1;
  }
  
  uint16_t position = 0;
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  
  comm_result = packet_handler_->read2ByteTxRx(port_handler_.get(), id, ft_address::PRESENT_POSITION, &position, &error);
  if (comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to get position: " << packet_handler_->getTxRxResult(comm_result) << std::endl;
    return -1;
  }
  
  return position;
}

int FeeTechAdapter::getTemperature(uint8_t id)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return -1;
  }
  
  // 温度寄存器地址 (假设为0x3F，需要根据实际舵机型号调整)
  uint8_t temp_addr = 0x3F;
  uint8_t temperature = 0;
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  
  comm_result = packet_handler_->read1ByteTxRx(port_handler_.get(), id, temp_addr, &temperature, &error);
  if (comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to get temperature: " << packet_handler_->getTxRxResult(comm_result) << std::endl;
    return -1;
  }
  
  return temperature;
}

float FeeTechAdapter::getVoltage(uint8_t id)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return -1.0f;
  }
  
  // 电压寄存器地址 (假设为0x3E，需要根据实际舵机型号调整)
  uint8_t volt_addr = 0x3E;
  uint8_t voltage = 0;
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  
  comm_result = packet_handler_->read1ByteTxRx(port_handler_.get(), id, volt_addr, &voltage, &error);
  if (comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to get voltage: " << packet_handler_->getTxRxResult(comm_result) << std::endl;
    return -1.0f;
  }
  
  return voltage / 10.0f;  // 电压单位为0.1V
}

bool FeeTechAdapter::setTorque(uint8_t id, bool enable)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return false;
  }
  
  int comm_result = COMM_TX_FAIL;
  uint8_t error = 0;
  
  comm_result = packet_handler_->write1ByteTxRx(port_handler_.get(), id, ft_address::TORQUE_ENABLE, enable ? 1 : 0, &error);
  if (comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to set torque: " << packet_handler_->getTxRxResult(comm_result) << std::endl;
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
    std::cerr << "Not connected to servo controller" << std::endl;
    return positions;
  }
  
  // 清除之前的参数
  sync_read_->clearParam();
  
  // 添加所有ID
  for (auto id : ids) {
    if (!sync_read_->addParam(id)) {
      std::cerr << "Failed to add ID " << static_cast<int>(id) << " to sync read" << std::endl;
      return positions;
    }
  }
  
  // 执行同步读取
  int comm_result = sync_read_->txRxPacket();
  if (comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to sync read positions: " << packet_handler_->getTxRxResult(comm_result) << std::endl;
    return positions;
  }
  
  // 获取每个舵机的位置
  for (auto id : ids) {
    // 检查数据是否可用
    if (sync_read_->isAvailable(id, ft_address::PRESENT_POSITION, 2)) {
      positions[id] = sync_read_->getData16(id, ft_address::PRESENT_POSITION);
    } else {
      std::cerr << "Position data not available for ID " << static_cast<int>(id) << std::endl;
    }
  }
  
  return positions;
}

bool FeeTechAdapter::syncWritePositions(const std::map<uint8_t, uint16_t>& positions)
{
  if (!connected_) {
    std::cerr << "Not connected to servo controller" << std::endl;
    return false;
  }
  
  // 清除之前的参数
  sync_write_->clearParam();
  
  // 添加所有位置
  for (const auto& pos : positions) {
    uint8_t id = pos.first;
    uint16_t position = pos.second;
    
    // 限制位置在舵机范围内
    position = clampPosition(id, position);
    
    // 准备数据
    uint8_t param[2];
    param[0] = SCS_LOBYTE(position);
    param[1] = SCS_HIBYTE(position);
    
    // 添加参数
    if (!sync_write_->addParam(id, param)) {
      std::cerr << "Failed to add ID " << static_cast<int>(id) << " to sync write" << std::endl;
      return false;
    }
  }
  
  // 执行同步写入
  int comm_result = sync_write_->txPacket();
  if (comm_result != COMM_SUCCESS) {
    std::cerr << "Failed to sync write positions: " << packet_handler_->getTxRxResult(comm_result) << std::endl;
    return false;
  }
  
  return true;
}

void FeeTechAdapter::adjustParamsByType(uint8_t id, uint16_t& position, uint16_t& time, uint16_t& speed)
{
  // 根据舵机类型调整参数
  if (servo_configs_.find(id) != servo_configs_.end()) {
    FTServoType type = servo_configs_[id].type;
    
    switch (type) {
      case FTServoType::STS_TYPE1:
        // STS 类型1舵机参数调整
        // 这里可以根据具体舵机型号调整参数
        std::cout << "Adjusting parameters for STS TYPE1 servo ID " << static_cast<int>(id) 
                  << ": position=" << position << ", time=" << time << ", speed=" << speed << std::endl;
        break;
        
      case FTServoType::STS_TYPE2:
        // STS 类型2舵机参数调整
        // 这里可以根据具体舵机型号调整参数
        std::cout << "Adjusting parameters for STS TYPE2 servo ID " << static_cast<int>(id) 
                  << ": position=" << position << ", time=" << time << ", speed=" << speed << std::endl;
        break;
        
      default:
        // 默认不做调整
        break;
    }
  }
}

uint16_t FeeTechAdapter::clampPosition(uint8_t id, uint16_t position)
{
  // 检查是否有该舵机的配置
  if (servo_configs_.find(id) != servo_configs_.end()) {
    const auto& config = servo_configs_[id];
    
    // 限制在舵机允许范围内
    if (position < config.min_position) {
      position = config.min_position;
    }
    else if (position > config.max_position) {
      position = config.max_position;
    }
  }
  
  return position;
}

} // namespace servo_control 