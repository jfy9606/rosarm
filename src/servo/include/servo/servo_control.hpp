#ifndef SERVO_CONTROL_HPP
#define SERVO_CONTROL_HPP

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <serial/serial.h>
#include "servo/feetech_adapter.hpp"

namespace servo_control {

class ServoControl
{
public:
  ServoControl(
    const std::string& port = "/dev/ttyUSB1", 
    int baud_rate = 1000000, 
    int timeout = 100);
  
  virtual ~ServoControl();

  // 初始化舵机
  bool init();
  
  // 关闭连接
  void close();
  
  // 设置舵机位置
  bool setPosition(uint8_t id, uint16_t position, uint16_t time = 0, uint16_t speed = 0);
  
  // 设置舵机速度
  bool setSpeed(uint8_t id, int16_t speed);
  
  // 获取舵机位置
  int getPosition(uint8_t id);
  
  // 获取舵机温度
  int getTemperature(uint8_t id);
  
  // 获取舵机电压
  float getVoltage(uint8_t id);
  
  // 使能/失能舵机扭矩
  bool setTorque(uint8_t id, bool enable);
  
  // 检查连接状态
  bool isConnected() const;

  // 加载舵机配置
  bool loadServoConfigs(const std::string& config_file);

  // 同步读取多个舵机的位置
  std::map<uint8_t, int> syncReadPositions(const std::vector<uint8_t>& ids);
  
  // 同步写入多个舵机的位置
  bool syncWritePositions(const std::map<uint8_t, uint16_t>& positions);

private:
  // 串口连接 (用于传统方式)
  serial::Serial serial_;
  
  // Feetech适配器 (用于FT系列舵机)
  std::unique_ptr<FeeTechAdapter> ft_adapter_;
  
  // 串口参数
  std::string port_;
  int baud_rate_;
  int timeout_;
  
  // 连接状态
  bool connected_;
  
  // 舵机类型映射
  std::map<uint8_t, FTServoType> servo_types_;
  
  // 发送指令包 (传统方式)
  bool sendPacket(const std::vector<uint8_t>& packet);
  
  // 读取响应 (传统方式)
  std::vector<uint8_t> readPacket();
  
  // 计算校验和 (传统方式)
  uint8_t calculateChecksum(const std::vector<uint8_t>& data);
};

} // namespace servo_control

#endif // SERVO_CONTROL_HPP 