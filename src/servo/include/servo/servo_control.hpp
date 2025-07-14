#ifndef SERVO_CONTROL_HPP
#define SERVO_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <servo/msg/ser_control.hpp>
#include <memory>
#include <string>
#include <vector>
#include <serial/serial.h>

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

private:
  // 串口连接
  serial::Serial serial_;
  
  // 串口参数
  std::string port_;
  int baud_rate_;
  int timeout_;
  
  // 连接状态
  bool connected_;
  
  // 发送指令包
  bool sendPacket(const std::vector<uint8_t>& packet);
  
  // 读取响应
  std::vector<uint8_t> readPacket();
  
  // 计算校验和
  uint8_t calculateChecksum(const std::vector<uint8_t>& data);
};

} // namespace servo_control

#endif // SERVO_CONTROL_HPP 