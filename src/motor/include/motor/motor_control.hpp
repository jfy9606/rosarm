#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <motor/msg/motor_order.hpp>
#include <memory>
#include <string>
#include <vector>
#include <serial/serial.h>

namespace motor_control {

class MotorControl
{
public:
  MotorControl(
    const std::string& port = "/dev/ttyUSB0", 
    int baud_rate = 115200, 
    int timeout = 1000);
  
  virtual ~MotorControl();

  // 初始化电机
  bool init();
  
  // 关闭连接
  void close();
  
  // 设置位置模式，移动到指定位置
  bool setPosition(uint8_t station_num, int32_t position, uint16_t threshold = 0);
  
  // 设置速度模式，以指定速度运行
  bool setVelocity(uint8_t station_num, int16_t velocity, 
                  uint16_t acc = 100, uint16_t dec = 100);
  
  // 执行电机指令
  bool executeOrder(const motor::msg::MotorOrder::SharedPtr order);
  
  // 获取电机状态
  std::string getStatus(uint8_t station_num);
  
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
  
  // 发送命令并等待响应
  std::string sendCommand(const std::string& command, bool wait_response = true);
  
  // 检验命令格式
  bool validateCommand(const std::string& command);
};

} // namespace motor_control

#endif // MOTOR_CONTROL_HPP 