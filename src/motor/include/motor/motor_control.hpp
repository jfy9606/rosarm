#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <motor/msg/motor_order.hpp>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <serial/serial.h>

namespace motor_control {

// 电机类型枚举
enum class MotorType {
  AI_MOTOR,  // AImotor - 大臂进给电机
  YF_MOTOR   // YF - 大臂俯仰电机
};

// 电机配置结构
struct MotorConfig {
  uint8_t id;
  std::string name;
  MotorType type;
  int max_speed;
  int max_acc;
};

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
  bool executeOrder(const std::shared_ptr<motor::msg::MotorOrder> order);
  
  // 获取电机状态
  std::string getStatus(uint8_t station_num);
  
  // 检查连接状态
  bool isConnected() const;

  // 加载电机配置
  bool loadMotorConfigs(const std::string& config_file);

private:
  // 串口连接
  serial::Serial serial_;
  
  // 串口参数
  std::string port_;
  int baud_rate_;
  int timeout_;
  
  // 连接状态
  bool connected_;
  
  // 电机配置映射
  std::map<uint8_t, MotorConfig> motor_configs_;
  
  // 发送命令并等待响应
  std::string sendCommand(const std::string& command, bool wait_response = true);
  
  // 检验命令格式
  bool validateCommand(const std::string& command);
  
  // 根据电机类型调整参数
  void adjustParamsByType(uint8_t station_num, int32_t& position, int16_t& velocity, 
                         uint16_t& acc, uint16_t& dec);
  
  // 获取电机类型
  MotorType getMotorType(uint8_t station_num) const;
};

} // namespace motor_control

#endif // MOTOR_CONTROL_HPP 