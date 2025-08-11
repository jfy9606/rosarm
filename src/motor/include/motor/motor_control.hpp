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
#include <array>

namespace motor_control {

// 电机类型枚举
enum class MotorType {
  AI_MOTOR,  // AImotor - 大臂进给电机
  YF_MOTOR,  // YF - 原大臂俯仰电机（保留兼容性）
  YSF4_HAL_MOTOR // YSF4_HAL_MOTOR-650.FOC_v5.2.0_57BLDC_HALLSensor
};

// 电机协议类型枚举
enum class ProtocolType {
  MODBUS_RTU,  // ModBus RTU协议
  TEXT_COMMAND  // 文本命令协议
};

// 电机配置结构
struct MotorConfig {
  uint8_t id;
  std::string name;
  MotorType type;
  ProtocolType protocol;
  int max_speed;
  int max_acc;
  int max_current;      // mA
  int rated_voltage;    // mV
  int pole_pairs;       // BLDC极对数
  int encoder_resolution; // 编码器分辨率
  bool hall_sensor;     // 霍尔传感器
  int temp_limit;       // 摄氏度
  float kv_rating;      // RPM/V
  int overcurrent_protection; // mA
  int thermal_protection;     // 摄氏度
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
  bool setPosition(uint8_t station_num, int32_t position, uint16_t threshold = 0,
                  uint16_t vel = 1000, uint16_t acc = 100);
  
  // 设置速度模式，以指定速度运行
  bool setVelocity(uint8_t station_num, int16_t velocity, 
                  uint16_t acc = 100, uint16_t dec = 100);
  
  // 执行电机指令
  bool executeOrder(const std::shared_ptr<motor::msg::MotorOrder> order);
  
  // 使能电机
  bool enableMotor(uint8_t station_num, bool enable);
  
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
  
  // 电机模式（速度/位置）缓存
  std::map<uint8_t, int8_t> motor_mode_cache_;
  
  // 发送命令并等待响应
  std::string sendCommand(const std::string& command, bool wait_response = true);
  
  // 发送二进制命令并等待响应
  std::vector<uint8_t> sendBinaryCommand(const std::vector<uint8_t>& command, bool wait_response = true);
  
  // 检验命令格式
  bool validateCommand(const std::string& command);
  
  // 根据电机类型调整参数
  void adjustParamsByType(uint8_t station_num, int32_t& position, int16_t& velocity, 
                         uint16_t& acc, uint16_t& dec);
  
  // 获取电机类型
  MotorType getMotorType(uint8_t station_num) const;
  
  // 获取电机协议类型
  ProtocolType getProtocolType(uint8_t station_num) const;
  
  // 设置电机控制模式 (0=位置模式, 1=速度模式)
  bool setControlMode(uint8_t station_num, int8_t mode);
  
  // ModBus协议相关方法
  std::vector<uint8_t> buildModBusCommand(uint8_t station_num, const std::vector<uint8_t>& data);
  std::vector<uint8_t> calculateCRC16(const std::vector<uint8_t>& message, uint8_t length, uint8_t station_num);
  
  // 预定义的ModBus命令代码
  std::vector<std::vector<uint8_t>> code_list;
  std::vector<std::vector<uint8_t>> motor_list;
  
  // 初始化预定义命令列表
  void initCommandList();
};

} // namespace motor_control

#endif // MOTOR_CONTROL_HPP 