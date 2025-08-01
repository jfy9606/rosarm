#ifndef VACUUM_NODE_HPP
#define VACUUM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <servo/srv/vacuum_cmd.hpp>
#include <motor/motor_control.hpp>

namespace servo_control {

class VacuumNode : public rclcpp::Node
{
public:
  explicit VacuumNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~VacuumNode();

private:
  // 参数
  std::string device_port_;
  int baudrate_;
  int timeout_;
  
  // 电机控制对象
  std::unique_ptr<motor_control::MotorControl> motor_control_;
  
  // 真空吸盘状态
  bool vacuum_enabled_;
  int vacuum_power_;
  
  // 真空泵电机ID
  uint8_t vacuum_id_;
  
  // 服务
  rclcpp::Service<servo::srv::VacuumCmd>::SharedPtr vacuum_cmd_srv_;
  
  // 发布器
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vacuum_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr vacuum_power_pub_;
  
  // 订阅器
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vacuum_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr vacuum_power_sub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 参数回调
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // 服务回调
  void vacuumCmdCallback(
    const std::shared_ptr<servo::srv::VacuumCmd::Request> request,
    std::shared_ptr<servo::srv::VacuumCmd::Response> response);
  
  // 订阅回调
  void vacuumEnableCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void vacuumPowerCallback(const std_msgs::msg::Int16::SharedPtr msg);
  
  // 定时回调，发布真空泵状态
  void timerCallback();
  
  // 参数回调
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  
  // 初始化电机控制
  bool initMotorControl();
  
  // 设置真空吸盘状态
  bool setVacuumState(bool enable, int power);
};

} // namespace servo_control

#endif // VACUUM_NODE_HPP