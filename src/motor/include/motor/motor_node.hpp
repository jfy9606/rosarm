#ifndef MOTOR_NODE_HPP
#define MOTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <motor/msg/motor_order.hpp>
#include "motor/motor_control.hpp"

namespace motor_control {

class MotorNode : public rclcpp::Node
{
public:
  explicit MotorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~MotorNode();

private:
  // 参数
  std::string device_port_;
  int baudrate_;
  int timeout_;

  // 电机控制对象
  std::unique_ptr<MotorControl> motor_control_;
  
  // 消息订阅
  rclcpp::Subscription<motor::msg::MotorOrder>::SharedPtr order_sub_;
  
  // 状态发布
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  
  // 参数变更回调
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 处理电机指令
  void motorOrderCallback(const motor::msg::MotorOrder::SharedPtr msg);
  
  // 定时发布状态信息
  void timerCallback();
  
  // 参数回调
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
    
  // 初始化电机连接
  bool initMotorControl();
};

} // namespace motor_control

#endif // MOTOR_NODE_HPP 