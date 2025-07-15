#ifndef MOTOR_NODE_HPP
#define MOTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <motor/msg/motor_order.hpp>
#include <motor/srv/motor_control.hpp>
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
  
  // 服务
  rclcpp::Service<motor::srv::MotorControl>::SharedPtr motor_control_srv_;
  
  // 发布器
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_status_pub_;
  
  // 订阅器
  rclcpp::Subscription<motor::msg::MotorOrder>::SharedPtr motor_order_sub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 参数回调
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // 订阅回调
  void motorOrderCallback(const motor::msg::MotorOrder::SharedPtr msg);
  
  // 服务回调
  void motorControlCallback(
    const std::shared_ptr<motor::srv::MotorControl::Request> request,
    std::shared_ptr<motor::srv::MotorControl::Response> response);
  
  // 定时回调，发布电机状态
  void timerCallback();
  
  // 参数回调
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
    
  // 初始化电机控制
  bool initMotorControl();
};

} // namespace motor_control

#endif // MOTOR_NODE_HPP 