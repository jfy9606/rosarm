#ifndef WRIST_NODE_HPP
#define WRIST_NODE_HPP

#include <rclcpp/rclcpp.hpp>
// 移除 sensor_msgs 依赖
#include <servo_interfaces/msg/ser_control.hpp>  // 使用生成的接口头文件
#include <servo_interfaces/srv/joint_control.hpp>
#include "servo/servo_control.hpp"
#include <string>
#include <vector>

namespace servo_control {

// 简单的关节状态结构体，代替sensor_msgs::msg::JointState
struct JointState {
  std::vector<std::string> name;
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
};

class WristNode : public rclcpp::Node
{
public:
  explicit WristNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~WristNode();

private:
  // 参数
  std::string device_port_;
  int baudrate_;
  int timeout_;
  
  // 舵机控制对象
  std::unique_ptr<ServoControl> servo_control_;
  
  // 关节ID映射
  std::map<std::string, uint8_t> joint_id_map_;
  
  // 关节限制
  std::map<std::string, std::pair<double, double>> joint_limits_;
  
  // 关节状态值
  std::map<std::string, double> joint_positions_;
  
  // 服务
  rclcpp::Service<servo_interfaces::srv::JointControl>::SharedPtr joint_control_srv_;
  
  // 发布器 - 使用我们自己的JointState结构体
  std::shared_ptr<JointState> current_joint_state_;
  
  // 订阅器
  rclcpp::Subscription<servo_interfaces::msg::SerControl>::SharedPtr servo_control_sub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 参数回调
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // 服务回调
  void jointControlCallback(
    const std::shared_ptr<servo_interfaces::srv::JointControl::Request> request,
    std::shared_ptr<servo_interfaces::srv::JointControl::Response> response);
  
  // 订阅回调
  void servoControlCallback(const servo_interfaces::msg::SerControl::SharedPtr msg);
  
  // 定时回调，更新关节状态
  void timerCallback();
  
  // 参数回调
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  
  // 初始化舵机控制
  bool initServoControl();
  
  // 将角度转换为舵机值
  uint16_t angleToServoValue(double angle, const std::string& joint_name);
  
  // 将舵机值转换为角度
  double servoValueToAngle(uint16_t value, const std::string& joint_name);
  
  // 加载关节限制
  void loadJointLimits();
};

} // namespace servo_control

#endif // WRIST_NODE_HPP 