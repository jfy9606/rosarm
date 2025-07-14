#ifndef KINEMATICS_NODE_HPP
#define KINEMATICS_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <trajectory/srv/forward_kinematics.hpp>
#include <trajectory/srv/inverse_kinematics.hpp>
#include "trajectory/kinematics.hpp"

namespace kinematics {

class KinematicsNode : public rclcpp::Node
{
public:
  explicit KinematicsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~KinematicsNode() = default;

private:
  // 运动学对象
  std::unique_ptr<RobotKinematics> kinematics_;
  
  // 服务
  rclcpp::Service<trajectory::srv::ForwardKinematics>::SharedPtr fk_service_;
  rclcpp::Service<trajectory::srv::InverseKinematics>::SharedPtr ik_service_;
  
  // 参数回调
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  // 服务回调
  void handleForwardKinematics(
    const std::shared_ptr<trajectory::srv::ForwardKinematics::Request> request,
    std::shared_ptr<trajectory::srv::ForwardKinematics::Response> response);
  
  void handleInverseKinematics(
    const std::shared_ptr<trajectory::srv::InverseKinematics::Request> request,
    std::shared_ptr<trajectory::srv::InverseKinematics::Response> response);
  
  // 参数回调
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);
    
  // 从参数中加载机器人参数
  void loadRobotParameters();
};

} // namespace kinematics

#endif // KINEMATICS_NODE_HPP 