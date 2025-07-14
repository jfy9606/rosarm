#include "trajectory/kinematics_node.hpp"
#include <vector>

namespace kinematics {

KinematicsNode::KinematicsNode(const rclcpp::NodeOptions & options)
: Node("kinematics_node", options)
{
  // 创建运动学对象
  kinematics_ = std::make_unique<RobotKinematics>();
  
  // 从参数加载机器人参数
  loadRobotParameters();
  
  // 创建服务
  fk_service_ = this->create_service<trajectory::srv::ForwardKinematics>(
    "forward_kinematics",
    std::bind(&KinematicsNode::handleForwardKinematics, this,
              std::placeholders::_1, std::placeholders::_2));
              
  ik_service_ = this->create_service<trajectory::srv::InverseKinematics>(
    "inverse_kinematics",
    std::bind(&KinematicsNode::handleInverseKinematics, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // 设置参数回调
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&KinematicsNode::parametersCallback, this, std::placeholders::_1));
    
  RCLCPP_INFO(this->get_logger(), "Kinematics node initialized");
}

void KinematicsNode::handleForwardKinematics(
  const std::shared_ptr<trajectory::srv::ForwardKinematics::Request> request,
  std::shared_ptr<trajectory::srv::ForwardKinematics::Response> response)
{
  // 检查输入关节位置数量
  if (request->joint_positions.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Forward kinematics request contains no joint positions");
    response->success = false;
    return;
  }
  
  // 将请求中的关节位置转换为向量
  std::vector<double> joint_positions(request->joint_positions.begin(), 
                                     request->joint_positions.end());
  
  // 检查关节位置是否有效
  if (!kinematics_->isValidJointPosition(joint_positions)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid joint positions in forward kinematics request");
    response->success = false;
    return;
  }
  
  // 计算前向运动学
  response->pose = kinematics_->forwardKinematics(joint_positions);
  response->success = true;
  
  RCLCPP_INFO(this->get_logger(), "Forward kinematics computed successfully");
}

void KinematicsNode::handleInverseKinematics(
  const std::shared_ptr<trajectory::srv::InverseKinematics::Request> request,
  std::shared_ptr<trajectory::srv::InverseKinematics::Response> response)
{
  // 初始关节位置（如果提供）
  std::vector<double> joint_positions;
  if (!request->initial_positions.empty()) {
    joint_positions.assign(request->initial_positions.begin(), 
                          request->initial_positions.end());
  }
  
  // 计算逆运动学
  bool success = kinematics_->inverseKinematics(request->target_pose, joint_positions);
  
  if (success) {
    // 将结果复制到响应
    response->joint_positions.assign(joint_positions.begin(), joint_positions.end());
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Inverse kinematics computed successfully");
  } else {
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "Failed to compute inverse kinematics");
  }
}

rcl_interfaces::msg::SetParametersResult KinematicsNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  bool should_reload = false;
  
  // 处理参数更新
  for (const auto & param : parameters) {
    // 如果更新了机器人参数，标记为需要重新加载
    if (param.get_name().find("robot.") == 0) {
      should_reload = true;
    }
  }
  
  // 如果需要，重新加载机器人参数
  if (should_reload) {
    loadRobotParameters();
  }
  
  return result;
}

void KinematicsNode::loadRobotParameters()
{
  // 声明默认参数
  this->declare_parameter<std::vector<double>>("robot.dh.a", {0.0, 0.1, 0.1, 0.0, 0.0});
  this->declare_parameter<std::vector<double>>("robot.dh.alpha", {M_PI/2, 0.0, 0.0, M_PI/2, 0.0});
  this->declare_parameter<std::vector<double>>("robot.dh.d", {0.1, 0.0, 0.0, 0.0, 0.1});
  this->declare_parameter<std::vector<double>>("robot.dh.theta_offset", {0.0, 0.0, 0.0, 0.0, 0.0});
  
  // 获取DH参数
  std::vector<double> a = this->get_parameter("robot.dh.a").as_double_array();
  std::vector<double> alpha = this->get_parameter("robot.dh.alpha").as_double_array();
  std::vector<double> d = this->get_parameter("robot.dh.d").as_double_array();
  std::vector<double> theta_offset = this->get_parameter("robot.dh.theta_offset").as_double_array();
  
  // 设置DH参数
  kinematics_->setDHParameters(a, alpha, d, theta_offset);
  
  // 尝试获取关节限制（如果参数存在）
  if (this->has_parameter("robot.joint_limits")) {
    // 注意：由于关节限制是pair<double, double>，需要特殊处理
    // 在实际应用中，可能需要定制参数解析
    // 这里只是一个简化示例
    RCLCPP_INFO(this->get_logger(), "Joint limits parameter exists, but custom parsing required");
  }
  
  // 尝试获取关节名称（如果参数存在）
  if (this->has_parameter("robot.joint_names")) {
    std::vector<std::string> joint_names = 
      this->get_parameter("robot.joint_names").as_string_array();
    
    // 设置关节名称
    if (!joint_names.empty()) {
      kinematics_->setJointNames(joint_names);
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Robot parameters loaded");
}

} // namespace kinematics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kinematics::KinematicsNode) 