#include "trajectory/kinematics.hpp"
#include <cmath>

namespace kinematics {

RobotKinematics::RobotKinematics()
  : dof_(0)
{
  // 默认设置为5自由度机械臂的DH参数
  // 这些参数需要根据实际机器人的尺寸进行调整
  dof_ = 5;
  
  // 连杆长度 (a)
  a_ = {0.0, 0.1, 0.1, 0.0, 0.0};
  
  // 连杆扭角 (alpha)
  alpha_ = {M_PI/2, 0.0, 0.0, M_PI/2, 0.0};
  
  // 连杆偏距 (d)
  d_ = {0.1, 0.0, 0.0, 0.0, 0.1};
  
  // 关节角偏移 (theta_offset)
  theta_offset_ = {0.0, 0.0, 0.0, 0.0, 0.0};
  
  // 关节限制 (弧度)
  joint_limits_ = {
    {-M_PI, M_PI},     // 关节1：±180度
    {-M_PI/2, M_PI/2}, // 关节2：±90度
    {-M_PI/2, M_PI/2}, // 关节3：±90度
    {-M_PI, M_PI},     // 关节4：±180度
    {-M_PI, M_PI}      // 关节5：±180度
  };
  
  // 关节名称
  joint_names_ = {
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5"
  };
}

geometry_msgs::msg::Pose RobotKinematics::forwardKinematics(
  const std::vector<double>& joint_positions)
{
  // 检查输入参数
  if (static_cast<int>(joint_positions.size()) != dof_) {
    RCLCPP_ERROR(rclcpp::get_logger("kinematics"), 
                "Invalid joint positions array size: %zu (expected %d)", 
                joint_positions.size(), dof_);
    return geometry_msgs::msg::Pose();
  }
  
  // 计算变换矩阵
  std::vector<Eigen::Matrix4d> transforms = calculateTransforms(joint_positions);
  
  // 计算末端执行器位姿
  Eigen::Matrix4d end_effector_transform = Eigen::Matrix4d::Identity();
  for (const auto& transform : transforms) {
    end_effector_transform = end_effector_transform * transform;
  }
  
  // 从变换矩阵中提取位置和姿态
  Eigen::Vector3d position = end_effector_transform.block<3,1>(0,3);
  Eigen::Matrix3d rotation = end_effector_transform.block<3,3>(0,0);
  Eigen::Quaterniond quaternion(rotation);
  
  // 创建ROS Pose消息
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  pose.orientation.w = quaternion.w();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  
  return pose;
}

bool RobotKinematics::inverseKinematics(
  const geometry_msgs::msg::Pose& target_pose,
  std::vector<double>& joint_positions)
{
  // 初始化关节位置（如果大小不匹配）
  if (static_cast<int>(joint_positions.size()) != dof_) {
    joint_positions.resize(dof_, 0.0);
  }
  
  // 从ROS Pose消息中提取位置和姿态
  Eigen::Vector3d position(
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z
  );
  
  Eigen::Quaterniond quaternion(
    target_pose.orientation.w,
    target_pose.orientation.x,
    target_pose.orientation.y,
    target_pose.orientation.z
  );
  
  // 构建目标变换矩阵
  Eigen::Matrix4d target_transform = Eigen::Matrix4d::Identity();
  target_transform.block<3,3>(0,0) = quaternion.toRotationMatrix();
  target_transform.block<3,1>(0,3) = position;
  
  // 这里实现逆运动学算法
  // 注意：完整的逆运动学实现相当复杂，这里只是一个框架
  // 可以使用数值方法（如雅可比矩阵法）或解析方法
  
  // 由于逆运动学很复杂，这里使用简单的梯度下降法作为示例
  // 在实际应用中，应该使用更高效的算法
  const int max_iterations = 1000;
  const double step_size = 0.01;
  const double tolerance = 1e-5;
  
  std::vector<double> current_joints = joint_positions;
  double error = std::numeric_limits<double>::max();
  
  for (int iter = 0; iter < max_iterations && error > tolerance; ++iter) {
    // 计算当前位姿
    geometry_msgs::msg::Pose current_pose = forwardKinematics(current_joints);
    
    // 计算位置误差
    Eigen::Vector3d position_error(
      target_pose.position.x - current_pose.position.x,
      target_pose.position.y - current_pose.position.y,
      target_pose.position.z - current_pose.position.z
    );
    
    // 计算姿态误差（简化版）
    Eigen::Quaterniond current_quat(
      current_pose.orientation.w,
      current_pose.orientation.x,
      current_pose.orientation.y,
      current_pose.orientation.z
    );
    
    Eigen::Quaterniond target_quat(
      target_pose.orientation.w,
      target_pose.orientation.x,
      target_pose.orientation.y,
      target_pose.orientation.z
    );
    
    // 计算四元数之间的差异
    Eigen::Quaterniond error_quat = target_quat * current_quat.inverse();
    Eigen::AngleAxisd error_aa(error_quat);
    Eigen::Vector3d orientation_error = error_aa.axis() * error_aa.angle();
    
    // 计算总误差
    error = position_error.norm() + orientation_error.norm();
    
    // 如果误差小于阈值，则收敛
    if (error < tolerance) {
      break;
    }
    
    // 简单的数值梯度计算（前向差分）
    for (int j = 0; j < dof_; ++j) {
      // 保存当前关节值
      double original_value = current_joints[j];
      
      // 微小扰动
      current_joints[j] += 1e-6;
      geometry_msgs::msg::Pose perturbed_pose = forwardKinematics(current_joints);
      
      // 计算梯度
      Eigen::Vector3d position_gradient(
        (perturbed_pose.position.x - current_pose.position.x) * 1e6,
        (perturbed_pose.position.y - current_pose.position.y) * 1e6,
        (perturbed_pose.position.z - current_pose.position.z) * 1e6
      );
      
      // 恢复原始值
      current_joints[j] = original_value;
      
      // 更新关节值（只考虑位置误差）
      double update = step_size * position_gradient.dot(position_error);
      current_joints[j] += update;
      
      // 确保在关节限制范围内
      if (current_joints[j] < joint_limits_[j].first) {
        current_joints[j] = joint_limits_[j].first;
      } else if (current_joints[j] > joint_limits_[j].second) {
        current_joints[j] = joint_limits_[j].second;
      }
    }
  }
  
  // 如果误差足够小，则返回计算的关节值
  if (error < tolerance) {
    joint_positions = current_joints;
    return true;
  }
  
  // 逆运动学求解失败
  RCLCPP_WARN(rclcpp::get_logger("kinematics"), 
             "Inverse kinematics failed to converge (error: %f)", error);
  return false;
}

bool RobotKinematics::isValidJointPosition(const std::vector<double>& joint_positions)
{
  // 检查关节数量
  if (static_cast<int>(joint_positions.size()) != dof_) {
    return false;
  }
  
  // 检查每个关节是否在限制范围内
  for (int i = 0; i < dof_; ++i) {
    if (joint_positions[i] < joint_limits_[i].first || 
        joint_positions[i] > joint_limits_[i].second) {
      return false;
    }
  }
  
  return true;
}

std::vector<std::pair<double, double>> RobotKinematics::getJointLimits()
{
  return joint_limits_;
}

std::vector<std::string> RobotKinematics::getJointNames()
{
  return joint_names_;
}

void RobotKinematics::setDHParameters(
  const std::vector<double>& a,
  const std::vector<double>& alpha,
  const std::vector<double>& d,
  const std::vector<double>& theta_offset)
{
  // 检查所有参数的大小是否一致
  if (a.size() != alpha.size() || a.size() != d.size() || a.size() != theta_offset.size()) {
    RCLCPP_ERROR(rclcpp::get_logger("kinematics"), 
                "DH parameter arrays have inconsistent sizes");
    return;
  }
  
  // 更新参数
  a_ = a;
  alpha_ = alpha;
  d_ = d;
  theta_offset_ = theta_offset;
  
  // 更新自由度
  dof_ = static_cast<int>(a.size());
  
  // 重新调整关节限制和名称大小
  if (static_cast<int>(joint_limits_.size()) != dof_) {
    joint_limits_.resize(dof_, {-M_PI, M_PI});
  }
  
  if (static_cast<int>(joint_names_.size()) != dof_) {
    joint_names_.clear();
  for (int i = 0; i < dof_; ++i) {
      joint_names_.push_back("joint" + std::to_string(i+1));
    }
  }
}

void RobotKinematics::setJointLimits(const std::vector<std::pair<double, double>>& limits)
{
  // 检查参数大小是否一致
  if (static_cast<int>(limits.size()) != dof_) {
    RCLCPP_ERROR(rclcpp::get_logger("kinematics"), 
                "Joint limits array size (%zu) doesn't match DOF (%d)", 
                limits.size(), dof_);
    return;
  }
  
  // 更新关节限制
  joint_limits_ = limits;
}

bool RobotKinematics::setJointLimits(
  const std::vector<double>& min_limits,
  const std::vector<double>& max_limits)
{
  // 检查参数大小是否一致
  if (static_cast<int>(min_limits.size()) != dof_ || 
      static_cast<int>(max_limits.size()) != dof_) {
    RCLCPP_ERROR(rclcpp::get_logger("kinematics"), 
                "Joint limits arrays size doesn't match DOF (%d)", dof_);
    return false;
  }
  
  // 检查限制是否有效（最小值 <= 最大值）
  for (int i = 0; i < dof_; ++i) {
    if (min_limits[i] > max_limits[i]) {
      RCLCPP_ERROR(rclcpp::get_logger("kinematics"), 
                  "Invalid joint limits: min > max for joint %d", i);
      return false;
    }
  }
  
  // 更新关节限制
  for (int i = 0; i < dof_; ++i) {
    joint_limits_[i] = {min_limits[i], max_limits[i]};
  }
  
  return true;
}

void RobotKinematics::setJointNames(const std::vector<std::string>& names)
{
  if (static_cast<int>(names.size()) != dof_) {
    RCLCPP_ERROR(rclcpp::get_logger("kinematics"), 
                "Joint names array size (%zu) does not match DOF (%d)", 
                names.size(), dof_);
    return;
  }
  
  joint_names_ = names;
}

std::vector<Eigen::Matrix4d> RobotKinematics::calculateTransforms(
  const std::vector<double>& joint_positions)
{
  std::vector<Eigen::Matrix4d> transforms;
  
  for (int i = 0; i < dof_; ++i) {
    // 计算关节角（考虑偏移）
    double theta = joint_positions[i] + theta_offset_[i];
    
    // 计算DH变换矩阵
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    
    // 旋转（绕Z轴旋转theta）
    transform(0, 0) = std::cos(theta);
    transform(0, 1) = -std::sin(theta);
    transform(1, 0) = std::sin(theta);
    transform(1, 1) = std::cos(theta);
    
    // 平移（沿X轴平移a）
    transform(0, 3) = a_[i];
    
    // 平移（沿Z轴平移d）
    transform(2, 3) = d_[i];
    
    // 旋转（绕X轴旋转alpha）
    Eigen::Matrix4d rot_x = Eigen::Matrix4d::Identity();
    rot_x(1, 1) = std::cos(alpha_[i]);
    rot_x(1, 2) = -std::sin(alpha_[i]);
    rot_x(2, 1) = std::sin(alpha_[i]);
    rot_x(2, 2) = std::cos(alpha_[i]);
    
    transform = transform * rot_x;
    transforms.push_back(transform);
  }
  
  return transforms;
}

} // namespace kinematics 