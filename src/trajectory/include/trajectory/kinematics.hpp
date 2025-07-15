#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory/srv/forward_kinematics.hpp>
#include <trajectory/srv/inverse_kinematics.hpp>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace kinematics {

class RobotKinematics
{
public:
  RobotKinematics();
  virtual ~RobotKinematics() = default;
  
  // 前向运动学：计算关节空间到笛卡尔空间的映射
  geometry_msgs::msg::Pose forwardKinematics(
    const std::vector<double>& joint_positions);
  
  // 逆运动学：计算笛卡尔空间到关节空间的映射
  bool inverseKinematics(
    const geometry_msgs::msg::Pose& target_pose,
    std::vector<double>& joint_positions);
    
  // 检查关节空间位置是否有效（在限制范围内）
  bool isValidJointPosition(
    const std::vector<double>& joint_positions);
    
  // 获取关节限制
  std::vector<std::pair<double, double>> getJointLimits();
  
  // 获取关节名称
  std::vector<std::string> getJointNames();
  
  // 设置机器人DH参数
  void setDHParameters(
    const std::vector<double>& a,    // 连杆长度
    const std::vector<double>& alpha, // 连杆扭角
    const std::vector<double>& d,    // 连杆偏距
    const std::vector<double>& theta_offset); // 关节角偏移
    
  // 设置关节限制
  void setJointLimits(
    const std::vector<std::pair<double, double>>& limits);
    
  // 设置关节限制（使用分离的min/max向量）
  bool setJointLimits(
    const std::vector<double>& min_limits,
    const std::vector<double>& max_limits);
    
  // 设置关节名称
  void setJointNames(
    const std::vector<std::string>& names);

private:
  // DH参数
  std::vector<double> a_;     // 连杆长度
  std::vector<double> alpha_; // 连杆扭角
  std::vector<double> d_;     // 连杆偏距
  std::vector<double> theta_offset_; // 关节角偏移
  
  // 关节限制
  std::vector<std::pair<double, double>> joint_limits_;
  
  // 关节名称
  std::vector<std::string> joint_names_;
  
  // 自由度（关节数量）
  int dof_;
  
  // 计算变换矩阵
  std::vector<Eigen::Matrix4d> calculateTransforms(
    const std::vector<double>& joint_positions);
};

} // namespace kinematics

#endif // KINEMATICS_HPP 