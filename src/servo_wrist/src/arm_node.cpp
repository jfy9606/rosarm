#include <servo_wrist/arm_node.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <algorithm>

namespace servo_wrist {

ArmNode::ArmNode(ros::NodeHandle& nh) : nh_(nh) {
  // 创建关节控制发布器
  joint_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/arm_controller/command", 10);
  
  // 创建关节状态订阅器
  joint_state_sub_ = nh_.subscribe("/joint_states", 10, &ArmNode::updateJointState, this);
  
  ROS_INFO("ArmNode初始化完成");
}

ArmNode::~ArmNode() {
  // 在析构函数中释放资源
  joint_pub_.shutdown();
  joint_state_sub_.shutdown();
}

void ArmNode::setJointPositions(const std::vector<double>& positions) {
  // 使用ArmControl设置关节位置
  arm_control_.setJointPositions(positions);
  
  // 创建并发布关节位置命令
  std_msgs::Float64MultiArray msg;
  msg.data = positions;
  joint_pub_.publish(msg);
  
  ROS_DEBUG("已发送关节位置命令");
}

std::vector<double> ArmNode::getJointPositions() const {
  // 从ArmControl获取当前关节位置
  return arm_control_.getJointPositions();
}

void ArmNode::updateJointState(const sensor_msgs::JointState::ConstPtr& msg) {
  // 获取关节名称列表
  const auto& joint_names = arm_control_.getJointNames();
  
  // 创建临时数组存储更新的关节位置
  std::vector<double> updated_positions(joint_names.size(), 0.0);
  bool found_positions = false;
  
  // 遍历收到的关节状态
  for (size_t i = 0; i < msg->name.size(); ++i) {
    // 查找对应的关节索引
    for (size_t j = 0; j < joint_names.size(); ++j) {
      if (msg->name[i] == joint_names[j] && i < msg->position.size()) {
        updated_positions[j] = msg->position[i];
        found_positions = true;
        break;
      }
    }
  }
  
  // 如果找到了有效的关节位置，更新ArmControl
  if (found_positions) {
    arm_control_.updateJointState(updated_positions);
  }
}

} // namespace servo_wrist 