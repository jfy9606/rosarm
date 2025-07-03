#include <servo_wrist/arm_controller.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <algorithm>

namespace servo_wrist {

ArmController::ArmController(ros::NodeHandle& nh) : nh_(nh) {
  // 初始化关节名称
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  
  // 初始化关节位置
  current_joint_positions_.resize(joint_names_.size(), 0.0);
  
  // 创建关节控制发布器
  joint_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/arm_controller/command", 10);
  
  // 创建关节状态订阅器
  joint_state_sub_ = nh_.subscribe("/joint_states", 10, &ArmController::updateJointState, this);
  
  ROS_INFO("ArmController初始化完成");
}

ArmController::~ArmController() {
  // 在析构函数中释放资源
  joint_pub_.shutdown();
  joint_state_sub_.shutdown();
}

void ArmController::setJointPositions(const std::vector<double>& positions) {
  // 检查位置数量是否匹配
  if (positions.size() != joint_names_.size()) {
    ROS_ERROR("关节位置数量不匹配: 期望 %zu, 实际 %zu", joint_names_.size(), positions.size());
    return;
  }
  
  // 创建并发布关节位置命令
  std_msgs::Float64MultiArray msg;
  msg.data = positions;
  joint_pub_.publish(msg);
  
  // 更新当前关节位置
  current_joint_positions_ = positions;
  
  ROS_DEBUG("已发送关节位置命令");
}

std::vector<double> ArmController::getJointPositions() const {
  return current_joint_positions_;
}

void ArmController::updateJointState(const sensor_msgs::JointState::ConstPtr& msg) {
  // 遍历收到的关节状态
  for (size_t i = 0; i < msg->name.size(); ++i) {
    // 查找对应的关节索引
    for (size_t j = 0; j < joint_names_.size(); ++j) {
      if (msg->name[i] == joint_names_[j] && i < msg->position.size()) {
        current_joint_positions_[j] = msg->position[i];
        break;
      }
    }
  }
}

} // namespace servo_wrist 