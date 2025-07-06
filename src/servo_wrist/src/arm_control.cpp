#include <servo_wrist/arm_control.h>
#include <algorithm>

namespace servo_wrist {

ArmControl::ArmControl() {
  // 初始化关节名称
  joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  
  // 初始化关节位置
  current_joint_positions_.resize(joint_names_.size(), 0.0);
}

ArmControl::~ArmControl() {
  // 在析构函数中释放资源（如有需要）
}

void ArmControl::setJointPositions(const std::vector<double>& positions) {
  // 检查位置数量是否匹配
  if (positions.size() != joint_names_.size()) {
    return;
  }
  
  // 更新当前关节位置
  current_joint_positions_ = positions;
}

std::vector<double> ArmControl::getJointPositions() const {
  return current_joint_positions_;
}

void ArmControl::updateJointState(const std::vector<double>& joint_positions) {
  // 检查位置数量是否匹配
  if (joint_positions.size() != current_joint_positions_.size()) {
    return;
  }
  
  // 更新当前关节位置
  current_joint_positions_ = joint_positions;
}

const std::vector<std::string>& ArmControl::getJointNames() const {
  return joint_names_;
}

} // namespace servo_wrist 