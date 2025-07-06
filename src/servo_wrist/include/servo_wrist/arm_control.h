#ifndef SERVO_WRIST_ARM_CONTROL_H
#define SERVO_WRIST_ARM_CONTROL_H

#include <vector>
#include <string>

namespace servo_wrist {

/**
 * @brief 机械臂控制类，包含核心机械臂控制逻辑，与ROS无关
 */
class ArmControl {
public:
  ArmControl();
  ~ArmControl();

  /**
   * @brief 设置关节位置
   * @param positions 关节位置数组
   */
  void setJointPositions(const std::vector<double>& positions);
  
  /**
   * @brief 获取当前关节位置
   * @return 关节位置数组
   */
  std::vector<double> getJointPositions() const;
  
  /**
   * @brief 更新关节状态
   * @param joint_positions 新的关节位置
   */
  void updateJointState(const std::vector<double>& joint_positions);
  
  /**
   * @brief 获取关节名称
   * @return 关节名称数组
   */
  const std::vector<std::string>& getJointNames() const;

private:
  std::vector<double> current_joint_positions_;
  std::vector<std::string> joint_names_;
};

} // namespace servo_wrist

#endif // SERVO_WRIST_ARM_CONTROL_H 