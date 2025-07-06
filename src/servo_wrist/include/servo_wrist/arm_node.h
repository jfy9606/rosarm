#ifndef SERVO_WRIST_ARM_NODE_H
#define SERVO_WRIST_ARM_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <servo_wrist/arm_control.h>

namespace servo_wrist {

/**
 * @brief 机械臂节点类，处理ROS通信并使用ArmControl进行核心功能
 */
class ArmNode {
public:
  /**
   * @brief 构造函数
   * @param nh ROS节点句柄
   */
  ArmNode(ros::NodeHandle& nh);
  
  /**
   * @brief 析构函数
   */
  ~ArmNode();

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
   * @brief 更新关节状态回调函数
   * @param msg 关节状态消息
   */
  void updateJointState(const sensor_msgs::JointState::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::Publisher joint_pub_;
  ros::Subscriber joint_state_sub_;
  
  // 核心控制器
  ArmControl arm_control_;
};

} // namespace servo_wrist

#endif // SERVO_WRIST_ARM_NODE_H 