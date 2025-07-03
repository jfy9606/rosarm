#ifndef SERVO_WRIST_ARM_CONTROLLER_H
#define SERVO_WRIST_ARM_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

namespace servo_wrist {

class ArmController {
public:
  ArmController(ros::NodeHandle& nh);
  ~ArmController();

  void setJointPositions(const std::vector<double>& positions);
  std::vector<double> getJointPositions() const;
  void updateJointState(const sensor_msgs::JointState::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::Publisher joint_pub_;
  ros::Subscriber joint_state_sub_;
  std::vector<double> current_joint_positions_;
  std::vector<std::string> joint_names_;
};

} // namespace servo_wrist

#endif // SERVO_WRIST_ARM_CONTROLLER_H
