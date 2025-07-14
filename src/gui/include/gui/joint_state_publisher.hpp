#ifndef GUI_JOINT_STATE_PUBLISHER_HPP
#define GUI_JOINT_STATE_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <servo/srv/joint_control.hpp>
#include <mutex>
#include <vector>
#include <string>

namespace gui {

class JointStatePublisher : public rclcpp::Node {
public:
  explicit JointStatePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~JointStatePublisher();

private:
  // ROS parameters
  double publish_rate_;
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  
  // ROS publishers and subscribers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Client<servo::srv::JointControl>::SharedPtr joint_control_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Mutex for thread safety
  std::mutex joint_state_mutex_;
  
  // Callback functions
  void timerCallback();
  void updateJointStates();
};

} // namespace gui

#endif // GUI_JOINT_STATE_PUBLISHER_HPP 