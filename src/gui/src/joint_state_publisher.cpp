#include "gui/joint_state_publisher.hpp"
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace gui {

JointStatePublisher::JointStatePublisher(const rclcpp::NodeOptions & options)
: Node("joint_state_publisher", options)
{
  // Declare and get parameters
  this->declare_parameter("publish_rate", 10.0);
  this->declare_parameter("joint_names", std::vector<std::string>{"joint1", "joint2", "joint3", "joint4", "joint5", "gripper"});
  
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  joint_names_ = this->get_parameter("joint_names").as_string_array();
  
  // Initialize joint states
  const size_t num_joints = joint_names_.size();
  joint_positions_.resize(num_joints, 0.0);
  joint_velocities_.resize(num_joints, 0.0);
  joint_efforts_.resize(num_joints, 0.0);
  
  // Create publisher
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_states", 10);
    
  // Create mock service client
  joint_control_client_ = std::make_shared<mock::JointControlClient>();
    
  // Create timer for publishing joint states
  auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
  timer_ = this->create_wall_timer(
    timer_period, std::bind(&JointStatePublisher::timerCallback, this));
    
  RCLCPP_INFO(this->get_logger(), "Joint State Publisher initialized with %zu joints", num_joints);
}

JointStatePublisher::~JointStatePublisher()
{
  // Clean up resources if needed
}

void JointStatePublisher::timerCallback()
{
  // Update joint states from actual robot
  updateJointStates();
  
  // Create and publish joint state message
  auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
  
  // Set header
  joint_state_msg->header.stamp = this->now();
  
  // Lock mutex for thread safety
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  
  // Set joint states
  joint_state_msg->name = joint_names_;
  joint_state_msg->position = joint_positions_;
  joint_state_msg->velocity = joint_velocities_;
  joint_state_msg->effort = joint_efforts_;
  
  // Publish message
  joint_state_pub_->publish(std::move(joint_state_msg));
}

void JointStatePublisher::updateJointStates()
{
  // This method would query the actual robot for joint states
  // For now, we'll just use placeholder values
  
  // Lock mutex for thread safety
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  
  // Update with placeholder values (would be replaced with actual robot data)
  for (size_t i = 0; i < joint_positions_.size(); ++i) {
    // Simulate some movement
    joint_positions_[i] = 0.1 * sin(this->now().seconds() + i);
    joint_velocities_[i] = 0.1 * cos(this->now().seconds() + i);
    joint_efforts_[i] = 0.0;
  }
}

} // namespace gui

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gui::JointStatePublisher) 