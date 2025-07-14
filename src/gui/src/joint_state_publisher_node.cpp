#include <rclcpp/rclcpp.hpp>
#include "gui/joint_state_publisher.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<gui::JointStatePublisher>());
  
  rclcpp::shutdown();
  return 0;
} 