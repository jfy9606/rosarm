#include <rclcpp/rclcpp.hpp>
#include "trajectory/kinematics_node.hpp"

int main(int argc, char * argv[])
{
  // 初始化ROS
  rclcpp::init(argc, argv);
  
  // 创建运动学节点
  auto node = std::make_shared<kinematics::KinematicsNode>();
  
  // 运行节点，直到接收到终止信号
  rclcpp::spin(node);
  
  // 清理资源并关闭
  rclcpp::shutdown();
  
  return 0;
} 