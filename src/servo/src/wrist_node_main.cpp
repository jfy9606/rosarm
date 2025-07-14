#include <rclcpp/rclcpp.hpp>
#include "servo/wrist_node.hpp"

int main(int argc, char * argv[])
{
  // 初始化ROS
  rclcpp::init(argc, argv);
  
  // 创建腕部舵机控制节点
  auto node = std::make_shared<servo_control::WristNode>();
  
  // 运行节点，直到接收到终止信号
  rclcpp::spin(node);
  
  // 清理资源并关闭
  rclcpp::shutdown();
  
  return 0;
} 