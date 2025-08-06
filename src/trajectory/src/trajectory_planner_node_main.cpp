#include <rclcpp/rclcpp.hpp>
#include "trajectory/trajectory_planner_node.hpp"

int main(int argc, char* argv[])
{
  // 初始化ROS 2
  rclcpp::init(argc, argv);
  
  // 创建并运行轨迹规划节点
  auto node = std::make_shared<trajectory::TrajectoryPlannerNode>();
  rclcpp::spin(node);
  
  // 关闭ROS 2
  rclcpp::shutdown();
  
  return 0;
}