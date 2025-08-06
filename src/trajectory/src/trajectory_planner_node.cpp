#include "trajectory/trajectory_planner_node.hpp"

namespace trajectory {

TrajectoryPlannerNode::TrajectoryPlannerNode(const rclcpp::NodeOptions& options)
  : Node("trajectory_planner_node", options)
{
  // 创建轨迹规划器实例
  planner_ = std::make_unique<TrajectoryPlanner>();
  
  // 创建轨迹规划服务
  plan_trajectory_service_ = create_service<trajectory::srv::PlanTrajectory>(
    "plan_trajectory",
    std::bind(&TrajectoryPlannerNode::handlePlanTrajectoryRequest, this,
              std::placeholders::_1, std::placeholders::_2));
              
  RCLCPP_INFO(get_logger(), "Trajectory planner node initialized");
}

void TrajectoryPlannerNode::handlePlanTrajectoryRequest(
  const std::shared_ptr<trajectory::srv::PlanTrajectory::Request> request,
  std::shared_ptr<trajectory::srv::PlanTrajectory::Response> response)
{
  RCLCPP_INFO(get_logger(), "Received trajectory planning request");
  
  try {
    // 提取起始和目标关节位置
    std::vector<double> start_positions;
    std::vector<double> target_positions;
    
    // 从请求中获取起始位置
    if (!request->start_pose.position.empty()) {
      for (const auto& pos : request->start_pose.position) {
        start_positions.push_back(pos);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Start positions are empty");
      response->success = false;
      response->message = "Start positions are empty";
      return;
    }
    
    // 从请求中获取目标位置
    if (!request->target_pose.position.empty()) {
      for (const auto& pos : request->target_pose.position) {
        target_positions.push_back(pos);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Target positions are empty");
      response->success = false;
      response->message = "Target positions are empty";
      return;
    }
    
    // 检查起始和目标位置的维度是否一致
    if (start_positions.size() != target_positions.size()) {
      RCLCPP_ERROR(get_logger(), "Start and target positions have different dimensions");
      response->success = false;
      response->message = "Start and target positions have different dimensions";
      return;
    }
    
    // 获取速度和加速度缩放因子
    double velocity_scaling = 1.0;
    double acceleration_scaling = 1.0;
    
    // 如果请求中指定了执行时间，则调整速度和加速度缩放因子
    if (request->execution_time > 0.0) {
      // 计算默认轨迹的时间
      auto default_trajectory = planner_->planJointTrajectory(
        start_positions, target_positions, 1.0, 1.0);
      
      if (!default_trajectory.points.empty()) {
        double default_time = default_trajectory.points.back().time_from_start.seconds();
        
        // 调整速度和加速度缩放因子以匹配请求的执行时间
        if (default_time > 0.0) {
          double time_ratio = default_time / request->execution_time;
          velocity_scaling = time_ratio;
          acceleration_scaling = time_ratio * time_ratio;
        }
      }
    }
    
    // 规划轨迹
    auto trajectory = planner_->planJointTrajectory(
      start_positions, target_positions, velocity_scaling, acceleration_scaling);
    
    // 设置响应
    response->success = true;
    response->message = "Trajectory planning successful";
    
    // 将JointTrajectory转换为JointState数组
    for (const auto& point : trajectory.points) {
      sensor_msgs::msg::JointState joint_state;
      joint_state.header.stamp = this->now();
      joint_state.name = trajectory.joint_names;
      joint_state.position = point.positions;
      joint_state.velocity = point.velocities;
      joint_state.effort.resize(point.positions.size(), 0.0); // 默认力矩为0
      
      response->trajectory.push_back(joint_state);
    }
    
    RCLCPP_INFO(get_logger(), "Trajectory planning successful");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Trajectory planning failed: %s", e.what());
    response->success = false;
    response->message = "Trajectory planning failed: " + std::string(e.what());
  }
}

} // namespace trajectory

#include "rclcpp_components/register_node_macro.hpp"

// 注册节点组件
RCLCPP_COMPONENTS_REGISTER_NODE(trajectory::TrajectoryPlannerNode)