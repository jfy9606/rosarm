#ifndef TRAJECTORY_PLANNER_NODE_HPP
#define TRAJECTORY_PLANNER_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <trajectory/trajectory_planner.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory/srv/plan_trajectory.hpp>

namespace trajectory {

class TrajectoryPlannerNode : public rclcpp::Node
{
public:
  explicit TrajectoryPlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  virtual ~TrajectoryPlannerNode() = default;

private:
  // 轨迹规划器实例
  std::unique_ptr<TrajectoryPlanner> planner_;
  
  // 轨迹规划服务
  rclcpp::Service<trajectory::srv::PlanTrajectory>::SharedPtr plan_trajectory_service_;
  
  // 服务回调函数
  void handlePlanTrajectoryRequest(
    const std::shared_ptr<trajectory::srv::PlanTrajectory::Request> request,
    std::shared_ptr<trajectory::srv::PlanTrajectory::Response> response);
};

} // namespace trajectory

#endif // TRAJECTORY_PLANNER_NODE_HPP