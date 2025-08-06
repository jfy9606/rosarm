#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace trajectory {

class TrajectoryPlanner
{
public:
  TrajectoryPlanner();
  virtual ~TrajectoryPlanner() = default;
  
  // 规划关节空间轨迹
  trajectory_msgs::msg::JointTrajectory planJointTrajectory(
    const std::vector<double>& start_positions,
    const std::vector<double>& target_positions,
    double velocity_scaling = 1.0,
    double acceleration_scaling = 1.0);
    
private:
  // 时间参数
  double default_vel_;
  double default_acc_;
  
  // 创建简单轨迹（起点和终点）
  trajectory_msgs::msg::JointTrajectory createSimpleTrajectory(
    const std::vector<double>& start_positions,
    const std::vector<double>& target_positions);
    
  // 计算最小运动时间
  double calculateMinTime(
    double distance,
    double max_vel,
    double max_acc);
    
  // 生成轨迹时间点
  std::vector<double> generateTimePoints(
    double total_time,
    int num_points = 50);
    
  // 五次多项式插值
  std::tuple<double, double, double> quinticInterpolation(
    double q0, double qd0, double qdd0,
    double qf, double qdf, double qddf,
    double t, double T);
};

} // namespace trajectory

#endif // TRAJECTORY_PLANNER_HPP