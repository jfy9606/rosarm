#include "trajectory/trajectory_planner.hpp"
#include <cmath>
#include <algorithm>

namespace trajectory {

TrajectoryPlanner::TrajectoryPlanner()
  : default_vel_(1.0),
    default_acc_(1.0)
{
}

trajectory_msgs::msg::JointTrajectory TrajectoryPlanner::planJointTrajectory(
  const std::vector<double>& start_positions,
  const std::vector<double>& target_positions,
  double velocity_scaling,
  double acceleration_scaling)
{
  // 检查输入参数
  if (start_positions.size() != target_positions.size()) {
    throw std::invalid_argument("Start and target positions must have the same size");
  }
  
  if (start_positions.empty()) {
    return createSimpleTrajectory(start_positions, target_positions);
  }
  
  // 计算每个关节的移动距离
  std::vector<double> distances;
  distances.reserve(start_positions.size());
  
  for (size_t i = 0; i < start_positions.size(); ++i) {
    distances.push_back(std::abs(target_positions[i] - start_positions[i]));
  }
  
  // 计算最大速度和加速度
  double max_vel = default_vel_ * velocity_scaling;
  double max_acc = default_acc_ * acceleration_scaling;
  
  // 计算每个关节的最小运动时间
  std::vector<double> joint_times;
  joint_times.reserve(distances.size());
  
  for (const auto& distance : distances) {
    joint_times.push_back(calculateMinTime(distance, max_vel, max_acc));
  }
  
  // 找出最长的运动时间
  double total_time = *std::max_element(joint_times.begin(), joint_times.end());
  
  // 如果总时间太短，设置一个最小值
  if (total_time < 0.1) {
    total_time = 0.1;
  }
  
  // 生成时间点
  std::vector<double> time_points = generateTimePoints(total_time);
  
  // 创建轨迹消息
  trajectory_msgs::msg::JointTrajectory trajectory;
  
  // 设置关节名称（这里假设关节名称未知，实际应用中应该从参数或配置中获取）
  for (size_t i = 0; i < start_positions.size(); ++i) {
    trajectory.joint_names.push_back("joint" + std::to_string(i+1));
  }
  
  // 为每个时间点生成轨迹点
  for (const auto& t : time_points) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = rclcpp::Duration::from_seconds(t);
    
    for (size_t i = 0; i < start_positions.size(); ++i) {
      double q0 = start_positions[i];
      double qf = target_positions[i];
      double qd0 = 0.0;  // 初始速度为0
      double qdf = 0.0;  // 最终速度为0
      double qdd0 = 0.0; // 初始加速度为0
      double qddf = 0.0; // 最终加速度为0
      
      auto [position, velocity, acceleration] = quinticInterpolation(
        q0, qd0, qdd0, qf, qdf, qddf, t, total_time);
      
      point.positions.push_back(position);
      point.velocities.push_back(velocity);
      point.accelerations.push_back(acceleration);
    }
    
    trajectory.points.push_back(point);
  }
  
  return trajectory;
}

trajectory_msgs::msg::JointTrajectory TrajectoryPlanner::createSimpleTrajectory(
  const std::vector<double>& start_positions,
  const std::vector<double>& target_positions)
{
  trajectory_msgs::msg::JointTrajectory trajectory;
  
  // 设置关节名称
  for (size_t i = 0; i < start_positions.size(); ++i) {
    trajectory.joint_names.push_back("joint" + std::to_string(i+1));
  }
  
  // 添加起点
  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.positions = start_positions;
  start_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
  trajectory.points.push_back(start_point);
  
  // 添加终点
  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.positions = target_positions;
  end_point.time_from_start = rclcpp::Duration::from_seconds(1.0); // 假设1秒钟完成
  trajectory.points.push_back(end_point);
  
  return trajectory;
}

double TrajectoryPlanner::calculateMinTime(
  double distance,
  double max_vel,
  double max_acc)
{
  // 计算达到最大速度所需的时间
  double t_acc = max_vel / max_acc;
  
  // 计算加速到最大速度时的距离
  double d_acc = 0.5 * max_acc * t_acc * t_acc;
  
  // 如果加速和减速的距离之和大于总距离，则无法达到最大速度
  if (2 * d_acc > distance) {
    // 计算无法达到最大速度时的最小时间
    return 2.0 * std::sqrt(distance / max_acc);
  } else {
    // 计算达到最大速度时的最小时间
    double t_const = (distance - 2 * d_acc) / max_vel;
    return 2 * t_acc + t_const;
  }
}

std::vector<double> TrajectoryPlanner::generateTimePoints(
  double total_time,
  int num_points)
{
  std::vector<double> time_points;
  time_points.reserve(num_points);
  
  double dt = total_time / (num_points - 1);
  
  for (int i = 0; i < num_points; ++i) {
    time_points.push_back(i * dt);
  }
  
  return time_points;
}

std::tuple<double, double, double> TrajectoryPlanner::quinticInterpolation(
  double q0, double qd0, double qdd0,
  double qf, double qdf, double qddf,
  double t, double T)
{
  // 归一化时间
  double tau = t / T;
  double tau2 = tau * tau;
  double tau3 = tau2 * tau;
  double tau4 = tau3 * tau;
  double tau5 = tau4 * tau;
  
  // 计算五次多项式系数
  double a0 = q0;
  double a1 = qd0 * T;
  double a2 = qdd0 * T * T / 2.0;
  
  double a3 = 10 * (qf - q0) - 6 * qd0 * T - 3 * qdd0 * T * T / 2.0 - 4 * qdf * T - qddf * T * T / 2.0;
  double a4 = -15 * (qf - q0) + 8 * qd0 * T + 3 * qdd0 * T * T / 2.0 + 7 * qdf * T + qddf * T * T / 2.0;
  double a5 = 6 * (qf - q0) - 3 * qd0 * T - qdd0 * T * T / 2.0 - 3 * qdf * T - qddf * T * T / 2.0;
  
  // 计算位置
  double position = a0 + a1 * tau + a2 * tau2 + a3 * tau3 + a4 * tau4 + a5 * tau5;
  
  // 计算速度
  double velocity = (a1 + 2 * a2 * tau + 3 * a3 * tau2 + 4 * a4 * tau3 + 5 * a5 * tau4) / T;
  
  // 计算加速度
  double acceleration = (2 * a2 + 6 * a3 * tau + 12 * a4 * tau2 + 20 * a5 * tau3) / (T * T);
  
  return {position, velocity, acceleration};
}

} // namespace trajectory