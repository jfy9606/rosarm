#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from trajectory.srv import PlanTrajectory
from geometry_msgs.msg import Pose

class TrajectoryPlanner:
    """轨迹规划器类"""
    
    def __init__(self):
        """初始化轨迹规划器"""
        # 时间参数
        self.default_vel = 0.5  # 默认速度（弧度/秒）
        self.default_acc = 0.2  # 默认加速度（弧度/秒²）
        
    def plan_joint_trajectory(self, start_positions, target_positions, 
                             velocity_scaling=1.0, acceleration_scaling=1.0):
        """
        规划关节空间轨迹
        
        Args:
            start_positions (list): 起始关节位置列表
            target_positions (list): 目标关节位置列表
            velocity_scaling (float): 速度缩放因子
            acceleration_scaling (float): 加速度缩放因子
            
        Returns:
            JointTrajectory: 关节轨迹消息
        """
        if len(start_positions) != len(target_positions):
            raise ValueError("起始位置和目标位置的关节数量不一致")
            
        # 计算每个关节的移动距离
        distances = [abs(target - start) for start, target in zip(start_positions, target_positions)]
        
        # 找到最大位移的关节
        max_distance = max(distances)
        if max_distance < 1e-6:  # 如果位移很小，不需要规划轨迹
            return self._create_simple_trajectory(start_positions, target_positions)
        
        # 计算轨迹总时间（基于最大位移关节）
        velocity = self.default_vel * velocity_scaling
        acceleration = self.default_acc * acceleration_scaling
        total_time = self._calculate_min_time(max_distance, velocity, acceleration)
        
        # 创建轨迹时间点
        times = self._generate_time_points(total_time)
        
        # 为每个时间点创建轨迹点
        trajectory = JointTrajectory()
        trajectory.joint_names = [f"joint_{i+1}" for i in range(len(start_positions))]
        
        for t in times:
            # 计算各个关节在时间t的位置、速度、加速度
            point = JointTrajectoryPoint()
            point.time_from_start.sec = int(t)
            point.time_from_start.nanosec = int((t - int(t)) * 1e9)
            
            positions = []
            velocities = []
            accelerations = []
            
            for i in range(len(start_positions)):
                # 使用五次多项式插值
                p, v, a = self._quintic_interpolation(
                    start_positions[i], 0.0, 0.0,  # 起始位置、速度、加速度
                    target_positions[i], 0.0, 0.0,  # 目标位置、速度、加速度
                    t, total_time
                )
                positions.append(p)
                velocities.append(v)
                accelerations.append(a)
            
            point.positions = positions
            point.velocities = velocities
            point.accelerations = accelerations
            trajectory.points.append(point)
        
        return trajectory
    
    def _create_simple_trajectory(self, start_positions, target_positions):
        """创建简单轨迹（起点和终点）"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [f"joint_{i+1}" for i in range(len(start_positions))]
        
        # 起点
        start_point = JointTrajectoryPoint()
        start_point.positions = start_positions
        start_point.velocities = [0.0] * len(start_positions)
        start_point.accelerations = [0.0] * len(start_positions)
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0
        
        # 终点
        end_point = JointTrajectoryPoint()
        end_point.positions = target_positions
        end_point.velocities = [0.0] * len(target_positions)
        end_point.accelerations = [0.0] * len(target_positions)
        end_point.time_from_start.sec = 1
        end_point.time_from_start.nanosec = 0
        
        trajectory.points = [start_point, end_point]
        return trajectory
    
    def _calculate_min_time(self, distance, max_vel, max_acc):
        """计算最小运动时间"""
        # 计算达到最大速度所需的时间和距离
        t_acc = max_vel / max_acc
        d_acc = 0.5 * max_acc * t_acc ** 2
        
        if 2 * d_acc <= distance:
            # 有匀速段的情况
            d_cruise = distance - 2 * d_acc
            t_cruise = d_cruise / max_vel
            total_time = 2 * t_acc + t_cruise
        else:
            # 无匀速段的情况
            t_acc = np.sqrt(distance / max_acc)
            total_time = 2 * t_acc
            
        return total_time
    
    def _generate_time_points(self, total_time, num_points=50):
        """生成轨迹时间点"""
        return np.linspace(0, total_time, num_points)
    
    def _quintic_interpolation(self, q0, qd0, qdd0, qf, qdf, qddf, t, T):
        """五次多项式插值"""
        if t < 0:
            return q0, qd0, qdd0
        elif t > T:
            return qf, qdf, qddf
        
        # 五次多项式插值系数
        t0 = q0
        t1 = qd0
        t2 = 0.5 * qdd0
        t3 = (20 * qf - 20 * q0 - (8 * qdf + 12 * qd0) * T - (3 * qdd0 - qddf) * T * T) / (2 * T * T * T)
        t4 = (30 * q0 - 30 * qf + (14 * qdf + 16 * qd0) * T + (3 * qdd0 - 2 * qddf) * T * T) / (2 * T * T * T * T)
        t5 = (12 * qf - 12 * q0 - (6 * qdf + 6 * qd0) * T - (qdd0 - qddf) * T * T) / (2 * T * T * T * T * T)
        
        # 计算位置、速度和加速度
        tnow = t / T
        position = t0 + t1 * tnow + t2 * tnow * tnow + t3 * tnow * tnow * tnow + t4 * tnow * tnow * tnow * tnow + t5 * tnow * tnow * tnow * tnow * tnow
        velocity = (t1 + 2 * t2 * tnow + 3 * t3 * tnow * tnow + 4 * t4 * tnow * tnow * tnow + 5 * t5 * tnow * tnow * tnow * tnow) / T
        acceleration = (2 * t2 + 6 * t3 * tnow + 12 * t4 * tnow * tnow + 20 * t5 * tnow * tnow * tnow) / (T * T)
        
        return position, velocity, acceleration

class TrajectoryPlannerNode(Node):
    """轨迹规划节点"""
    
    def __init__(self):
        super().__init__('trajectory_planner_node')
        
        # 创建轨迹规划器
        self.planner = TrajectoryPlanner()
        
        # 创建轨迹规划服务
        self.srv = self.create_service(
            PlanTrajectory,
            'plan_trajectory',
            self.handle_plan_trajectory
        )
        
        self.get_logger().info("轨迹规划节点已启动")
    
    def handle_plan_trajectory(self, request, response):
        """处理轨迹规划请求"""
        try:
            # 获取请求参数
            start_positions = list(request.start_positions)
            target_positions = list(request.target_positions)
            velocity_scaling = request.velocity_scaling
            acceleration_scaling = request.acceleration_scaling
            
            # 规划轨迹
            trajectory = self.planner.plan_joint_trajectory(
                start_positions,
                target_positions,
                velocity_scaling,
                acceleration_scaling
            )
            
            # 设置响应
            response.trajectory = trajectory
            response.success = True
            response.message = "轨迹规划成功"
            
            self.get_logger().info("轨迹规划成功")
        except Exception as e:
            self.get_logger().error(f"轨迹规划失败：{str(e)}")
            response.success = False
            response.message = f"轨迹规划失败：{str(e)}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 