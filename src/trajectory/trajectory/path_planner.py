#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose, Point, Vector3
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

class PathPlanner:
    """路径规划器类，用于生成点到点的路径"""
    
    def __init__(self):
        """初始化路径规划器"""
        # 路径规划参数
        self.step_size = 0.01  # 直线步长(m)
        self.angle_step = 0.05  # 角度步长(rad)
        
    def plan_linear_path(self, start_pose, target_pose, num_points=None):
        """
        规划笛卡尔空间的直线路径
        
        Args:
            start_pose (Pose): 起始位姿
            target_pose (Pose): 目标位姿
            num_points (int, optional): 路径点数量，如果为None则基于步长计算
            
        Returns:
            list: 路径点列表，每个点是一个Pose
        """
        # 计算起始点和目标点之间的直线距离
        start_pos = np.array([start_pose.position.x, 
                              start_pose.position.y, 
                              start_pose.position.z])
        target_pos = np.array([target_pose.position.x, 
                               target_pose.position.y, 
                               target_pose.position.z])
        
        distance = np.linalg.norm(target_pos - start_pos)
        
        # 如果没有指定点数，则基于步长计算
        if num_points is None:
            num_points = max(int(distance / self.step_size), 2)
        
        # 创建路径点
        path = []
        for i in range(num_points):
            t = i / (num_points - 1)  # 插值参数 [0, 1]
            
            # 位置线性插值
            pos_x = start_pose.position.x + t * (target_pose.position.x - start_pose.position.x)
            pos_y = start_pose.position.y + t * (target_pose.position.y - start_pose.position.y)
            pos_z = start_pose.position.z + t * (target_pose.position.z - start_pose.position.z)
            
            # 姿态球面线性插值(SLERP)
            start_quat = np.array([start_pose.orientation.w,
                                  start_pose.orientation.x,
                                  start_pose.orientation.y,
                                  start_pose.orientation.z])
            target_quat = np.array([target_pose.orientation.w,
                                   target_pose.orientation.x,
                                   target_pose.orientation.y,
                                   target_pose.orientation.z])
            
            interp_quat = self._slerp(start_quat, target_quat, t)
            
            # 创建插值位姿
            pose = Pose()
            pose.position.x = pos_x
            pose.position.y = pos_y
            pose.position.z = pos_z
            pose.orientation.w = interp_quat[0]
            pose.orientation.x = interp_quat[1]
            pose.orientation.y = interp_quat[2]
            pose.orientation.z = interp_quat[3]
            
            path.append(pose)
        
        return path
    
    def plan_circular_path(self, center_point, normal_vector, radius, start_angle, end_angle):
        """
        规划圆弧路径
        
        Args:
            center_point (Point): 圆心点
            normal_vector (Vector3): 圆平面法向量
            radius (float): 圆半径
            start_angle (float): 起始角度(弧度)
            end_angle (float): 结束角度(弧度)
            
        Returns:
            list: 路径点列表，每个点是一个Pose
        """
        # 标准化法向量
        normal = np.array([normal_vector.x, normal_vector.y, normal_vector.z])
        normal = normal / np.linalg.norm(normal)
        
        # 计算圆平面的两个正交基向量
        if abs(normal[2]) < 0.9:
            # 如果法向量不是接近z轴，使用z轴叉乘法向量得到第一个基向量
            base1 = np.cross(np.array([0, 0, 1]), normal)
        else:
            # 如果法向量接近z轴，使用x轴叉乘法向量
            base1 = np.cross(np.array([1, 0, 0]), normal)
            
        base1 = base1 / np.linalg.norm(base1)
        base2 = np.cross(normal, base1)  # 第二个基向量
        
        # 确定角度范围和步长
        if end_angle < start_angle:
            end_angle += 2 * np.pi
            
        angle_diff = end_angle - start_angle
        num_steps = max(int(angle_diff / self.angle_step), 2)
        
        # 创建路径点
        path = []
        for i in range(num_steps):
            t = i / (num_steps - 1)
            angle = start_angle + t * angle_diff
            
            # 计算圆上点的位置
            local_point = radius * (base1 * np.cos(angle) + base2 * np.sin(angle))
            global_point = np.array([center_point.x, center_point.y, center_point.z]) + local_point
            
            # 创建位姿(姿态设为z轴朝向圆心)
            pose = Pose()
            pose.position.x = global_point[0]
            pose.position.y = global_point[1]
            pose.position.z = global_point[2]
            
            # 计算朝向（z轴指向圆心的反方向）
            z_axis = -local_point / np.linalg.norm(local_point)
            x_axis = np.cross(base2, z_axis)  # 使x轴在圆平面上
            x_axis = x_axis / np.linalg.norm(x_axis)
            y_axis = np.cross(z_axis, x_axis)
            
            # 从旋转矩阵计算四元数
            quat = self._rotation_matrix_to_quaternion(np.column_stack((x_axis, y_axis, z_axis)))
            
            pose.orientation.w = quat[0]
            pose.orientation.x = quat[1]
            pose.orientation.y = quat[2]
            pose.orientation.z = quat[3]
            
            path.append(pose)
            
        return path
    
    def _slerp(self, q1, q2, t):
        """四元数球面线性插值"""
        # 确保是单位四元数
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)
        
        # 计算四元数点积
        dot = np.sum(q1 * q2)
        
        # 如果点积为负，取负号使插值走最短路径
        if dot < 0.0:
            q2 = -q2
            dot = -dot
            
        # 截断，避免浮点误差
        dot = min(1.0, max(-1.0, dot))
        
        # 计算插值角度
        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)
        
        # 如果角度接近0，使用线性插值
        if sin_theta_0 < 1e-10:
            return (1.0 - t) * q1 + t * q2
            
        # 球面线性插值
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        
        return s0 * q1 + s1 * q2
    
    def _rotation_matrix_to_quaternion(self, R):
        """从旋转矩阵计算四元数"""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
            
        return np.array([qw, qx, qy, qz])

class PathPlannerNode(Node):
    """路径规划节点"""
    
    def __init__(self):
        super().__init__('path_planner_node')
        
        # 创建路径规划器
        self.planner = PathPlanner()
        
        # 创建发布器
        self.path_pub = self.create_publisher(
            JointTrajectory,
            'planned_path',
            10
        )
        
        # 创建订阅者，监听目标位姿
        self.pose_sub = self.create_subscription(
            Pose,
            'target_pose',
            self.handle_target_pose,
            10
        )
        
        # 创建订阅者，监听当前关节状态
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.handle_joint_state,
            10
        )
        
        # 存储最新的关节状态
        self.latest_joint_state = None
        
        self.get_logger().info("路径规划节点已启动")
    
    def handle_target_pose(self, msg):
        """处理目标位姿消息"""
        self.get_logger().info("收到目标位姿")
        # 实际应用中，这里需要调用逆运动学和轨迹规划
        # 这里是个简化的示例
        pass
    
    def handle_joint_state(self, msg):
        """处理关节状态消息"""
        self.latest_joint_state = msg

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 