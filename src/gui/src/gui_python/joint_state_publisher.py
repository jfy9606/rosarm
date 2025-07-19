#!/usr/bin/env python3

"""
关节状态发布器模块。
用于发布机器人关节状态。
"""

import threading
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


class JointStatePublisherNode(Node):
    """
    关节状态发布器节点类。
    负责发布机器人关节状态信息。
    """
    
    def __init__(self):
        """初始化节点。"""
        super().__init__('joint_state_publisher')
        
        # 声明并获取参数
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('joint_names', 
                             ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper'])
        
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        
        # 如果joint_names参数为空，使用默认值
        if not self.joint_names:
            self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper']
        
        # 初始化关节状态
        self.num_joints = len(self.joint_names)
        self.joint_positions = [0.0] * self.num_joints
        self.joint_velocities = [0.0] * self.num_joints
        self.joint_efforts = [0.0] * self.num_joints
        
        # 创建发布器
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10)
        
        # 创建定时器
        timer_period = 1.0 / self.publish_rate  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 创建锁以确保线程安全
        self._lock = threading.Lock()
        
        self.get_logger().info(f'Joint State Publisher initialized with {self.num_joints} joints')
    
    def timer_callback(self):
        """定时器回调函数，定期发布关节状态。"""
        # 更新关节状态
        self.update_joint_states()
        
        # 创建并发布关节状态消息
        joint_state_msg = JointState()
        
        # 设置头
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 获取锁以确保线程安全
        with self._lock:
            # 设置关节状态
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = self.joint_positions
            joint_state_msg.velocity = self.joint_velocities
            joint_state_msg.effort = self.joint_efforts
        
        # 发布消息
        self.joint_state_pub.publish(joint_state_msg)
    
    def update_joint_states(self):
        """更新关节状态，现在使用模拟数据。"""
        # 这个方法会查询实际机器人的关节状态
        # 现在我们只使用占位符值
        
        # 获取锁以确保线程安全
        with self._lock:
            # 使用占位符值更新（将替换为实际机器人数据）
            current_time = time.time()
            for i in range(len(self.joint_positions)):
                # 模拟一些运动
                self.joint_positions[i] = 0.1 * math.sin(current_time + i)
                self.joint_velocities[i] = 0.1 * math.cos(current_time + i)
                self.joint_efforts[i] = 0.0 