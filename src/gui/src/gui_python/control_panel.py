#!/usr/bin/env python3

"""
控制面板模块，提供机器人关节控制界面。
"""

import tkinter as tk
from tkinter import ttk
import math
from rclpy.node import Node
from . import mock_services


class ControlPanel(ttk.Frame):
    """
    控制面板类，包含关节滑动条和控制按钮。
    """
    
    def __init__(self, parent, node: Node):
        """
        初始化控制面板。
        
        Args:
            parent: 父级Tkinter容器。
            node: ROS节点。
        """
        super().__init__(parent)
        self.node = node
        
        # 初始化关节名称
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper"]
        
        # 创建模拟服务客户端
        self.joint_control_client = mock_services.JointControlClient()
        self.vacuum_client = mock_services.VacuumCmdClient()
        
        # 设置UI
        self.setup_ui()
        
        # 模拟服务始终可用
        self.node.get_logger().info("Using mock services for control")
    
    def setup_ui(self):
        """设置UI组件。"""
        # 创建关节控制组
        joint_group = ttk.LabelFrame(self, text="Joint Control")
        joint_group.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 创建滑动条和标签
        self.joint_sliders = []
        self.joint_value_labels = []
        
        for i, name in enumerate(self.joint_names):
            # 创建行框架
            row_frame = ttk.Frame(joint_group)
            row_frame.pack(fill=tk.X, padx=5, pady=2)
            
            # 关节名称标签
            name_label = ttk.Label(row_frame, text=name, width=10)
            name_label.pack(side=tk.LEFT)
            
            # 关节值显示标签
            value_label = ttk.Label(row_frame, text="0.0°", width=10)
            value_label.pack(side=tk.RIGHT)
            self.joint_value_labels.append(value_label)
            
            # 滑动条
            slider = ttk.Scale(row_frame, from_=-90, to=90, orient=tk.HORIZONTAL)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            slider.bind("<ButtonRelease-1>", lambda event, idx=i: self.on_slider_changed(event, idx))
            self.joint_sliders.append(slider)
        
        # 创建按钮行
        button_frame = ttk.Frame(self)
        button_frame.pack(fill=tk.X, pady=10)
        
        # 回到起始位置按钮
        self.home_button = ttk.Button(button_frame, text="Home Position", 
                                     command=self.on_home_clicked)
        self.home_button.pack(side=tk.LEFT, padx=5)
        
        # 夹持器按钮
        self.gripper_button = ttk.Button(button_frame, text="Toggle Gripper", 
                                        command=self.on_gripper_clicked)
        self.gripper_button.pack(side=tk.LEFT, padx=5)
    
    def update_joint_label(self, joint_index, value):
        """
        更新关节值标签。
        
        Args:
            joint_index: 关节索引。
            value: 关节值。
        """
        if 0 <= joint_index < len(self.joint_value_labels):
            # 转换滑动条值为角度
            degrees = float(value)
            self.joint_value_labels[joint_index].config(text=f"{degrees:.1f}°")
    
    def on_slider_changed(self, event, joint_index):
        """
        滑动条值变化时的回调函数。
        
        Args:
            event: 事件对象。
            joint_index: 关节索引。
        """
        # 获取滑动条值
        slider = self.joint_sliders[joint_index]
        value = slider.get()
        
        # 更新关节值标签
        self.update_joint_label(joint_index, value)
        
        # 将角度转换为弧度并发送命令
        position = value * math.pi / 180.0  # 转换角度为弧度
        self.send_joint_command(joint_index, position)
    
    def send_joint_command(self, joint_index, position):
        """
        发送关节控制命令。
        
        Args:
            joint_index: 关节索引。
            position: 目标位置（弧度）。
        """
        if not (0 <= joint_index < len(self.joint_names)):
            return
        
        # 创建模拟请求
        request = {'position': [0.0] * len(self.joint_names)}
        
        # 设置所有关节的位置，但只改变选定的关节
        request['position'][joint_index] = position
        
        # 发送模拟请求
        event = self.joint_control_client.call_async(request)
        
        # 添加回调处理响应
        def on_response():
            event.wait()
            result = self.joint_control_client.response
            if result.success:
                self.node.get_logger().info(f"Joint {joint_index} moved successfully: {result.message}")
            else:
                self.node.get_logger().error(f"Failed to move joint {joint_index}: {result.message}")
        
        # 启动线程处理响应
        import threading
        threading.Thread(target=on_response, daemon=True).start()
    
    def on_home_clicked(self):
        """回到起始位置按钮点击事件处理。"""
        self.node.get_logger().info("Moving to home position")
        
        # 将所有滑动条设置为0
        for i, slider in enumerate(self.joint_sliders):
            slider.set(0)
            self.update_joint_label(i, 0)
        
        # 家位置是通过将所有滑动条设置为0来处理的
    
    def on_gripper_clicked(self):
        """夹持器按钮点击事件处理。"""
        self.node.get_logger().info("Toggling gripper")
        
        # 创建模拟请求
        request = {'enable': True}  # 根据当前状态切换
        
        # 发送模拟请求
        event = self.vacuum_client.call_async(request)
        
        # 添加回调处理响应
        def on_response():
            event.wait()
            result = self.vacuum_client.response
            if result.success:
                self.node.get_logger().info("Gripper toggled successfully")
            else:
                self.node.get_logger().error(f"Failed to toggle gripper: {result.message}")
        
        # 启动线程处理响应
        import threading
        threading.Thread(target=on_response, daemon=True).start() 