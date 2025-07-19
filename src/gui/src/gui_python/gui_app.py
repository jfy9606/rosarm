#!/usr/bin/env python3

"""
机器人控制图形用户界面应用类。
"""

import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from .control_panel import ControlPanel


class RobotArmGUI:
    """机器人控制GUI应用类。"""
    
    def __init__(self, node):
        """
        初始化GUI应用。
        
        Args:
            node: ROS节点。
        """
        self.node = node
        self.root = tk.Tk()
        self.root.title("ROS 2 Robotic Arm Control")
        self.root.geometry("800x600")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 创建主框架
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建控制面板
        self.control_panel = ControlPanel(self.main_frame, node)
        self.control_panel.pack(fill=tk.BOTH, expand=True)
        
        # 创建状态栏
        self.status_var = tk.StringVar(value="Ready")
        self.status_bar = ttk.Label(self.root, textvariable=self.status_var, 
                                   relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # 设置ROS发布者和订阅者
        self.command_pub = node.create_publisher(String, 'arm_command', 10)
        self.joint_state_sub = node.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        
        # 启动定时器，定期更新机器人状态
        self.root.after(100, self.update_robot_status)
        
        self.node.get_logger().info("GUI Main Window initialized")
    
    def joint_state_callback(self, msg):
        """
        关节状态回调函数。
        
        Args:
            msg: 关节状态消息。
        """
        # 处理关节状态更新，可以将关节位置显示在界面上
        pass
    
    def update_robot_status(self):
        """定期更新机器人状态。"""
        # 这里可以添加代码来刷新UI和处理ROS消息
        
        # 重新安排下一次更新
        self.root.after(100, self.update_robot_status)
    
    def on_closing(self):
        """窗口关闭事件处理。"""
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.root.destroy()
    
    def run(self):
        """运行GUI应用。"""
        self.root.mainloop()


def create_and_run(args=None):
    """
    创建并运行GUI应用。
    
    Args:
        args: 命令行参数。
    """
    # 初始化ROS
    rclpy.init(args=args)
    node = Node("gui_node")
    
    # 创建线程运行ROS回调
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()
    
    try:
        # 创建并运行GUI
        app = RobotArmGUI(node)
        app.run()
    except Exception as e:
        node.get_logger().error(f"Error in GUI: {str(e)}")
    finally:
        # 清理
        node.destroy_node()
        rclpy.shutdown()
        
        # 等待线程结束
        if ros_thread.is_alive():
            ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    create_and_run() 