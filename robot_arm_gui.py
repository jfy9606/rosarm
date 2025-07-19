#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
机械臂控制GUI
支持小臂舵机和大臂电机的单独连接和控制
集成OpenCV视觉功能
"""

import sys
import os
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import serial.tools.list_ports
import threading
import queue
import cv2
from PIL import Image, ImageTk
import numpy as np # Added for numpy

# 添加src目录到路径
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

# 导入RobotArm类
from src.robot_arm import RobotArm


class AsyncTask(threading.Thread):
    """异步任务线程基类"""
    def __init__(self, callback=None):
        super().__init__()
        self.callback = callback
        self.daemon = True  # 设置为守护线程，随主线程退出
        self.result = None
        self.error = None
        
    def run(self):
        try:
            self.result = self.task()
            if self.callback:
                self.callback(self.result)
        except Exception as e:
            self.error = str(e)
            if self.callback:
                self.callback(None, error=str(e))
                
    def task(self):
        """子类需要重写此方法"""
        raise NotImplementedError()


class PortScanTask(AsyncTask):
    """串口扫描任务"""
    def task(self):
        """执行串口扫描"""
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            return ports
        except Exception as e:
            raise Exception(f"扫描串口时出错: {e}")


class ServoScanTask(AsyncTask):
    """舵机扫描任务"""
    def __init__(self, robot, start_id, end_id, callback=None):
        super().__init__(callback)
        self.robot = robot
        self.start_id = start_id
        self.end_id = end_id
        
    def task(self):
        """执行舵机扫描"""
        try:
            servos = self.robot.scan_servos(self.start_id, self.end_id)
            return servos
        except Exception as e:
            raise Exception(f"扫描舵机时出错: {e}")


class VideoCapture:
    """视频捕获类，用于处理OpenCV视频流"""
    def __init__(self, camera_index=0):
        self.camera_index = camera_index
        self.cap = None
        self.running = False
        self.frame_queue = queue.Queue(maxsize=1)
        self.thread = None
        
    def start(self):
        """启动视频捕获"""
        if self.running:
            return True
            
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                return False
                
            self.running = True
            self.thread = threading.Thread(target=self._capture_loop)
            self.thread.daemon = True
            self.thread.start()
            return True
        except Exception:
            self.stop()
            return False
            
    def stop(self):
        """停止视频捕获"""
        self.running = False
        if self.thread:
            if self.thread.is_alive():
                self.thread.join(timeout=1.0)
            self.thread = None
            
        if self.cap:
            self.cap.release()
            self.cap = None
            
    def _capture_loop(self):
        """视频捕获循环"""
        while self.running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break
                
            # 如果队列已满，先清空队列
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except queue.Empty:
                    pass
                    
            # 将新帧放入队列
            try:
                self.frame_queue.put(frame, block=False)
            except queue.Full:
                pass
                
        self.running = False
        
    def get_frame(self):
        """获取当前帧"""
        if not self.running:
            return None
            
        try:
            return self.frame_queue.get(block=False)
        except queue.Empty:
            return None
            
    def is_running(self):
        """检查捕获是否正在运行"""
        return self.running 


class RobotArmGUI:
    """
    机械臂控制GUI类
    允许用户选择性连接小臂舵机或大臂电机，并单独控制各个部分
    集成OpenCV视觉功能
    """
    
    def __init__(self, root):
        """
        初始化GUI
        
        Args:
            root: tkinter根窗口
        """
        self.root = root
        self.root.title("机械臂控制面板")
        self.root.geometry("1000x700")  # 增大窗口以容纳视觉功能
        
        # 创建RobotArm实例
        self.robot = RobotArm()
        
        # 连接状态
        self.servo_connected = False
        self.motor_connected = False
        
        # 视频捕获
        self.video = VideoCapture(0)  # 默认使用摄像头0
        self.video_enabled = False
        self.video_update_ms = 50  # 视频更新间隔(毫秒)
        self.video_frame = None
        
        # 视觉处理参数
        self.vision_processing_enabled = False
        self.vision_params = {
            'hue_low': 0,
            'hue_high': 179,
            'sat_low': 0,
            'sat_high': 255,
            'val_low': 0,
            'val_high': 255
        }
        
        # 舵机信息
        self.servos = []
        
        # 是否在更新状态
        self.is_updating_status = False
        
        # 电机控制器类型
        self.motor_controller_type = "standard"
        
        # 调试模式
        self.debug_mode = False
        
        # 创建GUI组件
        self.create_widgets()
        
        # 创建选项卡式界面
        self.create_notebook()
        
        # 更新端口列表
        self.update_port_list()
        
        # 定时刷新状态
        self.root.after(1000, self.update_status)
    
    def create_widgets(self):
        """创建GUI主要组件"""
        # 创建顶部导航栏
        self.create_menu()
        
        # 创建选项卡窗口
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 底部状态栏
        self.status_bar = ttk.Label(self.root, text="就绪", relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
    
    def create_menu(self):
        """创建菜单栏"""
        menu_bar = tk.Menu(self.root)
        
        # 文件菜单
        file_menu = tk.Menu(menu_bar, tearoff=0)
        file_menu.add_command(label="退出", command=self.on_exit)
        menu_bar.add_cascade(label="文件", menu=file_menu)
        
        # 连接菜单
        conn_menu = tk.Menu(menu_bar, tearoff=0)
        conn_menu.add_command(label="连接舵机", command=self.connect_servo)
        conn_menu.add_command(label="连接电机", command=self.connect_motor)
        conn_menu.add_command(label="断开连接", command=self.disconnect)
        conn_menu.add_separator()
        conn_menu.add_command(label="刷新端口", command=self.update_port_list)
        menu_bar.add_cascade(label="连接", menu=conn_menu)
        
        # 控制菜单
        ctrl_menu = tk.Menu(menu_bar, tearoff=0)
        ctrl_menu.add_command(label="回到初始位置", command=self.home_robot)
        ctrl_menu.add_command(label="停止所有电机", command=self.emergency_stop)
        ctrl_menu.add_command(label="校准机械臂", command=self.calibrate_robot)
        menu_bar.add_cascade(label="控制", menu=ctrl_menu)
        
        # 视觉菜单
        vision_menu = tk.Menu(menu_bar, tearoff=0)
        vision_menu.add_command(label="开启摄像头", command=self.toggle_camera)
        vision_menu.add_command(label="视觉参数设置", command=self.show_vision_settings)
        vision_menu.add_separator()
        vision_menu.add_command(label="运行物体检测", command=self.run_object_detection)
        menu_bar.add_cascade(label="视觉", menu=vision_menu)
        
        # 演示菜单
        demo_menu = tk.Menu(menu_bar, tearoff=0)
        demo_menu.add_command(label="抓取-放置演示", command=self.run_pick_place_demo)
        demo_menu.add_command(label="方形轨迹演示", command=self.run_square_demo)
        menu_bar.add_cascade(label="演示", menu=demo_menu)
        
        # 设置菜单
        settings_menu = tk.Menu(menu_bar, tearoff=0)
        settings_menu.add_checkbutton(label="调试模式", command=self.toggle_debug_mode)
        menu_bar.add_cascade(label="设置", menu=settings_menu)
        
        # 帮助菜单
        help_menu = tk.Menu(menu_bar, tearoff=0)
        help_menu.add_command(label="关于", command=self.show_about)
        menu_bar.add_cascade(label="帮助", menu=help_menu)
        
        self.root.config(menu=menu_bar)
    
    def create_notebook(self):
        """创建选项卡式界面"""
        # 基本控制选项卡
        self.basic_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.basic_tab, text="基本控制")
        self.create_basic_tab()
        
        # 笛卡尔控制选项卡
        self.cartesian_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.cartesian_tab, text="笛卡尔控制")
        self.create_cartesian_tab()
        
        # 视觉选项卡
        self.vision_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.vision_tab, text="视觉")
        self.create_vision_tab()
    
    def create_basic_tab(self):
        """创建基本控制选项卡内容"""
        # 左右分栏
        paned_window = ttk.PanedWindow(self.basic_tab, orient=tk.HORIZONTAL)
        paned_window.pack(fill=tk.BOTH, expand=True)
        
        # 左侧控制面板
        left_frame = ttk.Frame(paned_window)
        paned_window.add(left_frame, weight=1)
        
        # 右侧状态面板
        right_frame = ttk.Frame(paned_window)
        paned_window.add(right_frame, weight=1)
        
        # 创建连接设置面板
        self.create_connection_panel(left_frame)
        
        # 创建小臂舵机控制面板
        self.create_servo_control_panel(left_frame)
        
        # 创建大臂电机控制面板
        self.create_motor_control_panel(left_frame)
        
        # 创建功能按钮面板
        self.create_function_panel(left_frame)
        
        # 创建状态面板
        self.create_status_panel(right_frame)
    
    def create_vision_tab(self):
        """创建视觉选项卡内容"""
        # 左右分栏
        paned_window = ttk.PanedWindow(self.vision_tab, orient=tk.HORIZONTAL)
        paned_window.pack(fill=tk.BOTH, expand=True)
        
        # 左侧视频显示区域
        video_frame = ttk.Frame(paned_window)
        paned_window.add(video_frame, weight=3)
        
        # 右侧视觉控制区域
        control_frame = ttk.Frame(paned_window)
        paned_window.add(control_frame, weight=1)
        
        # 视频显示
        self.video_label = ttk.Label(video_frame)
        self.video_label.pack(fill=tk.BOTH, expand=True)
        
        # 视频控制
        video_control_frame = ttk.LabelFrame(control_frame, text="视频控制")
        video_control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 开启/关闭摄像头按钮
        self.camera_btn = ttk.Button(video_control_frame, text="开启摄像头", command=self.toggle_camera)
        self.camera_btn.pack(fill=tk.X, padx=5, pady=5)
        
        # 视觉处理开关
        vision_enable_var = tk.BooleanVar(value=False)
        self.vision_enable_check = ttk.Checkbutton(
            video_control_frame, 
            text="启用视觉处理",
            variable=vision_enable_var,
            command=lambda: self.toggle_vision_processing(vision_enable_var.get())
        )
        self.vision_enable_check.pack(fill=tk.X, padx=5, pady=5)
        
        # 视觉参数控制框架
        vision_params_frame = ttk.LabelFrame(control_frame, text="颜色过滤参数")
        vision_params_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # HSV参数滑块
        param_names = [
            ("色调下限", "hue_low", 0, 179),
            ("色调上限", "hue_high", 0, 179),
            ("饱和度下限", "sat_low", 0, 255),
            ("饱和度上限", "sat_high", 0, 255),
            ("亮度下限", "val_low", 0, 255),
            ("亮度上限", "val_high", 0, 255)
        ]
        
        self.vision_sliders = {}
        
        for i, (name, param, min_val, max_val) in enumerate(param_names):
            # 参数名称标签
            ttk.Label(vision_params_frame, text=name).grid(row=i, column=0, sticky=tk.W, padx=5, pady=2)
            
            # 参数滑块
            slider = ttk.Scale(
                vision_params_frame,
                from_=min_val,
                to=max_val,
                orient=tk.HORIZONTAL,
                command=lambda val, p=param: self.update_vision_param(p, float(val))
            )
            slider.set(self.vision_params[param])
            slider.grid(row=i, column=1, padx=5, pady=2)
            
            # 参数值标签
            value_var = tk.StringVar(value=str(self.vision_params[param]))
            ttk.Label(vision_params_frame, textvariable=value_var).grid(row=i, column=2, padx=5, pady=2)
            
            self.vision_sliders[param] = (slider, value_var)
        
        # 视觉操作按钮
        vision_actions_frame = ttk.LabelFrame(control_frame, text="视觉操作")
        vision_actions_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(
            vision_actions_frame,
            text="运行物体检测",
            command=self.run_object_detection
        ).pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(
            vision_actions_frame,
            text="跟踪物体",
            command=self.track_object
        ).pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(
            vision_actions_frame,
            text="物体抓取",
            command=self.grab_detected_object
        ).pack(fill=tk.X, padx=5, pady=5)
    
    def create_cartesian_tab(self):
        """创建笛卡尔控制选项卡内容"""
        # 笛卡尔坐标控制框架
        cartesian_frame = ttk.LabelFrame(self.cartesian_tab, text="笛卡尔坐标控制")
        cartesian_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # X坐标控制
        ttk.Label(cartesian_frame, text="X 坐标:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.x_var = tk.IntVar(value=0)
        x_spinbox = ttk.Spinbox(cartesian_frame, from_=-50, to=50, textvariable=self.x_var, width=5)
        x_spinbox.grid(row=0, column=1, padx=5, pady=5)
        
        # Y坐标控制
        ttk.Label(cartesian_frame, text="Y 坐标:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.y_var = tk.IntVar(value=15)
        y_spinbox = ttk.Spinbox(cartesian_frame, from_=-50, to=50, textvariable=self.y_var, width=5)
        y_spinbox.grid(row=1, column=1, padx=5, pady=5)
        
        # Z坐标控制
        ttk.Label(cartesian_frame, text="Z 坐标:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        self.z_var = tk.IntVar(value=0)
        z_spinbox = ttk.Spinbox(cartesian_frame, from_=-50, to=50, textvariable=self.z_var, width=5)
        z_spinbox.grid(row=2, column=1, padx=5, pady=5)
        
        # 移动按钮
        ttk.Button(
            cartesian_frame,
            text="移动到位置",
            command=self.move_cartesian
        ).grid(row=3, column=0, columnspan=2, padx=5, pady=5)
        
        # 预设动作框架
        presets_frame = ttk.LabelFrame(self.cartesian_tab, text="预设动作")
        presets_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 抓取-放置示例按钮
        ttk.Button(
            presets_frame,
            text="执行抓取-放置示例",
            command=self.run_pick_place_demo
        ).pack(fill=tk.X, padx=5, pady=5)
        
        # 方形轨迹示例按钮
        ttk.Button(
            presets_frame,
            text="执行方形轨迹示例",
            command=self.run_square_demo
        ).pack(fill=tk.X, padx=5, pady=5) 

    def create_connection_panel(self, parent):
        """创建连接设置面板"""
        # 连接设置框架
        conn_frame = ttk.LabelFrame(parent, text="连接设置")
        conn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 舵机连接设置
        ttk.Label(conn_frame, text="舵机端口:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.servo_port_var = tk.StringVar()
        self.servo_port_combo = ttk.Combobox(conn_frame, textvariable=self.servo_port_var)
        self.servo_port_combo.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(conn_frame, text="舵机波特率:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.servo_baudrate_var = tk.StringVar(value="1000000")
        self.servo_baudrate_combo = ttk.Combobox(conn_frame, textvariable=self.servo_baudrate_var, 
                                                values=["115200", "1000000", "500000"])
        self.servo_baudrate_combo.grid(row=1, column=1, padx=5, pady=5)
        
        self.servo_connect_btn = ttk.Button(conn_frame, text="连接舵机", command=self.connect_servo)
        self.servo_connect_btn.grid(row=0, column=2, rowspan=2, padx=5, pady=5)
        
        # 电机连接设置
        ttk.Label(conn_frame, text="电机端口:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        self.motor_port_var = tk.StringVar()
        self.motor_port_combo = ttk.Combobox(conn_frame, textvariable=self.motor_port_var)
        self.motor_port_combo.grid(row=2, column=1, padx=5, pady=5)
        
        ttk.Label(conn_frame, text="电机波特率:").grid(row=3, column=0, sticky=tk.W, padx=5, pady=5)
        self.motor_baudrate_var = tk.StringVar(value="115200")
        self.motor_baudrate_combo = ttk.Combobox(conn_frame, textvariable=self.motor_baudrate_var, 
                                               values=["9600", "19200", "38400", "57600", "115200"])
        self.motor_baudrate_combo.grid(row=3, column=1, padx=5, pady=5)
        
        self.motor_connect_btn = ttk.Button(conn_frame, text="连接电机", command=self.connect_motor)
        self.motor_connect_btn.grid(row=2, column=2, rowspan=2, padx=5, pady=5)
    
    def create_servo_control_panel(self, parent):
        """创建小臂舵机控制面板"""
        # 小臂舵机控制框架
        servo_frame = ttk.LabelFrame(parent, text="小臂舵机控制")
        servo_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 舵机扫描按钮和使能按钮
        buttons_frame = ttk.Frame(servo_frame)
        buttons_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.scan_button = ttk.Button(buttons_frame, text="扫描舵机", command=self.scan_servos)
        self.scan_button.pack(side=tk.LEFT, padx=5)
        
        self.enable_button = ttk.Button(buttons_frame, text="使能舵机", command=self.toggle_torque)
        self.enable_button.pack(side=tk.LEFT, padx=5)
        
        # 舵机速度控制
        speed_frame = ttk.Frame(servo_frame)
        speed_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(speed_frame, text="舵机速度:").pack(side=tk.LEFT, padx=5)
        self.servo_speed_var = tk.IntVar(value=500)
        self.servo_speed_slider = ttk.Scale(speed_frame, from_=0, to=1000, orient=tk.HORIZONTAL,
                                           variable=self.servo_speed_var, 
                                           command=self.update_speeds)
        self.servo_speed_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.servo_speed_label = ttk.Label(speed_frame, text="500")
        self.servo_speed_label.pack(side=tk.LEFT, padx=5)
        
        # 加速度控制
        acc_frame = ttk.Frame(servo_frame)
        acc_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(acc_frame, text="加速度:").pack(side=tk.LEFT, padx=5)
        self.acc_var = tk.IntVar(value=5)
        self.acc_slider = ttk.Scale(acc_frame, from_=1, to=50, orient=tk.HORIZONTAL,
                                   variable=self.acc_var,
                                   command=self.update_acceleration)
        self.acc_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.acc_label = ttk.Label(acc_frame, text="5")
        self.acc_label.pack(side=tk.LEFT, padx=5)
        
        # 分别创建四个关节控制
        self.joint_controls = {}
        self.joint_values = {}
        
        for i, joint in enumerate(['joint1', 'joint2', 'joint3', 'joint4']):
            # 创建关节框架
            joint_frame = ttk.Frame(servo_frame)
            joint_frame.pack(fill=tk.X, padx=5, pady=5)
            
            # 创建关节名称标签
            joint_label = "底座旋转" if joint == 'joint1' else \
                         "肩部" if joint == 'joint2' else \
                         "肘部" if joint == 'joint3' else "腕部"
            ttk.Label(joint_frame, text=f"{joint_label}:").pack(side=tk.LEFT, padx=5)
            
            # 创建关节位置变量和滑块
            self.joint_values[joint] = tk.IntVar(value=2048)
            joint_scale = ttk.Scale(joint_frame, from_=0, to=4095, orient=tk.HORIZONTAL, 
                                   variable=self.joint_values[joint], length=200)
            joint_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            
            # 创建位置显示和设置按钮
            pos_frame = ttk.Frame(joint_frame)
            pos_frame.pack(side=tk.LEFT, padx=5)
            
            pos_label = ttk.Label(pos_frame, textvariable=self.joint_values[joint], width=5)
            pos_label.pack(side=tk.LEFT, padx=2)
            
            set_btn = ttk.Button(pos_frame, text="设置", 
                               command=lambda j=joint: self.set_joint_position(j))
            set_btn.pack(side=tk.LEFT, padx=2)
            
            self.joint_controls[joint] = {
                'scale': joint_scale,
                'label': pos_label,
                'button': set_btn
            }
    
    def create_motor_control_panel(self, parent):
        """创建大臂电机控制面板"""
        # 大臂电机控制框架
        motor_frame = ttk.LabelFrame(parent, text="大臂电机控制")
        motor_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # DC电机使能控制和测试按钮
        buttons_frame = ttk.Frame(motor_frame)
        buttons_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.dc_enable_button = ttk.Button(buttons_frame, text="使能DC电机", command=self.enable_dc_motors)
        self.dc_enable_button.pack(side=tk.LEFT, padx=5)
        
        self.test_dc_button = ttk.Button(buttons_frame, text="测试DC电机通信", command=self.test_dc_motors)
        self.test_dc_button.pack(side=tk.LEFT, padx=5)
        
        # YF俯仰电机控制
        pitch_frame = ttk.Frame(motor_frame)
        pitch_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(pitch_frame, text="YF俯仰速度:").pack(side=tk.LEFT, padx=5)
        self.pitch_speed_var = tk.IntVar(value=100)
        self.pitch_speed_slider = ttk.Scale(pitch_frame, from_=0, to=255, orient=tk.HORIZONTAL,
                                           variable=self.pitch_speed_var,
                                           command=self.update_speeds)
        self.pitch_speed_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.pitch_speed_label = ttk.Label(pitch_frame, text="100")
        self.pitch_speed_label.pack(side=tk.LEFT, padx=5)
        
        # 俯仰位置控制
        pitch_pos_frame = ttk.Frame(motor_frame)
        pitch_pos_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(pitch_pos_frame, text="YF俯仰位置:").pack(side=tk.LEFT, padx=5)
        self.pitch_value = tk.IntVar(value=0)
        self.pitch_slider = ttk.Scale(pitch_pos_frame, from_=-45, to=45, orient=tk.HORIZONTAL,
                                     variable=self.pitch_value,
                                     command=lambda v: self.update_pitch_position())
        self.pitch_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        pitch_control_frame = ttk.Frame(pitch_pos_frame)
        pitch_control_frame.pack(side=tk.LEFT, padx=5)
        
        self.pitch_value_label = ttk.Label(pitch_control_frame, textvariable=self.pitch_value, width=5)
        self.pitch_value_label.pack(side=tk.LEFT, padx=2)
        
        ttk.Button(pitch_control_frame, text="设置", command=self.set_pitch_position).pack(side=tk.LEFT, padx=2)
        
        # AIMotor进给电机控制
        linear_frame = ttk.Frame(motor_frame)
        linear_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(linear_frame, text="AImotor线性速度:").pack(side=tk.LEFT, padx=5)
        self.linear_speed_var = tk.IntVar(value=100)
        self.linear_speed_slider = ttk.Scale(linear_frame, from_=0, to=255, orient=tk.HORIZONTAL,
                                            variable=self.linear_speed_var,
                                            command=self.update_speeds)
        self.linear_speed_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        self.linear_speed_label = ttk.Label(linear_frame, text="100")
        self.linear_speed_label.pack(side=tk.LEFT, padx=5)
        
        # 线性位置控制
        linear_pos_frame = ttk.Frame(motor_frame)
        linear_pos_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(linear_pos_frame, text="AImotor线性位置:").pack(side=tk.LEFT, padx=5)
        self.linear_value = tk.IntVar(value=0)
        self.linear_slider = ttk.Scale(linear_pos_frame, from_=-500, to=500, orient=tk.HORIZONTAL,
                                      variable=self.linear_value,
                                      command=lambda v: self.update_linear_position())
        self.linear_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        linear_control_frame = ttk.Frame(linear_pos_frame)
        linear_control_frame.pack(side=tk.LEFT, padx=5)
        
        self.linear_value_label = ttk.Label(linear_control_frame, textvariable=self.linear_value, width=5)
        self.linear_value_label.pack(side=tk.LEFT, padx=2)
        
        ttk.Button(linear_control_frame, text="设置", command=self.set_linear_position).pack(side=tk.LEFT, padx=2)
    
    def create_function_panel(self, parent):
        """创建功能按钮面板"""
        # 功能按钮框架
        function_frame = ttk.Frame(parent)
        function_frame.pack(fill=tk.X, padx=5, pady=10)
        
        # 紧急停止按钮
        self.stop_button = ttk.Button(function_frame, text="紧急停止", command=self.emergency_stop, style="Red.TButton")
        self.stop_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # 归位按钮
        self.home_button = ttk.Button(function_frame, text="归位", command=self.home_robot)
        self.home_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # 校准按钮
        self.calibrate_button = ttk.Button(function_frame, text="校准", command=self.calibrate_robot)
        self.calibrate_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # 断开连接按钮
        self.disconnect_button = ttk.Button(function_frame, text="断开连接", command=self.disconnect)
        self.disconnect_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # 创建红色按钮样式
        style = ttk.Style()
        style.configure("Red.TButton", foreground="white", background="red")
    
    def create_status_panel(self, parent):
        """创建状态面板"""
        # 状态面板标题
        ttk.Label(parent, text="状态面板", font=("Arial", 16)).pack(pady=10)
        
        # 连接状态框架
        conn_status_frame = ttk.LabelFrame(parent, text="连接状态")
        conn_status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 舵机连接状态
        ttk.Label(conn_status_frame, text="舵机状态:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.servo_status_var = tk.StringVar(value="未连接")
        ttk.Label(conn_status_frame, textvariable=self.servo_status_var).grid(row=0, column=1, padx=5, pady=5)
        
        # 电机连接状态
        ttk.Label(conn_status_frame, text="电机状态:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.motor_status_var = tk.StringVar(value="未连接")
        ttk.Label(conn_status_frame, textvariable=self.motor_status_var).grid(row=1, column=1, padx=5, pady=5)
        
        # 电压状态
        ttk.Label(conn_status_frame, text="舵机电压:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=5)
        self.voltage_var = tk.StringVar(value="未检测")
        self.voltage_label = ttk.Label(conn_status_frame, textvariable=self.voltage_var)
        self.voltage_label.grid(row=2, column=1, padx=5, pady=5)
        
        # 检查电压按钮
        self.check_voltage_button = ttk.Button(conn_status_frame, text="检查电压", command=self.check_voltage)
        self.check_voltage_button.grid(row=2, column=2, padx=5, pady=5)
        
        # 关节位置显示框架
        joint_status_frame = ttk.LabelFrame(parent, text="关节位置")
        joint_status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 各关节位置显示
        self.joint_status_vars = {}
        for i, joint in enumerate(['joint1', 'joint2', 'joint3', 'joint4']):
            joint_label = "底座旋转" if joint == 'joint1' else \
                         "肩部" if joint == 'joint2' else \
                         "肘部" if joint == 'joint3' else "腕部"
            ttk.Label(joint_status_frame, text=f"{joint_label}:").grid(row=i, column=0, sticky=tk.W, padx=5, pady=2)
            
            self.joint_status_vars[joint] = tk.StringVar(value="--")
            ttk.Label(joint_status_frame, textvariable=self.joint_status_vars[joint]).grid(row=i, column=1, padx=5, pady=2)
        
        # 电机位置显示框架
        motor_status_frame = ttk.LabelFrame(parent, text="电机位置")
        motor_status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 俯仰电机位置
        ttk.Label(motor_status_frame, text="俯仰电机:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.pitch_status_var = tk.StringVar(value="--")
        ttk.Label(motor_status_frame, textvariable=self.pitch_status_var).grid(row=0, column=1, padx=5, pady=2)
        
        # 进给电机位置
        ttk.Label(motor_status_frame, text="进给电机:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.linear_status_var = tk.StringVar(value="--")
        ttk.Label(motor_status_frame, textvariable=self.linear_status_var).grid(row=1, column=1, padx=5, pady=2)
        
        # 日志显示框
        log_frame = ttk.LabelFrame(parent, text="操作日志")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.log_text = tk.Text(log_frame, wrap=tk.WORD, height=10)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 添加滚动条
        scrollbar = ttk.Scrollbar(self.log_text, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)
        
        # 禁用文本框编辑
        self.log_text.config(state=tk.DISABLED) 

    def log(self, message):
        """向日志框添加消息"""
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
        
        # 同时更新状态栏
        self.status_bar.config(text=message)
    
    def update_port_list(self):
        """更新串口列表"""
        self.log("正在扫描可用串口...")
        
        # 启动异步扫描任务
        scan_task = PortScanTask(callback=self._on_port_scan_complete)
        scan_task.start()
    
    def _on_port_scan_complete(self, ports, error=None):
        """串口扫描完成的回调"""
        if error:
            self.log(f"扫描串口时出错: {error}")
            return
            
        if not ports:
            self.log("未找到可用串口")
            ports = [""]
        
        # 更新下拉框内容
        self.servo_port_combo["values"] = ports
        self.motor_port_combo["values"] = ports
        
        # 如果当前选择为空，则设置为第一个可用端口
        if not self.servo_port_var.get() and ports[0]:
            self.servo_port_var.set(ports[0])
            
        if not self.motor_port_var.get() and len(ports) > 1:
            self.motor_port_var.set(ports[1] if len(ports) > 1 else ports[0])
        
        self.log(f"已更新端口列表: {', '.join(ports)}")
    
    def connect_servo(self):
        """连接舵机控制器"""
        if self.servo_connected:
            self.log("舵机控制器已连接")
            return
        
        servo_port = self.servo_port_var.get()
        if not servo_port:
            messagebox.showerror("错误", "请选择舵机串口")
            return
        
        try:
            servo_baudrate = int(self.servo_baudrate_var.get())
            
            self.log(f"正在连接舵机控制器 ({servo_port}, {servo_baudrate})...")
            
            # 连接舵机控制器
            if self.robot.connect_servo(servo_port, servo_baudrate):
                self.servo_connected = True
                self.servo_status_var.set(f"已连接 ({servo_port})")
                self.log("舵机控制器连接成功")
                
                # 启用舵机相关控件
                self.enable_servo_controls(True)
                
                # 更新关节位置
                self.update_joint_positions()
            else:
                messagebox.showerror("连接失败", "无法连接到舵机控制器")
                self.log("舵机控制器连接失败")
        except Exception as e:
            messagebox.showerror("连接错误", f"连接舵机控制器时出错: {e}")
            self.log(f"连接舵机控制器时出错: {e}")
    
    def connect_motor(self):
        """连接电机控制器"""
        if self.motor_connected:
            self.log("电机控制器已连接")
            return
        
        motor_port = self.motor_port_var.get()
        if not motor_port:
            messagebox.showerror("错误", "请选择电机串口")
            return
        
        try:
            motor_baudrate = int(self.motor_baudrate_var.get())
            
            self.log(f"正在连接电机控制器 ({motor_port}, {motor_baudrate})...")
            
            # 连接电机控制器
            if self.robot.connect_motor(motor_port, motor_baudrate):
                self.motor_connected = True
                self.motor_status_var.set(f"已连接 ({motor_port})")
                self.log("电机控制器连接成功")
                
                # 启用电机相关控件
                self.enable_motor_controls(True)
                
                # 更新电机位置
                self.update_motor_positions()
            else:
                messagebox.showerror("连接失败", "无法连接到电机控制器")
                self.log("电机控制器连接失败")
        except Exception as e:
            messagebox.showerror("连接错误", f"连接电机控制器时出错: {e}")
            self.log(f"连接电机控制器时出错: {e}")
    
    def disconnect(self):
        """断开与控制器的连接"""
        try:
            if self.servo_connected or self.motor_connected:
                # 停止所有电机
                if self.servo_connected or self.motor_connected:
                    try:
                        self.robot.stop_all()
                    except Exception as e:
                        self.log(f"停止电机时出错: {e}")
                
                # 断开连接
                self.robot.disconnect()
                
                # 更新状态
                self.servo_connected = False
                self.motor_connected = False
                
                self.servo_status_var.set("未连接")
                self.motor_status_var.set("未连接")
                
                # 禁用控件
                self.enable_servo_controls(False)
                self.enable_motor_controls(False)
                
                self.log("已断开所有连接")
            else:
                self.log("未连接任何控制器")
        except Exception as e:
            self.log(f"断开连接时出错: {e}")
    
    def enable_servo_controls(self, enable=True):
        """启用或禁用舵机控制相关控件"""
        # 舵机扫描和使能按钮
        self.scan_button.config(state=tk.NORMAL if enable else tk.DISABLED)
        self.enable_button.config(state=tk.NORMAL if enable else tk.DISABLED)
        
        # 速度和加速度滑块
        self.servo_speed_slider.config(state=tk.NORMAL if enable else tk.DISABLED)
        self.acc_slider.config(state=tk.NORMAL if enable else tk.DISABLED)
        self.check_voltage_button.config(state=tk.NORMAL if enable else tk.DISABLED)
        
        # 关节控制
        for joint in self.joint_controls:
            self.joint_controls[joint]['scale'].config(state=tk.NORMAL if enable else tk.DISABLED)
            self.joint_controls[joint]['button'].config(state=tk.NORMAL if enable else tk.DISABLED)
    
    def enable_motor_controls(self, enable=True):
        """启用或禁用电机控制相关控件"""
        # DC电机使能和测试按钮
        self.dc_enable_button.config(state=tk.NORMAL if enable else tk.DISABLED)
        self.test_dc_button.config(state=tk.NORMAL if enable else tk.DISABLED)
        
        # 速度控制
        self.pitch_speed_slider.config(state=tk.NORMAL if enable else tk.DISABLED)
        self.linear_speed_slider.config(state=tk.NORMAL if enable else tk.DISABLED)
        
        # 位置控制
        self.pitch_slider.config(state=tk.NORMAL if enable else tk.DISABLED)
        self.linear_slider.config(state=tk.NORMAL if enable else tk.DISABLED)
    
    def scan_servos(self):
        """扫描舵机"""
        if not self.servo_connected:
            self.log("请先连接舵机控制器")
            return
            
        self.log("正在扫描舵机...")
        
        # 创建扫描任务
        scan_task = ServoScanTask(self.robot, 1, 10, callback=self._on_servo_scan_complete)
        scan_task.start()
    
    def _on_servo_scan_complete(self, servos, error=None):
        """舵机扫描完成回调"""
        if error:
            self.log(f"扫描舵机时出错: {error}")
            messagebox.showerror("扫描错误", f"扫描舵机时出错: {error}")
            return
            
        self.servos = servos
        
        if not servos:
            self.log("未找到任何舵机")
            messagebox.showinfo("扫描结果", "未找到任何舵机")
        else:
            # 构建舵机信息文本
            servo_info = "\n".join([f"ID: {servo_id}, 型号: {model}" for servo_id, model in servos])
            self.log(f"找到 {len(servos)} 个舵机")
            messagebox.showinfo("扫描结果", f"找到 {len(servos)} 个舵机:\n{servo_info}")
    
    def toggle_torque(self):
        """切换舵机力矩状态"""
        if not self.servo_connected:
            self.log("请先连接舵机控制器")
            return
            
        try:
            if self.enable_button.cget("text") == "使能舵机":
                self.log("正在使能舵机...")
                if self.robot.enable_torque(True):
                    self.enable_button.config(text="禁用舵机")
                    self.log("舵机已使能")
                else:
                    self.log("使能舵机失败")
            else:
                self.log("正在禁用舵机...")
                if self.robot.enable_torque(False):
                    self.enable_button.config(text="使能舵机")
                    self.log("舵机已禁用")
                else:
                    self.log("禁用舵机失败")
        except Exception as e:
            self.log(f"切换力矩状态时出错: {e}")
    
    def enable_dc_motors(self):
        """启用或禁用DC电机"""
        if not self.motor_connected:
            self.log("请先连接电机控制器")
            return
            
        try:
            if self.dc_enable_button.cget("text") == "使能DC电机":
                self.log("正在使能DC电机...")
                
                # 初始化电机
                self.robot.motor.home_motors()
                time.sleep(0.5)
                
                # 设置低速度
                self.robot.motor.set_motor_speed(50, 50)
                
                self.dc_enable_button.config(text="禁用DC电机")
                self.log("DC电机已使能")
            else:
                self.log("正在禁用DC电机...")
                
                # 停止电机
                self.robot.motor.stop_all()
                
                self.dc_enable_button.config(text="使能DC电机")
                self.log("DC电机已禁用")
        except Exception as e:
            self.log(f"控制DC电机时出错: {e}")
            self.dc_enable_button.config(text="使能DC电机")
    
    def update_speeds(self, *args):
        """更新各种电机的速度"""
        try:
            # 更新舵机速度显示
            servo_speed = self.servo_speed_var.get()
            self.servo_speed_label.config(text=str(servo_speed))
            
            # 更新DC电机速度显示
            pitch_speed = self.pitch_speed_var.get()
            linear_speed = self.linear_speed_var.get()
            self.pitch_speed_label.config(text=str(pitch_speed))
            self.linear_speed_label.config(text=str(linear_speed))
            
            # 如果已连接，应用速度设置
            if self.servo_connected:
                self.robot.set_speeds(servo_speed=servo_speed)
                
            if self.motor_connected and self.dc_enable_button.cget("text") == "禁用DC电机":
                self.robot.motor.set_motor_speed(pitch_speed, linear_speed)
        except Exception as e:
            self.log(f"更新速度时出错: {e}")
    
    def update_acceleration(self, *args):
        """更新舵机加速度"""
        try:
            # 更新加速度显示
            acc = self.acc_var.get()
            self.acc_label.config(text=str(acc))
            
            # 如果已连接，应用加速度设置
            if self.servo_connected:
                self.robot.set_all_servo_acc(acc)
        except Exception as e:
            self.log(f"更新加速度时出错: {e}")
    
    def update_pitch_position(self):
        """更新俯仰滑块显示"""
        self.pitch_value_label.config(text=str(self.pitch_value.get()))
    
    def update_linear_position(self):
        """更新线性滑块显示"""
        self.linear_value_label.config(text=str(self.linear_value.get()))
    
    def set_joint_position(self, joint):
        """设置关节位置"""
        if not self.servo_connected:
            self.log("请先连接舵机控制器")
            return
            
        try:
            # 检查舵机是否使能
            if self.enable_button.cget("text") != "禁用舵机":
                self.log("请先使能舵机")
                return
                
            position = self.joint_values[joint].get()
            
            self.log(f"正在设置{joint}位置到 {position}...")
            
            # 设置关节位置
            if self.robot.set_joint_position(joint, position):
                self.log(f"{joint}位置已设置为 {position}")
            else:
                self.log(f"设置{joint}位置失败")
                
            # 更新状态
            self.update_joint_positions()
        except Exception as e:
            self.log(f"设置关节位置时出错: {e}")
    
    def set_pitch_position(self):
        """设置俯仰电机位置"""
        if not self.motor_connected:
            self.log("请先连接电机控制器")
            return
            
        try:
            # 检查电机是否使能
            if self.dc_enable_button.cget("text") != "禁用DC电机":
                self.log("请先使能DC电机")
                return
                
            position = self.pitch_value.get()
            
            self.log(f"正在设置俯仰电机位置到 {position}...")
            
            # 设置俯仰电机位置
            if self.robot.set_pitch_position(position, self.pitch_speed_var.get()):
                self.log(f"俯仰电机位置已设置为 {position}")
            else:
                self.log(f"设置俯仰电机位置失败")
                
            # 更新状态
            self.update_motor_positions()
        except Exception as e:
            self.log(f"设置俯仰电机位置时出错: {e}")
    
    def set_linear_position(self):
        """设置线性电机位置"""
        if not self.motor_connected:
            self.log("请先连接电机控制器")
            return
            
        try:
            # 检查电机是否使能
            if self.dc_enable_button.cget("text") != "禁用DC电机":
                self.log("请先使能DC电机")
                return
                
            position = self.linear_value.get()
            
            self.log(f"正在设置线性电机位置到 {position}...")
            
            # 设置线性电机位置
            if self.robot.set_linear_position(position, self.linear_speed_var.get()):
                self.log(f"线性电机位置已设置为 {position}")
            else:
                self.log(f"设置线性电机位置失败")
                
            # 更新状态
            self.update_motor_positions()
        except Exception as e:
            self.log(f"设置线性电机位置时出错: {e}")
    
    def emergency_stop(self):
        """紧急停止所有电机"""
        try:
            self.log("紧急停止！")
            
            if self.servo_connected or self.motor_connected:
                self.robot.stop_all()
                
                # 更新按钮状态
                if self.servo_connected:
                    self.enable_button.config(text="使能舵机")
                
                if self.motor_connected:
                    self.dc_enable_button.config(text="使能DC电机")
                
                self.log("所有电机已停止")
            else:
                self.log("未连接任何控制器")
        except Exception as e:
            self.log(f"紧急停止时出错: {e}")
    
    def update_status(self):
        """定时更新状态信息"""
        # 如果已经在更新中，则跳过本次更新
        if self.is_updating_status:
            self.root.after(1000, self.update_status)
            return
            
        self.is_updating_status = True
        
        try:
            # 更新关节位置
            if self.servo_connected:
                self.update_joint_positions()
            
            # 更新电机位置
            if self.motor_connected:
                self.update_motor_positions()
        except Exception as e:
            pass  # 忽略错误，不影响GUI正常运行
        finally:
            self.is_updating_status = False
            
        # 继续定时更新
        self.root.after(1000, self.update_status)
    
    def update_joint_positions(self):
        """更新关节位置信息"""
        if not self.servo_connected:
            return
            
        try:
            positions = self.robot.read_joint_positions()
            if positions:
                for joint, position in positions.items():
                    if joint in self.joint_status_vars:
                        self.joint_status_vars[joint].set(str(position))
        except Exception:
            pass  # 忽略错误
    
    def update_motor_positions(self):
        """更新电机位置信息"""
        if not self.motor_connected:
            return
            
        try:
            pitch_pos, linear_pos = self.robot.get_motor_positions()
            
            if pitch_pos is not None:
                self.pitch_status_var.set(str(pitch_pos))
            
            if linear_pos is not None:
                self.linear_status_var.set(str(linear_pos))
        except Exception:
            pass  # 忽略错误 

    def check_voltage(self):
        """检查舵机电压"""
        if not self.servo_connected:
            self.log("请先连接舵机控制器")
            return
            
        try:
            self.log("正在检查舵机电压...")
            
            voltages = {}
            all_ok = True
            
            # 检查所有舵机的电压
            for joint, servo_id in self.robot.joint_ids.items():
                is_ok, voltage = self.robot.check_servo_voltage(servo_id)
                voltages[joint] = voltage
                
                if not is_ok:
                    all_ok = False
            
            # 更新电压显示
            if voltages:
                voltage_text = ", ".join([f"{j}: {v:.1f}V" for j, v in voltages.items()])
                
                if all_ok:
                    self.voltage_var.set(f"正常: {voltage_text}")
                    self.voltage_label.config(foreground="green")
                else:
                    self.voltage_var.set(f"异常! {voltage_text}")
                    self.voltage_label.config(foreground="red")
                
                self.log("电压检查完成")
            else:
                self.voltage_var.set("读取失败")
                self.voltage_label.config(foreground="orange")
                self.log("无法读取电压")
        except Exception as e:
            self.log(f"检查电压时出错: {e}")
    
    def home_robot(self):
        """将机械臂移动到初始位置"""
        if not self.servo_connected and not self.motor_connected:
            self.log("请先连接至少一个控制器")
            return
            
        try:
            self.log("正在回到初始位置...")
            
            if self.robot.home(blocking=True):
                self.log("已回到初始位置")
                
                # 更新滑块位置
                if self.servo_connected:
                    for joint in self.joint_values:
                        self.joint_values[joint].set(2048)
                
                if self.motor_connected:
                    self.pitch_value.set(0)
                    self.linear_value.set(0)
            else:
                self.log("回到初始位置失败")
        except Exception as e:
            self.log(f"回到初始位置时出错: {e}")
    
    def calibrate_robot(self):
        """校准机械臂"""
        if not self.servo_connected and not self.motor_connected:
            self.log("请先连接至少一个控制器")
            return
            
        try:
            self.log("正在校准机械臂...")
            
            if self.servo_connected:
                # 先禁用再重新使能
                self.robot.enable_torque(False)
                time.sleep(0.3)
                self.robot.enable_torque(True)
                self.enable_button.config(text="禁用舵机")
            
            # 如果舵机已连接，检查电压
            if self.servo_connected:
                self.check_voltage()
            
            # 设置较低的速度和加速度
            if self.servo_connected:
                self.robot.set_speeds(servo_speed=300)
                self.robot.set_all_servo_acc(3)
                self.servo_speed_var.set(300)
                self.acc_var.set(3)
            
            if self.motor_connected:
                self.robot.motor.set_motor_speed(80, 80)
                self.pitch_speed_var.set(80)
                self.linear_speed_var.set(80)
            
            # 返回初始位置
            self.home_robot()
            
            self.log("校准完成")
        except Exception as e:
            self.log(f"校准时出错: {e}")
    
    def test_dc_motors(self):
        """测试DC电机通信"""
        if not self.motor_connected:
            self.log("请先连接电机控制器")
            return
            
        try:
            # 启用调试模式
            self.log("正在测试DC电机通信...")
            
            # 测试通信
            results = self.robot.motor.test_communication()
            
            # 显示结果
            result_text = f"YF俯仰电机: {'成功' if results['yf_pitch'] else '失败'}\n"
            result_text += f"AImotor进给电机: {'成功' if results['ai_linear'] else '失败'}"
            
            self.log(f"测试结果: {result_text}")
            messagebox.showinfo("测试结果", result_text)
        except Exception as e:
            self.log(f"测试DC电机通信时出错: {e}")
            messagebox.showerror("测试错误", str(e))
    
    def toggle_debug_mode(self):
        """切换调试模式"""
        self.debug_mode = not self.debug_mode
        
        # 如果电机已连接，设置调试模式
        if self.motor_connected:
            self.robot.motor.set_debug(self.debug_mode)
        
        self.log(f"调试模式: {'已启用' if self.debug_mode else '已禁用'}")
    
    def move_cartesian(self):
        """基于笛卡尔坐标移动机械臂"""
        # 检查舵机和电机是否连接
        if not self.servo_connected or not self.motor_connected:
            self.log("请先连接舵机和电机控制器")
            return
            
        # 检查舵机和电机是否使能
        if self.enable_button.cget("text") != "禁用舵机" or self.dc_enable_button.cget("text") != "禁用DC电机":
            self.log("请先使能舵机和DC电机")
            return
            
        try:
            x = self.x_var.get()
            y = self.y_var.get()
            z = self.z_var.get()
            
            self.log(f"正在移动到笛卡尔坐标: X={x}, Y={y}, Z={z}...")
            
            # 先检查电压
            if self.servo_connected:
                self.check_voltage()
            
            # 设置较低的速度
            self.robot.set_speeds(servo_speed=300)
            self.robot.set_all_servo_acc(3)
            self.robot.motor.set_motor_speed(80, 80)
            
            # 简单映射到关节和电机坐标
            # X -> joint1 (底座旋转)
            # Y -> linear (进给电机)
            # Z -> pitch (俯仰电机)
            
            # 将X映射到底座旋转
            joint1_pos = int(x * 10 + 2048)  # 简单线性映射
            
            # 设置底座位置
            self.robot.set_joint_position('joint1', joint1_pos)
            time.sleep(0.5)  # 等待底座移动
            
            # 设置进给位置
            self.robot.set_linear_position(y * 100)  # 放大系数
            time.sleep(0.5)
            
            # 设置俯仰位置
            self.robot.set_pitch_position(z * 100)  # 放大系数
            
            self.log(f"已移动到: X={x}, Y={y}, Z={z}")
            
            # 更新UI上的滑块值
            self.joint_values['joint1'].set(joint1_pos)
            self.linear_value.set(y * 100)
            self.pitch_value.set(z * 100)
        except Exception as e:
            self.log(f"笛卡尔坐标移动出错: {e}")
    
    def run_pick_place_demo(self):
        """执行抓取和放置演示"""
        # 检查舵机和电机是否连接
        if not self.servo_connected or not self.motor_connected:
            self.log("请先连接舵机和电机控制器")
            return
            
        # 检查舵机和电机是否使能
        if self.enable_button.cget("text") != "禁用舵机" or self.dc_enable_button.cget("text") != "禁用DC电机":
            self.log("请先使能舵机和DC电机")
            return
            
        try:
            self.log("执行抓取-放置演示...")
            
            # 先检查电压
            self.check_voltage()
            
            # 设置较低的速度和加速度
            self.robot.set_speeds(servo_speed=300)
            self.robot.set_all_servo_acc(3)
            self.robot.motor.set_motor_speed(80, 80)
            
            # 抓取位置
            self.log("移动到抓取位置...")
            positions = {
                'joint1': 1500,  # 底座旋转
                'joint2': 1700,  # 肩部
                'joint3': 2300,  # 肘部
                'joint4': 2048   # 手腕
            }
            
            # 设置关节位置
            success = self.robot.set_joint_positions(positions, blocking=True)
            if not success:
                self.log("移动失败，取消演示")
                return
            
            # 更新UI上的滑块
            for joint, pos in positions.items():
                self.joint_values[joint].set(pos)
            
            # 降低抓取
            self.log("降低抓取...")
            self.robot.set_pitch_position(-1000)
            time.sleep(1.5)
            self.pitch_value.set(-1000)
            
            # 模拟抓取
            self.log("抓取中...")
            time.sleep(1)
            
            # 抬起
            self.log("抬起物体...")
            self.robot.set_pitch_position(0)
            time.sleep(1.5)
            self.pitch_value.set(0)
            
            # 移动到放置位置
            self.log("移动到放置位置...")
            self.robot.set_joint_position('joint1', 2500, blocking=True)
            time.sleep(0.5)
            self.joint_values['joint1'].set(2500)
            
            # 降低放置
            self.log("降低放置...")
            self.robot.set_pitch_position(-800)
            time.sleep(1.5)
            self.pitch_value.set(-800)
            
            # 释放
            self.log("释放物体...")
            time.sleep(1)
            
            # 抬起
            self.log("抬起机械臂...")
            self.robot.set_pitch_position(0)
            time.sleep(1.5)
            self.pitch_value.set(0)
            
            # 返回初始位置
            self.log("返回初始位置...")
            self.home_robot()
            
            self.log("抓取-放置演示完成")
        except Exception as e:
            self.log(f"执行演示时出错: {e}")
    
    def run_square_demo(self):
        """执行方形轨迹演示"""
        # 检查舵机和电机是否连接
        if not self.servo_connected or not self.motor_connected:
            self.log("请先连接舵机和电机控制器")
            return
            
        # 检查舵机和电机是否使能
        if self.enable_button.cget("text") != "禁用舵机" or self.dc_enable_button.cget("text") != "禁用DC电机":
            self.log("请先使能舵机和DC电机")
            return
            
        try:
            self.log("执行方形轨迹演示...")
            
            # 先检查电压
            self.check_voltage()
            
            # 设置较低的速度和加速度
            self.robot.set_speeds(servo_speed=300)
            self.robot.set_all_servo_acc(3)
            self.robot.motor.set_motor_speed(80, 80)
            
            # 点1 (10, 10, 10)
            self.log("移动到点1 (10, 10, 10)")
            joint1_pos = 10*10 + 2048
            self.robot.set_joint_position('joint1', joint1_pos, blocking=True)
            self.robot.set_linear_position(10*100)
            time.sleep(0.5)
            self.robot.set_pitch_position(10*100)
            time.sleep(1.5)
            
            # 更新UI
            self.joint_values['joint1'].set(joint1_pos)
            self.linear_value.set(10*100)
            self.pitch_value.set(10*100)
            
            # 点2 (10, 20, 10)
            self.log("移动到点2 (10, 20, 10)")
            self.robot.set_linear_position(20*100)
            time.sleep(1.5)
            self.linear_value.set(20*100)
            
            # 点3 (-10, 20, 10)
            self.log("移动到点3 (-10, 20, 10)")
            joint1_pos = -10*10 + 2048
            self.robot.set_joint_position('joint1', joint1_pos, blocking=True)
            time.sleep(1.5)
            self.joint_values['joint1'].set(joint1_pos)
            
            # 点4 (-10, 10, 10)
            self.log("移动到点4 (-10, 10, 10)")
            self.robot.set_linear_position(10*100)
            time.sleep(1.5)
            self.linear_value.set(10*100)
            
            # 回到点1 (10, 10, 10)
            self.log("移动回点1 (10, 10, 10)")
            joint1_pos = 10*10 + 2048
            self.robot.set_joint_position('joint1', joint1_pos, blocking=True)
            time.sleep(1.5)
            self.joint_values['joint1'].set(joint1_pos)
            
            # 返回初始位置
            self.log("返回初始位置...")
            self.home_robot()
            
            self.log("方形轨迹演示完成")
        except Exception as e:
            self.log(f"执行演示时出错: {e}")
    
    def toggle_camera(self):
        """开启/关闭摄像头"""
        if self.video_enabled:
            # 关闭摄像头
            self.video.stop()
            self.video_enabled = False
            self.camera_btn.config(text="开启摄像头")
            self.log("摄像头已关闭")
        else:
            # 开启摄像头
            if self.video.start():
                self.video_enabled = True
                self.camera_btn.config(text="关闭摄像头")
                self.log("摄像头已开启")
                # 启动视频更新
                self.update_video_frame()
            else:
                self.log("无法开启摄像头")
                messagebox.showerror("错误", "无法开启摄像头，请检查摄像头连接")
    
    def update_video_frame(self):
        """更新视频帧"""
        if not self.video_enabled:
            return
            
        try:
            # 获取当前帧
            frame = self.video.get_frame()
            
            if frame is not None:
                # 如果启用了视觉处理，应用HSV过滤
                if self.vision_processing_enabled:
                    frame = self.process_frame(frame)
                
                # 调整大小
                frame = cv2.resize(frame, (640, 480))
                
                # 转换颜色空间
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # 转换为PhotoImage
                image = Image.fromarray(frame)
                photo = ImageTk.PhotoImage(image=image)
                
                # 更新标签
                self.video_label.config(image=photo)
                self.video_label.image = photo  # 保持引用
        except Exception as e:
            self.log(f"更新视频帧时出错: {e}")
            
        # 继续更新
        if self.video_enabled:
            self.root.after(self.video_update_ms, self.update_video_frame)
    
    def process_frame(self, frame):
        """处理视频帧，应用HSV过滤"""
        try:
            # 转换到HSV颜色空间
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # 创建掩码
            lower_bound = np.array([
                int(self.vision_params['hue_low']),
                int(self.vision_params['sat_low']),
                int(self.vision_params['val_low'])
            ])
            
            upper_bound = np.array([
                int(self.vision_params['hue_high']),
                int(self.vision_params['sat_high']),
                int(self.vision_params['val_high'])
            ])
            
            # 应用掩码
            mask = cv2.inRange(hsv, lower_bound, upper_bound)
            result = cv2.bitwise_and(frame, frame, mask=mask)
            
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 绘制轮廓
            cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
            
            # 找到最大轮廓
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                
                # 计算中心点
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # 绘制中心点
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            
            return frame
        except Exception as e:
            self.log(f"处理视频帧时出错: {e}")
            return frame
    
    def toggle_vision_processing(self, enabled):
        """开启/关闭视觉处理"""
        self.vision_processing_enabled = enabled
        self.log(f"视觉处理: {'已启用' if enabled else '已禁用'}")
    
    def update_vision_param(self, param, value):
        """更新视觉参数"""
        self.vision_params[param] = int(value)
        
        # 更新对应的标签
        if param in self.vision_sliders:
            _, value_var = self.vision_sliders[param]
            value_var.set(str(int(value)))
    
    def show_vision_settings(self):
        """显示视觉设置对话框"""
        # 创建一个顶层窗口
        settings_window = tk.Toplevel(self.root)
        settings_window.title("视觉参数设置")
        settings_window.geometry("400x300")
        settings_window.grab_set()  # 模态
        
        # 创建HSV参数滑块
        param_names = [
            ("色调下限", "hue_low", 0, 179),
            ("色调上限", "hue_high", 0, 179),
            ("饱和度下限", "sat_low", 0, 255),
            ("饱和度上限", "sat_high", 0, 255),
            ("亮度下限", "val_low", 0, 255),
            ("亮度上限", "val_high", 0, 255)
        ]
        
        for i, (name, param, min_val, max_val) in enumerate(param_names):
            frame = ttk.Frame(settings_window)
            frame.pack(fill=tk.X, padx=10, pady=5)
            
            # 参数名称
            ttk.Label(frame, text=name).pack(side=tk.LEFT, padx=5)
            
            # 参数滑块
            slider = ttk.Scale(
                frame,
                from_=min_val,
                to=max_val,
                orient=tk.HORIZONTAL,
                value=self.vision_params[param],
                command=lambda val, p=param: self.update_vision_param(p, float(val))
            )
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            
            # 数值显示
            value_var = tk.StringVar(value=str(self.vision_params[param]))
            ttk.Label(frame, textvariable=value_var, width=5).pack(side=tk.LEFT, padx=5)
            
            # 更新滑块引用
            self.vision_sliders[param] = (slider, value_var)
        
        # 确定按钮
        ttk.Button(
            settings_window,
            text="确定",
            command=settings_window.destroy
        ).pack(pady=10)
    
    def run_object_detection(self):
        """运行物体检测"""
        if not self.video_enabled:
            self.log("请先开启摄像头")
            return
            
        # 启用视觉处理
        self.vision_processing_enabled = True
        
        # 更新复选框状态
        self.vision_enable_check.state(['selected'])
        
        self.log("已启动物体检测")
    
    def track_object(self):
        """跟踪检测到的物体"""
        if not self.video_enabled or not self.vision_processing_enabled:
            self.log("请先开启摄像头并启用视觉处理")
            return
            
        if not self.servo_connected or not self.motor_connected:
            self.log("请先连接舵机和电机控制器")
            return
            
        self.log("开始跟踪物体...")
        
        # 实际应用中，这里应该启动一个线程来处理跟踪逻辑
        messagebox.showinfo("跟踪", "物体跟踪功能需要结合具体应用场景进行开发")
    
    def grab_detected_object(self):
        """抓取检测到的物体"""
        if not self.video_enabled or not self.vision_processing_enabled:
            self.log("请先开启摄像头并启用视觉处理")
            return
            
        if not self.servo_connected or not self.motor_connected:
            self.log("请先连接舵机和电机控制器")
            return
            
        self.log("开始抓取检测到的物体...")
        
        # 实际应用中，这里应该启动一个线程来处理抓取逻辑
        messagebox.showinfo("抓取", "物体抓取功能需要结合具体应用场景进行开发")
    
    def show_about(self):
        """显示关于信息"""
        messagebox.showinfo("关于", "机械臂控制面板 v1.0\n\n"
                          "支持单独连接小臂舵机或大臂电机\n"
                          "集成OpenCV视觉功能\n\n"
                          "Copyright © 2023")
    
    def on_exit(self):
        """退出程序"""
        if messagebox.askyesno("确认退出", "确定要退出程序吗？"):
            try:
                # 停止视频
                if self.video_enabled:
                    self.video.stop()
                
                # 断开连接
                if self.servo_connected or self.motor_connected:
                    self.disconnect()
            except:
                pass
                
            self.root.destroy() 


def main():
    """主程序入口"""
    root = tk.Tk()
    app = RobotArmGUI(root)
    
    # 设置窗口图标和样式
    style = ttk.Style()
    if sys.platform.startswith('win'):
        # 在Windows上使用原生主题
        style.theme_use('vista')
    
    # 启用高DPI支持
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except:
        pass
    
    # 加载配置
    # TODO: 添加配置文件支持
    
    root.protocol("WM_DELETE_WINDOW", app.on_exit)  # 处理窗口关闭事件
    
    # 启动主循环
    root.mainloop()


if __name__ == "__main__":
    main()