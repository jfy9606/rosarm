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
import math
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import serial.tools.list_ports
import threading
import queue
import cv2
from PIL import Image, ImageTk
import numpy as np # Added for numpy
import torch
from ultralytics import YOLO

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
    """视频捕获类，用于处理OpenCV视频流和YOLO物体检测"""
    def __init__(self, camera_index=0, stereo_mode=False):
        self.camera_index = camera_index
        self.stereo_mode = stereo_mode
        self.cap = None
        self.running = False
        self.frame_queue = queue.Queue(maxsize=1)
        self.thread = None
        
        # YOLO模型
        self.yolo_model = None
        self.yolo_enabled = False
        self.detection_results = []
        self.confidence_threshold = 0.5
        
        # 双目相机参数
        self.stereo_width = 3840  # 默认分辨率宽度
        self.stereo_height = 1080  # 默认分辨率高度
        self.left_frame = None
        self.right_frame = None
        self.depth_frame = None
        
        # 双目相机校准参数
        self.stereo_calibrated = False
        self.stereo_matcher = None
        
        # 加载YOLO模型的线程
        self.model_loading_thread = None
        
        # 视图模式
        self.view_mode = "normal"  # normal, left, right, depth, anaglyph
        
    @staticmethod
    def get_camera_list():
        """获取可用摄像头列表"""
        camera_list = []
        index = 0
        while True:
            cap = cv2.VideoCapture(index)
            if not cap.isOpened():
                break
            ret, frame = cap.read()
            if ret:
                # 获取摄像头信息
                name = f"Camera {index}"
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                fps = int(cap.get(cv2.CAP_PROP_FPS))
                camera_list.append({
                    "index": index,
                    "name": name,
                    "resolution": f"{width}x{height}",
                    "fps": fps
                })
            cap.release()
            index += 1
        return camera_list
        
    def load_yolo_model(self, model_path=None):
        """异步加载YOLO模型"""
        def _load_model():
            try:
                if model_path and os.path.exists(model_path):
                    self.yolo_model = YOLO(model_path)
                    print(f"成功加载自定义模型: {model_path}")
                else:
                    # 如果没有指定模型路径，尝试使用预训练的YOLOv11n模型
                    try:
                        # 使用force_reload=True确保模型被下载
                        print("正在加载/下载YOLOv11n模型...")
                        self.yolo_model = YOLO('yolo11n.pt')
                        print("YOLOv11n模型加载完成")
                    except Exception as e1:
                        print(f"尝试加载YOLOv11n模型失败: {e1}，尝试使用本地模型...")
                        # 尝试在当前目录和models目录查找模型文件
                        model_paths = ['yolo11n.pt', 'models/yolo11n.pt']
                        for path in model_paths:
                            if os.path.exists(path):
                                self.yolo_model = YOLO(path)
                                print(f"成功加载本地模型: {path}")
                                break
                        else:
                            # 如果本地没有找到模型，尝试创建models目录并下载模型
                            try:
                                print("尝试创建models目录并下载YOLOv11n模型...")
                                os.makedirs('models', exist_ok=True)
                                # 使用ultralytics库直接下载模型
                                from ultralytics.utils.downloads import download
                                model_url = 'https://ghproxy.cfd/https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt'
                                download_path = os.path.join('models', 'yolo11n.pt')
                                download(model_url, download_path)
                                self.yolo_model = YOLO(download_path)
                                print(f"成功下载并加载模型: {download_path}")
                            except Exception as e2:
                                print(f"下载模型失败: {e2}，请手动下载模型")
                                self.yolo_model = None
                
                # 保存COCO数据集的类别名称，用于YOLOv11n格式的结果
                if self.yolo_model is not None and hasattr(self.yolo_model, 'names'):
                    self.class_names = self.yolo_model.names
                else:
                    # 使用COCO数据集的80个类别作为默认类别
                    self.class_names = {
                        0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 6: 'train', 7: 'truck', 
                        8: 'boat', 9: 'traffic light', 10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench', 
                        14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear', 
                        22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 
                        29: 'frisbee', 30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat', 
                        35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 
                        40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 46: 'banana', 
                        47: 'apple', 48: 'sandwich', 49: 'orange', 50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 
                        54: 'donut', 55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed', 60: 'dining table', 
                        61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone', 
                        68: 'microwave', 69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock', 
                        75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'
                    }
            except Exception as e:
                print(f"加载YOLO模型时出错: {e}")
                self.yolo_model = None
                self.class_names = {}
        
        if self.model_loading_thread is None or not self.model_loading_thread.is_alive():
            self.model_loading_thread = threading.Thread(target=_load_model)
            self.model_loading_thread.daemon = True
            self.model_loading_thread.start()
    
    def setup_stereo_matcher(self):
        """设置双目匹配器用于生成深度图"""
        if not self.stereo_calibrated:
            # 创建立体匹配器 - 使用StereoBM算法，速度快但质量一般
            self.stereo_matcher = cv2.StereoBM.create(
                numDisparities=128,  # 视差搜索范围
                blockSize=21         # 匹配块大小
            )
            
            # 可选：使用更高质量但更慢的SGBM算法
            # self.stereo_matcher = cv2.StereoSGBM.create(
            #     minDisparity=0,
            #     numDisparities=128,
            #     blockSize=11,
            #     P1=8 * 3 * 11 * 11,
            #     P2=32 * 3 * 11 * 11,
            #     disp12MaxDiff=1,
            #     uniquenessRatio=10,
            #     speckleWindowSize=100,
            #     speckleRange=32
            # )
            
            self.stereo_calibrated = True
    
    def compute_depth_map(self):
        """计算深度图"""
        if not self.stereo_mode or self.left_frame is None or self.right_frame is None:
            return None
            
        # 确保立体匹配器已设置
        if not self.stereo_calibrated:
            self.setup_stereo_matcher()
            
        try:
            # 转换为灰度图
            left_gray = cv2.cvtColor(self.left_frame, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(self.right_frame, cv2.COLOR_BGR2GRAY)
            
            # 计算视差图
            disparity = self.stereo_matcher.compute(left_gray, right_gray)
            
            # 归一化视差图以便显示
            disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # 创建伪彩色深度图
            depth_colormap = cv2.applyColorMap(disparity_normalized, cv2.COLORMAP_JET)
            
            return depth_colormap
        except Exception as e:
            print(f"计算深度图时出错: {e}")
            return None
    
    def create_anaglyph(self):
        """创建红蓝3D立体图（Anaglyph）"""
        if not self.stereo_mode or self.left_frame is None or self.right_frame is None:
            return None
            
        try:
            # 分离左右图像的通道
            left_b, left_g, left_r = cv2.split(self.left_frame)
            right_b, right_g, right_r = cv2.split(self.right_frame)
            
            # 创建红蓝立体图
            anaglyph_r = right_r
            anaglyph_g = right_g
            anaglyph_b = left_b
            
            # 合并通道
            anaglyph = cv2.merge([anaglyph_b, anaglyph_g, anaglyph_r])
            
            return anaglyph
        except Exception as e:
            print(f"创建立体图时出错: {e}")
            return None
    
    def start(self):
        """启动视频捕获"""
        if self.running:
            return True
            
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                return False
            
            # 如果是双目模式，设置适当的分辨率
            if self.stereo_mode:
                # 设置为双目相机支持的分辨率
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.stereo_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.stereo_height)
                
            self.running = True
            self.thread = threading.Thread(target=self._capture_loop)
            self.thread.daemon = True
            self.thread.start()
            
            # 加载YOLO模型
            if self.yolo_enabled and self.yolo_model is None:
                self.load_yolo_model()
                
            return True
        except Exception as e:
            print(f"启动视频捕获时出错: {e}")
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
            
            # 处理双目相机图像
            if self.stereo_mode and frame is not None:
                # 分割左右图像
                h, w = frame.shape[:2]
                mid = w // 2
                self.left_frame = frame[:, :mid]
                self.right_frame = frame[:, mid:]
                
                # 计算深度图（如果需要）
                if self.view_mode == "depth":
                    self.depth_frame = self.compute_depth_map()
                
                # 进行物体检测
                if self.yolo_enabled and self.yolo_model is not None:
                    try:
                        # 在左图上进行检测
                        results = self.yolo_model(self.left_frame, verbose=False)
                        self.detection_results = results  # 保存检测结果
                    except Exception as e:
                        print(f"YOLO检测出错: {e}")
                        self.detection_results = []
            else:
                # 单目模式
                if self.yolo_enabled and self.yolo_model is not None:
                    try:
                        results = self.yolo_model(frame, verbose=False)
                        self.detection_results = results  # 保存检测结果
                    except Exception as e:
                        print(f"YOLO检测出错: {e}")
                        self.detection_results = []
                
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
    
    def get_stereo_frames(self):
        """获取双目相机的左右帧"""
        if not self.running or not self.stereo_mode:
            return None, None
        return self.left_frame, self.right_frame
    
    def get_depth_frame(self):
        """获取深度图"""
        if not self.running or not self.stereo_mode:
            return None
        
        if self.depth_frame is None and self.left_frame is not None and self.right_frame is not None:
            self.depth_frame = self.compute_depth_map()
            
        return self.depth_frame
    
    def get_anaglyph_frame(self):
        """获取红蓝3D立体图"""
        if not self.running or not self.stereo_mode:
            return None
            
        return self.create_anaglyph()
    
    def get_detection_results(self):
        """获取最新的检测结果"""
        # 确保返回的结果是有效的YOLO结果对象，而不是空列表
        if isinstance(self.detection_results, list) and not self.detection_results:
            return None
        return self.detection_results
    
    def set_yolo_enabled(self, enabled):
        """启用或禁用YOLO检测"""
        self.yolo_enabled = enabled
        if enabled and self.yolo_model is None:
            self.load_yolo_model()
    
    def set_confidence_threshold(self, threshold):
        """设置置信度阈值"""
        self.confidence_threshold = max(0.1, min(1.0, threshold))
    
    def is_running(self):
        """检查捕获是否正在运行"""
        return self.running
        
    def set_stereo_mode(self, enabled, width=3840, height=1080):
        """设置是否使用双目模式"""
        was_running = self.running
        if was_running:
            self.stop()
            
        self.stereo_mode = enabled
        self.stereo_width = width
        self.stereo_height = height
        
        if was_running:
            self.start()
            
    def set_view_mode(self, mode):
        """设置视图模式
        
        Args:
            mode: 视图模式，可选值：normal, left, right, depth, anaglyph
        """
        self.view_mode = mode


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
        
        # 摄像头选择
        self.camera_list = []
        self.selected_camera = tk.IntVar(value=0)
        self.update_camera_list()
        
        # 双目相机设置
        self.stereo_mode = False
        self.stereo_var = tk.BooleanVar(value=False)  # 添加stereo_var变量
        self.camera_resolutions = [
            "1280x480 (30FPS)",
            "1920x1080 (30FPS)",
            "2160x1080 (30FPS)",
            "2560x720 (30FPS)",
            "3840x1080 (30FPS)",
            "3840x1520 (10FPS)"
        ]
        self.selected_resolution = tk.StringVar(value=self.camera_resolutions[0])
        
        # 视图模式
        self.view_modes = [
            "正常视图",
            "左目视图",
            "右目视图", 
            "深度视图",
            "3D立体视图"
        ]
        self.selected_view_mode = tk.StringVar(value=self.view_modes[0])
        
        # YOLO物体检测设置
        self.yolo_enabled = False
        self.yolo_model_path = ""
        self.detection_class_names = []  # 检测到的类别名称
        self.confidence_threshold = tk.DoubleVar(value=0.5)  # 置信度阈值
        
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
        
        # 视觉系统选项卡
        self.vision_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.vision_tab, text="视觉系统")
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
        """创建视觉系统标签页"""
        # 使用已创建的视觉系统标签页
        tab = self.vision_tab
        
        # 左侧面板 - 视频显示区域
        left_panel = ttk.Frame(tab)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # 视频画布
        self.video_canvas = tk.Canvas(left_panel, bg="black", width=640, height=480)
        self.video_canvas.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 相机控制按钮组
        camera_control_frame = ttk.LabelFrame(left_panel, text="相机控制")
        camera_control_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # 相机列表和控制
        camera_frame = ttk.Frame(camera_control_frame)
        camera_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(camera_frame, text="相机:").pack(side=tk.LEFT)
        self.camera_var = tk.StringVar()
        self.camera_combo = ttk.Combobox(camera_frame, textvariable=self.camera_var)
        self.camera_combo.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.camera_combo.bind("<<ComboboxSelected>>", self.on_camera_change)
        
        refresh_btn = ttk.Button(camera_frame, text="刷新", command=self.refresh_camera_list)
        refresh_btn.pack(side=tk.LEFT, padx=5)
        
        # 获取摄像头列表并填充下拉框
        self.update_camera_list()
        
        # 相机按钮组
        camera_btn_frame = ttk.Frame(camera_control_frame)
        camera_btn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 开启/关闭相机按钮
        self.camera_btn = ttk.Button(camera_btn_frame, text="开启相机", command=self.toggle_camera)
        self.camera_btn.pack(side=tk.LEFT, padx=5)
        
        # 双目模式切换按钮
        self.stereo_var = tk.BooleanVar(value=False)  # 确保在这里也定义了stereo_var
        self.stereo_btn = ttk.Checkbutton(camera_btn_frame, text="双目模式", variable=self.stereo_var, command=self.toggle_stereo_mode)
        self.stereo_btn.pack(side=tk.LEFT, padx=5)
        
        # 视图模式
        view_frame = ttk.Frame(camera_btn_frame)
        view_frame.pack(side=tk.LEFT, padx=5)
        ttk.Label(view_frame, text="视图:").pack(side=tk.LEFT)
        
        self.view_mode_combo = ttk.Combobox(view_frame, textvariable=self.selected_view_mode, values=self.view_modes, width=10)
        self.view_mode_combo.pack(side=tk.LEFT)
        self.view_mode_combo.bind("<<ComboboxSelected>>", lambda e: self.on_view_mode_change())
        
        # 右侧面板 - 控制区域
        right_panel = ttk.Frame(tab)
        right_panel.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=10)
        
        # 视觉处理区域
        vision_frame = ttk.LabelFrame(right_panel, text="视觉检测")
        vision_frame.pack(fill=tk.X, expand=False, pady=5)
        
        # YOLO检测开关
        yolo_frame = ttk.Frame(vision_frame)
        yolo_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.yolo_var = tk.BooleanVar(value=False)
        self.yolo_enable_check = ttk.Checkbutton(yolo_frame, text="启用YOLO检测", variable=self.yolo_var, 
                                     command=lambda: self.toggle_yolo_detection(self.yolo_var.get()))
        self.yolo_enable_check.pack(side=tk.LEFT)
        
        # 加载自定义模型按钮
        load_model_btn = ttk.Button(yolo_frame, text="加载模型", command=self.load_custom_yolo_model)
        load_model_btn.pack(side=tk.RIGHT)
        
        # 置信度阈值
        conf_frame = ttk.Frame(vision_frame)
        conf_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(conf_frame, text="置信度阈值:").pack(side=tk.LEFT)
        
        self.confidence_threshold = tk.DoubleVar(value=0.5)
        conf_scale = ttk.Scale(conf_frame, from_=0.1, to=1.0, length=150, 
                              variable=self.confidence_threshold,
                              command=self.update_confidence_threshold)
        conf_scale.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        # 显示当前置信度值
        self.conf_label = ttk.Label(conf_frame, text="0.5")
        self.conf_label.pack(side=tk.LEFT, padx=5)
        
        # 当置信度值变化时更新标签
        def update_conf_label(*args):
            self.conf_label.config(text=f"{self.confidence_threshold.get():.1f}")
        
        self.confidence_threshold.trace_add("write", update_conf_label)
        
        # 物体抓取区域
        grab_frame = ttk.LabelFrame(right_panel, text="物体抓取")
        grab_frame.pack(fill=tk.X, expand=False, pady=5)
        
        # 物体选择框架
        object_frame = ttk.Frame(grab_frame)
        object_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 检测到的物体标签
        ttk.Label(object_frame, text="检测到的物体:").pack(side=tk.LEFT)
        
        # 检测到的物体显示
        self.detected_objects_var = tk.StringVar(value="无")
        detected_objects_label = ttk.Label(object_frame, textvariable=self.detected_objects_var)
        detected_objects_label.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        target_frame = ttk.Frame(grab_frame)
        target_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(target_frame, text="目标序号:").pack(side=tk.LEFT)
        
        self.target_var = tk.StringVar(value="1")
        target_combo = ttk.Combobox(target_frame, textvariable=self.target_var, 
                                  values=["1", "2", "3", "4", "5", "6"], width=8)
        target_combo.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        # 坐标显示
        coords_frame = ttk.Frame(grab_frame)
        coords_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.coords_label = ttk.Label(coords_frame, text="X: 0.0mm, Y: 0.0mm, Z: 0.0mm")
        self.coords_label.pack(fill=tk.X)
        
        # 抓取控制按钮
        grab_btn_frame = ttk.Frame(grab_frame)
        grab_btn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        detect_btn = ttk.Button(grab_btn_frame, text="检测物体",
                               command=self.run_object_detection)
        detect_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        track_btn = ttk.Button(grab_btn_frame, text="跟踪物体", 
                              command=self.track_object)
        track_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        grab_btn = ttk.Button(grab_btn_frame, text="抓取物体",
                             command=self.grab_detected_object)
        grab_btn.pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        
        # 添加末端坐标控制区域
        cartesian_frame = ttk.LabelFrame(right_panel, text="末端坐标控制")
        cartesian_frame.pack(fill=tk.X, expand=False, pady=5)
        
        # X坐标控制
        x_frame = ttk.Frame(cartesian_frame)
        x_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(x_frame, text="X坐标(mm):").pack(side=tk.LEFT)
        self.x_var = tk.DoubleVar(value=250)  # 从0.25m改为250mm
        x_entry = ttk.Entry(x_frame, textvariable=self.x_var, width=6)
        x_entry.pack(side=tk.LEFT, padx=5)
        
        # X坐标微调按钮
        x_btn_frame = ttk.Frame(x_frame)
        x_btn_frame.pack(side=tk.LEFT, padx=5)
        ttk.Button(x_btn_frame, text="-", width=2, command=lambda: self.adjust_coordinate('x', -50)).pack(side=tk.LEFT)
        ttk.Button(x_btn_frame, text="+", width=2, command=lambda: self.adjust_coordinate('x', 50)).pack(side=tk.LEFT)
        
        # Y坐标控制
        y_frame = ttk.Frame(cartesian_frame)
        y_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(y_frame, text="Y坐标(mm):").pack(side=tk.LEFT)
        self.y_var = tk.DoubleVar(value=0.0)  # 0mm
        y_entry = ttk.Entry(y_frame, textvariable=self.y_var, width=6)
        y_entry.pack(side=tk.LEFT, padx=5)
        
        # Y坐标微调按钮
        y_btn_frame = ttk.Frame(y_frame)
        y_btn_frame.pack(side=tk.LEFT, padx=5)
        ttk.Button(y_btn_frame, text="-", width=2, command=lambda: self.adjust_coordinate('y', -50)).pack(side=tk.LEFT)
        ttk.Button(y_btn_frame, text="+", width=2, command=lambda: self.adjust_coordinate('y', 50)).pack(side=tk.LEFT)
        
        # Z坐标控制
        z_frame = ttk.Frame(cartesian_frame)
        z_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Label(z_frame, text="Z坐标(mm):").pack(side=tk.LEFT)
        self.z_var = tk.DoubleVar(value=200)  # 从0.2m改为200mm
        z_entry = ttk.Entry(z_frame, textvariable=self.z_var, width=6)
        z_entry.pack(side=tk.LEFT, padx=5)
        
        # Z坐标微调按钮
        z_btn_frame = ttk.Frame(z_frame)
        z_btn_frame.pack(side=tk.LEFT, padx=5)
        ttk.Button(z_btn_frame, text="-", width=2, command=lambda: self.adjust_coordinate('z', -50)).pack(side=tk.LEFT)
        ttk.Button(z_btn_frame, text="+", width=2, command=lambda: self.adjust_coordinate('z', 50)).pack(side=tk.LEFT)
        
        # 移动按钮
        move_btn_frame = ttk.Frame(cartesian_frame)
        move_btn_frame.pack(fill=tk.X, padx=5, pady=5)
        
        move_btn = ttk.Button(move_btn_frame, text="移动到坐标",
                             command=self.move_to_cartesian_from_vision)
        move_btn.pack(fill=tk.X, expand=True)
        
        # 获取当前位置按钮
        get_pos_btn = ttk.Button(move_btn_frame, text="获取当前位置",
                               command=self.get_current_position_to_vision)
        get_pos_btn.pack(fill=tk.X, expand=True, pady=2)
        
        # 日志区域
        log_frame = ttk.LabelFrame(right_panel, text="日志")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.vision_log = tk.Text(log_frame, height=10, width=30)
        self.vision_log.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 滚动条
        log_scrollbar = ttk.Scrollbar(self.vision_log)
        log_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.vision_log.config(yscrollcommand=log_scrollbar.set)
        log_scrollbar.config(command=self.vision_log.yview)
        
        # 禁用文本框编辑
        self.vision_log.config(state=tk.DISABLED)
    
    def move_to_cartesian_from_vision(self):
        """从视觉系统界面移动机械臂到指定笛卡尔坐标"""
        if not self.servo_connected and not self.motor_connected:
            self.log_to_vision("请先连接至少一个控制器")
            return
            
        try:
            # 获取输入的坐标值（毫米）
            x_mm = float(self.x_var.get())
            y_mm = float(self.y_var.get())
            z_mm = float(self.z_var.get())
            
            # 转换为米单位（机械臂控制函数使用米为单位）
            x = x_mm / 1000.0
            y = y_mm / 1000.0
            z = z_mm / 1000.0
            
            self.log_to_vision(f"正在移动到坐标: X={x_mm:.1f}mm, Y={y_mm:.1f}mm, Z={z_mm:.1f}mm")
            
            # 调用机械臂移动函数（使用米为单位）
            success = self.robot.move_to_cartesian_position(x, y, z, blocking=True, strict_error_check=False)
            
            if success:
                self.log_to_vision("已到达目标位置")
            else:
                self.log_to_vision("移动失败，可能超出工作范围")
                
        except Exception as e:
            self.log_to_vision(f"移动到坐标出错: {e}")
            import traceback
            self.log_to_vision(traceback.format_exc())
    
    def get_current_position_to_vision(self):
        """获取当前机械臂末端位置并显示在视觉系统界面"""
        if not self.servo_connected and not self.motor_connected:
            self.log_to_vision("请先连接至少一个控制器")
            return
            
        try:
            # 获取当前末端位置
            pose = self.robot.get_current_cartesian_position()
            
            if pose:
                # 将米转换为毫米并更新界面上的坐标值
                x_mm = pose['position'][0] * 1000
                y_mm = pose['position'][1] * 1000
                z_mm = pose['position'][2] * 1000
                
                self.x_var.set(round(x_mm, 1))
                self.y_var.set(round(y_mm, 1))
                self.z_var.set(round(z_mm, 1))
                
                self.log_to_vision(f"当前位置: X={x_mm:.1f}mm, Y={y_mm:.1f}mm, Z={z_mm:.1f}mm")
            else:
                self.log_to_vision("无法获取当前位置")
                
        except Exception as e:
            self.log_to_vision(f"获取当前位置时出错: {e}")
            import traceback
            self.log_to_vision(traceback.format_exc())
    
    def log_to_vision(self, message):
        """记录信息到视觉日志"""
        self.vision_log.config(state=tk.NORMAL)
        self.vision_log.insert(tk.END, f"{time.strftime('%H:%M:%S')} - {message}\n")
        self.vision_log.see(tk.END)
        self.vision_log.config(state=tk.DISABLED)
    
    def run_square_trajectory(self):
        """运行方形轨迹示例"""
        if not self.robot.connected:
            self.log("控制器未连接，无法执行方形轨迹")
            return
            
        self.log("开始执行方形轨迹...")
        
        try:
            # 获取当前位置作为起点
            current_pose = self.robot.get_current_cartesian_position()
            if current_pose is None:
                self.log("无法获取当前位置，轨迹执行失败")
                return
                
            start_x = current_pose['position'][0]
            start_y = current_pose['position'][1]
            start_z = current_pose['position'][2]
            
            # 方形边长(米)
            side_length = 0.05
            
            # 设置较低的速度和加速度
            original_speed = self.servo_speed_var.get()
            original_acc = self.acc_var.get()
            
            self.servo_speed_var.set(200)
            self.acc_var.set(3)
            self.update_speeds(None)
            self.update_acceleration(None)
            
            # 执行方形轨迹
            points = [
                (start_x + side_length, start_y, start_z),  # 右
                (start_x + side_length, start_y + side_length, start_z),  # 右上
                (start_x, start_y + side_length, start_z),  # 左上
                (start_x, start_y, start_z)   # 回到起点
            ]
            
            for i, (x, y, z) in enumerate(points):
                self.log(f"移动到方形轨迹点 {i+1}/4")
                self.robot.move_to_cartesian_position(x, y, z, blocking=True, strict_error_check=False)
                time.sleep(0.5)  # 在每个点停留片刻
            
            # 恢复原来的速度和加速度
            self.servo_speed_var.set(original_speed)
            self.acc_var.set(original_acc)
            self.update_speeds(None)
            self.update_acceleration(None)
            
            self.log("方形轨迹执行完成")
            
        except Exception as e:
            self.log(f"执行方形轨迹时出错: {str(e)}")
            
    def run_circle_trajectory(self):
        """运行圆形轨迹示例"""
        if not self.robot.connected:
            self.log("控制器未连接，无法执行圆形轨迹")
            return
            
        self.log("开始执行圆形轨迹...")
        
        try:
            # 获取当前位置作为圆心
            current_pose = self.robot.get_current_cartesian_position()
            if current_pose is None:
                self.log("无法获取当前位置，轨迹执行失败")
                return
                
            center_x = current_pose['position'][0]
            center_y = current_pose['position'][1]
            center_z = current_pose['position'][2]
            
            # 圆的半径(米)
            radius = 0.05
            
            # 设置较低的速度和加速度
            original_speed = self.servo_speed_var.get()
            original_acc = self.acc_var.get()
            
            self.servo_speed_var.set(200)
            self.acc_var.set(3)
            self.update_speeds(None)
            self.update_acceleration(None)
            
            # 执行圆形轨迹
            num_points = 12  # 圆周上的点数
            for i in range(num_points + 1):  # +1是为了回到起点
                angle = 2 * math.pi * i / num_points
                x = center_x + radius * math.cos(angle)
                y = center_y + radius * math.sin(angle)
                z = center_z
                
                self.log(f"移动到圆形轨迹点 {i+1}/{num_points+1}")
                self.robot.move_to_cartesian_position(x, y, z, blocking=True)
                time.sleep(0.2)  # 短暂停留
            
            # 恢复原来的速度和加速度
            self.servo_speed_var.set(original_speed)
            self.acc_var.set(original_acc)
            self.update_speeds(None)
            self.update_acceleration(None)
            
            self.log("圆形轨迹执行完成")
            
        except Exception as e:
            self.log(f"执行圆形轨迹时出错: {str(e)}")
            
    def run_wave_trajectory(self):
        """运行波浪轨迹示例"""
        if not self.robot.connected:
            self.log("控制器未连接，无法执行波浪轨迹")
            return
            
        self.log("开始执行波浪轨迹...")
        
        try:
            # 获取当前位置作为起点
            current_pose = self.robot.get_current_cartesian_position()
            if current_pose is None:
                self.log("无法获取当前位置，轨迹执行失败")
                return
                
            start_x = current_pose['position'][0]
            start_y = current_pose['position'][1]
            start_z = current_pose['position'][2]
            
            # 波浪参数
            length = 0.1  # 波浪长度(米)
            amplitude = 0.03  # 波浪振幅(米)
            
            # 设置较低的速度和加速度
            original_speed = self.servo_speed_var.get()
            original_acc = self.acc_var.get()
            
            self.servo_speed_var.set(200)
            self.acc_var.set(3)
            self.update_speeds(None)
            self.update_acceleration(None)
            
            # 执行波浪轨迹
            num_points = 20
            for i in range(num_points + 1):  # +1是为了回到起点
                t = i / num_points
                x = start_x + length * t
                y = start_y + amplitude * math.sin(2 * math.pi * t * 2)  # 2个完整波
                z = start_z
                
                self.log(f"移动到波浪轨迹点 {i+1}/{num_points+1}")
                self.robot.move_to_cartesian_position(x, y, z, blocking=True)
                time.sleep(0.1)  # 短暂停留
            
            # 回到起点
            self.robot.move_to_cartesian_position(start_x, start_y, start_z, blocking=True, strict_error_check=False)
            
            # 恢复原来的速度和加速度
            self.servo_speed_var.set(original_speed)
            self.acc_var.set(original_acc)
            self.update_speeds(None)
            self.update_acceleration(None)
            
            self.log("波浪轨迹执行完成")
            
        except Exception as e:
            self.log(f"执行波浪轨迹时出错: {str(e)}")

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
        style.configure("Red.TButton", foreground="white", background="red", font=("Arial", 10, "bold"))
        
        # 示例动作面板
        presets_frame = ttk.LabelFrame(parent, text="示例动作")
        presets_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 示例动作按钮
        presets_button_frame = ttk.Frame(presets_frame)
        presets_button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 方形轨迹示例
        square_button = ttk.Button(presets_button_frame, text="方形轨迹", 
                                  command=lambda: self.run_square_trajectory())
        square_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # 圆形轨迹示例
        circle_button = ttk.Button(presets_button_frame, text="圆形轨迹", 
                                  command=lambda: self.run_circle_trajectory())
        circle_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # 波浪轨迹示例
        wave_button = ttk.Button(presets_button_frame, text="波浪轨迹", 
                                command=lambda: self.run_wave_trajectory())
        wave_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
    
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
                
            # 更新笛卡尔坐标显示
            if self.servo_connected or self.motor_connected:
                self.update_cartesian_display()
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
            
    def update_cartesian_display(self):
        """更新笛卡尔坐标显示"""
        try:
            # 获取当前末端位置
            pose = self.robot.get_current_cartesian_position()
            
            if pose:
                # 将米转换为毫米
                x_mm = pose['position'][0] * 1000
                y_mm = pose['position'][1] * 1000
                z_mm = pose['position'][2] * 1000
                
                # 更新坐标显示标签
                self.coords_label.config(text=f"X: {x_mm:.1f}mm, Y: {y_mm:.1f}mm, Z: {z_mm:.1f}mm")
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
            
            # 设置较低的速度和加速度，避免过载
            if self.servo_connected:
                self.robot.set_speeds(servo_speed=500)  # 增加时间，降低速度
                self.robot.set_all_servo_acc(2)  # 降低加速度
                
            if self.motor_connected:
                self.robot.motor.set_motor_speed(50, 50)  # 降低电机速度
            
            # 尝试回到初始位置，但捕获可能的过载错误
            try:
                success = self.robot.home(blocking=True)
                if success:
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
                error_str = str(e)
                if "Overload error" in error_str:
                    self.log("警告：舵机过载！可能是负载过大或电压不稳定")
                    messagebox.showwarning("舵机过载", "检测到舵机过载错误！\n\n可能原因：\n1. 机械臂负载过大\n2. 电源电压不足\n3. 舵机运动速度过快\n\n建议：\n1. 检查电源\n2. 减小负载\n3. 降低速度和加速度")
                    
                    # 尝试紧急停止
                    self.emergency_stop()
                else:
                    raise  # 重新抛出其他类型的错误
                
        except Exception as e:
            self.log(f"回到初始位置时出错: {e}")
            
            # 如果出现严重错误，尝试紧急停止
            try:
                self.emergency_stop()
            except:
                pass
    
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
                self.robot.set_speeds(servo_speed=500)  # 增加时间，降低速度
                self.robot.set_all_servo_acc(2)  # 降低加速度
                self.servo_speed_var.set(500)
                self.acc_var.set(2)
            
            if self.motor_connected:
                self.robot.motor.set_motor_speed(50, 50)  # 降低电机速度
                self.pitch_speed_var.set(50)
                self.linear_speed_var.set(50)
            
            # 返回初始位置，使用修改后的home_robot方法（已添加错误处理）
            self.home_robot()
            
            self.log("校准完成")
        except Exception as e:
            error_str = str(e)
            if "Overload error" in error_str:
                self.log("警告：校准过程中舵机过载！可能是负载过大或电压不稳定")
                messagebox.showwarning("舵机过载", "校准过程中检测到舵机过载错误！\n\n可能原因：\n1. 机械臂负载过大\n2. 电源电压不足\n3. 舵机运动速度过快\n\n建议：\n1. 检查电源\n2. 减小负载\n3. 降低速度和加速度")
                
                # 尝试紧急停止
                self.emergency_stop()
            else:
                self.log(f"校准时出错: {e}")
                
                # 如果出现严重错误，尝试紧急停止
                try:
                    self.emergency_stop()
                except:
                    pass
    
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
    
    # 删除move_cartesian函数
    
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
            # 停止摄像头
            self.video.stop()
            self.video_enabled = False
            self.camera_btn.config(text="开启摄像头")
            self.log("摄像头已关闭")
        else:
            # 获取选定的摄像头索引
            camera_index = self.selected_camera.get()
            
            # 更新摄像头索引
            self.video.camera_index = camera_index
            
            # 启动摄像头
            if self.video.start():
                self.video_enabled = True
                self.camera_btn.config(text="关闭摄像头")
                self.log(f"摄像头已开启 (索引: {camera_index})")
                
                # 启动视频更新
                self.update_video_frame()
            else:
                messagebox.showerror("错误", "无法开启摄像头，请检查摄像头连接")
    
    def toggle_stereo_mode(self):
        """切换双目相机模式"""
        stereo_enabled = self.stereo_var.get()
        
        # 如果摄像头正在运行，需要先停止
        was_running = self.video_enabled
        if was_running:
            self.video.stop()
            self.video_enabled = False
        
        # 解析当前选择的分辨率
        resolution_str = self.selected_resolution.get().split(" ")[0]  # 例如 "3840x1080 (30FPS)" -> "3840x1080"
        width, height = map(int, resolution_str.split("x"))
        
        # 设置双目模式
        self.stereo_mode = stereo_enabled
        self.video.set_stereo_mode(stereo_enabled, width, height)
        
        # 如果之前在运行，重新启动摄像头
        if was_running:
            self.toggle_camera()
        
        self.log(f"双目相机模式: {'已启用' if stereo_enabled else '已禁用'}")
    
    def on_resolution_change(self, event):
        """处理分辨率变化"""
        if not self.stereo_mode:
            return
            
        # 解析新的分辨率
        resolution_str = self.selected_resolution.get().split(" ")[0]
        width, height = map(int, resolution_str.split("x"))
        
        # 如果摄像头正在运行，需要先停止
        was_running = self.video_enabled
        if was_running:
            self.video.stop()
            self.video_enabled = False
        
        # 更新分辨率
        self.video.set_stereo_mode(True, width, height)
        
        # 如果之前在运行，重新启动摄像头
        if was_running:
            self.toggle_camera()
        
        self.log(f"分辨率已设置为 {width}x{height}")
    
    def toggle_yolo_detection(self, enabled):
        """启用或禁用YOLO物体检测"""
        self.yolo_enabled = enabled
        self.video.set_yolo_enabled(enabled)
        
        if enabled and self.video_enabled:
            self.log("正在加载YOLO模型，请稍候...")
        else:
            self.log(f"YOLO物体检测: {'已启用' if enabled else '已禁用'}")
    
    def update_confidence_threshold(self, value):
        """更新置信度阈值"""
        threshold = float(value)
        self.video.set_confidence_threshold(threshold)
    
    def load_custom_yolo_model(self):
        """加载自定义YOLO模型"""
        from tkinter import filedialog
        
        model_path = filedialog.askopenfilename(
            title="选择YOLO模型文件",
            filetypes=[("PyTorch模型", "*.pt"), ("ONNX模型", "*.onnx"), ("所有文件", "*.*")]
        )
        
        if model_path:
            self.yolo_model_path = model_path
            self.log(f"正在加载自定义模型: {os.path.basename(model_path)}")
            self.video.load_yolo_model(model_path)
            
            # 自动启用YOLO检测
            self.yolo_enable_check.state(['selected'])
            self.toggle_yolo_detection(True)
    
    def update_video_frame(self):
        """更新视频帧"""
        if not self.video_enabled:
            return
            
        try:
            # 获取当前帧
            frame = self.video.get_frame()
            
            if frame is not None:
                display_frame = None
                
                # 根据视图模式和双目模式选择显示的帧
                view_mode = self.selected_view_mode.get()
                
                if self.stereo_mode:
                    left_frame, right_frame = self.video.get_stereo_frames()
                    
                    if view_mode == "正常视图":
                        # 在左帧上绘制检测结果
                        if self.yolo_enabled and left_frame is not None:
                            left_display = self.draw_detection_results(left_frame.copy())
                        elif self.vision_processing_enabled and left_frame is not None:
                            left_display = self.process_frame(left_frame.copy())
                        else:
                            left_display = left_frame.copy() if left_frame is not None else None
                            
                        # 将左右帧并排显示
                        if left_display is not None and right_frame is not None:
                            h, w = left_display.shape[:2]
                            combined_frame = np.zeros((h, w*2, 3), dtype=np.uint8)
                            combined_frame[:, :w] = left_display
                            combined_frame[:, w:] = right_frame
                            display_frame = combined_frame
                    
                    elif view_mode == "左目视图":
                        # 只显示左目视图
                        if left_frame is not None:
                            if self.yolo_enabled:
                                display_frame = self.draw_detection_results(left_frame.copy())
                            elif self.vision_processing_enabled:
                                display_frame = self.process_frame(left_frame.copy())
                            else:
                                display_frame = left_frame.copy()
                    
                    elif view_mode == "右目视图":
                        # 只显示右目视图
                        if right_frame is not None:
                            display_frame = right_frame.copy()
                    
                    elif view_mode == "深度视图":
                        # 显示深度图
                        depth_frame = self.video.get_depth_frame()
                        if depth_frame is not None:
                            display_frame = depth_frame
                    
                    elif view_mode == "3D立体视图":
                        # 显示红蓝3D立体图
                        anaglyph_frame = self.video.get_anaglyph_frame()
                        if anaglyph_frame is not None:
                            display_frame = anaglyph_frame
                else:
                    # 单目模式
                    if self.yolo_enabled:
                        display_frame = self.draw_detection_results(frame.copy())
                    elif self.vision_processing_enabled:
                        display_frame = self.process_frame(frame.copy())
                    else:
                        display_frame = frame.copy()
                
                # 如果没有有效的显示帧，使用原始帧
                if display_frame is None:
                    display_frame = frame.copy()
                
                # 获取画布当前尺寸
                canvas_width = self.video_canvas.winfo_width()
                canvas_height = self.video_canvas.winfo_height()
                
                # 确保画布尺寸有效（初始化时可能为1）
                if canvas_width <= 1:
                    canvas_width = 640
                if canvas_height <= 1:
                    canvas_height = 480
                
                # 调整大小以适应画布显示
                display_frame = cv2.resize(display_frame, (canvas_width, canvas_height))
                
                # 转换颜色空间
                display_frame = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
                
                # 转换为PhotoImage
                image = Image.fromarray(display_frame)
                photo = ImageTk.PhotoImage(image=image)
                
                # 更新画布
                self.video_canvas.create_image(0, 0, anchor=tk.NW, image=photo)
                self.video_canvas.image = photo  # 保持引用
        except Exception as e:
            self.log(f"更新视频帧时出错: {e}")
            
        # 继续更新
        if self.video_enabled:
            self.root.after(self.video_update_ms, self.update_video_frame)
    
    def draw_detection_results(self, frame):
        """在帧上绘制YOLO检测结果"""
        if not self.yolo_enabled:
            return frame
            
        results = self.video.get_detection_results()
        if results is None:
            if hasattr(self, 'detected_objects_var'):
                self.detected_objects_var.set("无")
            return frame
            
        try:
            # 检测到的物体列表
            detected_objects = []
            
            # 处理YOLOv11n的结果格式（Results对象）
            if hasattr(results, 'boxes'):
                # 获取检测框
                boxes = results.boxes
                
                # 遍历所有检测结果
                for i, box in enumerate(boxes):
                    # 获取边界框坐标
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # 获取置信度
                    conf = float(box.conf[0])
                    
                    # 如果置信度低于阈值，跳过
                    if conf < self.confidence_threshold.get():
                        continue
                    
                    # 获取类别ID和名称
                    cls_id = int(box.cls[0])
                    # 使用VideoCapture类中保存的类别名称
                    if hasattr(self.video, 'class_names') and cls_id in self.video.class_names:
                        cls_name = self.video.class_names[cls_id]
                    else:
                        cls_name = f"类别{cls_id}"
                    
                    # 计算物体中心点
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # 添加到检测到的物体列表
                    detected_objects.append(f"{i+1}:{cls_name}({conf:.2f})")
                    
                    # 绘制边界框
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # 绘制类别名称、置信度和编号
                    label = f"{i+1}:{cls_name} {conf:.2f}"
                    (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(frame, (x1, y1-label_height-5), (x1+label_width, y1), (0, 255, 0), -1)
                    cv2.putText(frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
                    
                    # 绘制中心点和坐标
                    cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)
                    coord_label = f"({center_x},{center_y})"
                    cv2.putText(frame, coord_label, (center_x + 5, center_y + 15), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            else:
                # 不支持的结果格式
                print(f"不支持的YOLO结果格式: {type(results)}")
                if hasattr(self, 'detected_objects_var'):
                    self.detected_objects_var.set("无法识别的结果格式")
            
            # 更新检测到的物体显示
            if hasattr(self, 'detected_objects_var'):
                if detected_objects:
                    self.detected_objects_var.set(", ".join(detected_objects))
                else:
                    self.detected_objects_var.set("无")
                    
        except Exception as e:
            print(f"绘制检测结果时出错: {e}")
            
        return frame
        
    def adjust_coordinate(self, axis, delta):
        """调整笛卡尔坐标"""
        if axis == 'x':
            new_value = self.x_var.get() + delta
            # 限制X坐标范围在100到500mm之间
            new_value = max(100, min(500, new_value))
            self.x_var.set(new_value)
        elif axis == 'y':
            new_value = self.y_var.get() + delta
            # 限制Y坐标范围在-300到300mm之间
            new_value = max(-300, min(300, new_value))
            self.y_var.set(new_value)
        elif axis == 'z':
            new_value = self.z_var.get() + delta
            # 限制Z坐标范围在50到400mm之间
            new_value = max(50, min(400, new_value))
            self.z_var.set(new_value)
    
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
    
    def run_object_detection(self):
        """运行物体检测"""
        if not self.video_enabled:
            messagebox.showinfo("提示", "请先开启摄像头")
            return
            
        # 启用YOLO检测
        self.yolo_enable_check.state(['selected'])
        self.toggle_yolo_detection(True)
        self.log("已启动物体检测")
    
    def track_object(self):
        """跟踪检测到的物体"""
        if not self.video_enabled or not self.yolo_enabled:
            messagebox.showinfo("提示", "请先开启摄像头并启用YOLO检测")
            return
            
        # 获取当前检测结果
        results = self.video.get_detection_results()
        if results is None:
            self.log("未检测到物体，无法跟踪")
            return
            
        # 处理YOLOv11n的结果格式（Results对象）
        if hasattr(results, 'boxes'):
            boxes = results.boxes
            if len(boxes) == 0:  # 没有检测到物体
                self.log("未检测到物体，无法跟踪")
                return
                
            # 找到置信度最高的物体
            best_box = None
            best_conf = 0
            
            for box in boxes:
                conf = float(box.conf[0])
                if conf > best_conf and conf >= self.confidence_threshold.get():
                    best_conf = conf
                    best_box = box
            
            if best_box is None:
                self.log("未找到符合置信度要求的物体")
                return
                
            # 获取物体位置
            x1, y1, x2, y2 = map(int, best_box.xyxy[0])
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            # 获取物体类别
            cls_id = int(best_box.cls[0])
            # 使用VideoCapture类中保存的类别名称
            if hasattr(self.video, 'class_names') and cls_id in self.video.class_names:
                cls_name = self.video.class_names[cls_id]
            else:
                # 如果在VideoCapture类中没有找到，尝试从results中获取
                if hasattr(results, 'names') and cls_id in results.names:
                    cls_name = results.names[cls_id]
                else:
                    cls_name = f"类别{cls_id}"
        else:
            self.log("检测结果格式不支持，只支持YOLOv11n格式的结果")
            return
        
        self.log(f"正在跟踪物体: {cls_name}, 位置: ({center_x}, {center_y})")
        
        # TODO: 实现物体跟踪逻辑
        # 这里可以添加控制机械臂移动到物体位置的代码
    
    def grab_detected_object(self):
        """抓取检测到的物体"""
        if not self.robot or not self.robot.connected:
            messagebox.showerror("错误", "机械臂未连接")
            return
            
        # 确保双目相机已初始化
        if not hasattr(self.robot, 'cameras_initialized') or not self.robot.cameras_initialized:
            # 尝试初始化双目相机
            self.log("正在初始化双目相机...")
            if not self.robot.init_stereo_cameras():
                messagebox.showerror("错误", "无法初始化双目相机")
                return
        
        # 使用YOLO检测物体
        if self.yolo_enabled:
            # 获取当前检测结果
            results = self.video.get_detection_results()
            if results is None:
                self.log("未检测到物体，无法抓取")
                messagebox.showinfo("提示", "未检测到物体，无法抓取")
                self.detected_objects_var.set("无")
                return
                
            # 获取所有符合置信度要求的物体
            valid_boxes = []
            detected_objects = []
            
            # 处理YOLOv11n的结果格式
            if hasattr(results, 'boxes'):
                boxes = results.boxes
                for i, box in enumerate(boxes):
                    # 获取置信度
                    conf = float(box.conf[0])
                    
                    if conf >= self.confidence_threshold.get():
                        # 获取类别ID和名称
                        cls_id = int(box.cls[0])
                        # 使用VideoCapture类中保存的类别名称
                        if hasattr(self.video, 'class_names') and cls_id in self.video.class_names:
                            cls_name = self.video.class_names[cls_id]
                        else:
                            # 如果在VideoCapture类中没有找到，尝试从results中获取
                            if hasattr(results, 'names') and cls_id in results.names:
                                cls_name = results.names[cls_id]
                            else:
                                cls_name = f"类别{cls_id}"
                            
                        # 创建box对象
                        box_info = {
                            'xyxy': [box.xyxy[0]],  # 边界框坐标
                            'conf': conf,           # 置信度
                            'cls': [cls_id]         # 类别ID
                        }
                        
                        valid_boxes.append({
                            'index': i,
                            'box': box_info,
                            'conf': conf,
                            'cls_id': cls_id,
                            'cls_name': cls_name
                        })
                        detected_objects.append(f"{i+1}:{cls_name}({conf:.2f})")
            else:
                self.log("检测结果格式不支持，只支持YOLOv11n格式的结果")
                messagebox.showinfo("提示", "检测结果格式不支持，只支持YOLOv11n格式的结果")
                self.detected_objects_var.set("无")
                return
            
            if not valid_boxes:
                self.log("未找到符合置信度要求的物体")
                messagebox.showinfo("提示", "未找到符合置信度要求的物体")
                self.detected_objects_var.set("无")
                return
            
            # 更新检测到的物体显示
            self.detected_objects_var.set(", ".join(detected_objects))
                
            # 如果有多个物体，让用户选择要抓取的物体序号
            target_index = 0
            if len(valid_boxes) > 1:
                options = [f"{i+1}. {box['cls_name']} (置信度: {box['conf']:.2f})" for i, box in enumerate(valid_boxes)]
                choice = simpledialog.askstring(
                    "选择物体",
                    "检测到多个物体，请选择要抓取的物体序号:",
                    initialvalue="1"
                )
                
                if not choice:
                    return
                    
                try:
                    target_index = int(choice) - 1
                    if target_index < 0 or target_index >= len(valid_boxes):
                        target_index = 0
                except ValueError:
                    target_index = 0
            
            # 获取选中的物体信息
            selected_box = valid_boxes[target_index]
            cls_name = selected_box['cls_name']
            
            # 更新选中的物体显示
            self.detected_objects_var.set(f"已选择: {target_index+1}:{cls_name}({selected_box['conf']:.2f})")
            
            # 禁用界面控件
            self.disable_controls()
            self.log(f"正在执行抓取 {cls_name} 物体的操作...")
            
            # 创建异步任务
            def grab_task():
                # 将YOLO检测框传递给grab_object_with_vision函数
                box_xyxy = selected_box['box']['xyxy'][0]
                success = self.robot.grab_object_with_vision(object_name=cls_name, yolo_bbox=box_xyxy)
                return success
        else:
            # YOLO未启用，使用传统的颜色检测方法
            # 让用户选择要抓取的物体序号
            target_options = ["1", "2", "3", "4", "5", "6"]
            target_index = simpledialog.askstring(
                "选择目标",
                "请选择要抓取的目标序号(1-6):",
                initialvalue=target_options[0]
            )
            
            if not target_index:
                return
                
            # 确保输入是有效的序号
            try:
                target_num = int(target_index)
                if target_num < 1 or target_num > 6:
                    target_num = 1
            except ValueError:
                target_num = 1
                
            # 将序号转换为名称，用于显示
            cls_name = f"目标{target_num}"
                
            # 禁用界面控件
            self.disable_controls()
            self.log(f"正在执行抓取{cls_name}的操作...")
            
            # 创建异步任务
            def grab_task():
                success = self.robot.grab_object_with_vision(object_name=str(target_num))
                return success
            
        def on_complete(success, error=None):
            if error:
                self.log(f"抓取过程中发生错误: {error}")
                messagebox.showerror("错误", f"抓取失败: {error}")
                self.detected_objects_var.set(f"抓取失败: {cls_name}")
            elif success:
                self.log(f"成功抓取 {cls_name} 物体")
                messagebox.showinfo("成功", f"成功抓取 {cls_name} 物体")
                self.detected_objects_var.set(f"抓取成功: {cls_name}")
            else:
                self.log(f"抓取 {cls_name} 物体失败")
                messagebox.showerror("错误", f"抓取 {cls_name} 物体失败")
                self.detected_objects_var.set(f"抓取失败: {cls_name}")
                
            # 重新启用界面控件
            self.enable_controls()
        
        # 创建并启动任务
        task = AsyncTask(callback=on_complete)
        task.task = grab_task
        task.start()
    
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

    def update_camera_list(self):
        """更新可用摄像头列表"""
        try:
            self.camera_list = VideoCapture.get_camera_list()
            # 如果没有找到任何摄像头，至少添加一个默认摄像头
            if not self.camera_list:
                self.camera_list.append({
                    "index": 0,
                    "name": "默认摄像头",
                    "resolution": "未知",
                    "fps": 0
                })
        except Exception as e:
            self.log(f"获取摄像头列表时出错: {e}")
            # 添加一个默认摄像头
            self.camera_list = [{
                "index": 0,
                "name": "默认摄像头",
                "resolution": "未知",
                "fps": 0
            }]
    
    def refresh_camera_list(self):
        """刷新摄像头列表"""
        # 如果摄像头正在运行，先停止
        was_running = self.video_enabled
        if was_running:
            self.toggle_camera()
            
        # 更新摄像头列表
        self.update_camera_list()
        
        # 更新下拉框内容
        camera_options = []
        for cam in self.camera_list:
            camera_options.append(f"{cam['name']} ({cam['resolution']})")
            
        self.camera_combo['values'] = camera_options
        if camera_options:
            self.camera_combo.current(0)
            
        self.log("摄像头列表已刷新")
        
    def on_camera_change(self, event):
        """处理摄像头选择变化"""
        # 获取选择的索引
        selected_index = self.camera_combo.current()
        if selected_index < 0 or selected_index >= len(self.camera_list):
            return
            
        # 如果摄像头正在运行，需要先停止
        was_running = self.video_enabled
        if was_running:
            self.toggle_camera()
            
        # 更新摄像头索引
        camera_index = self.camera_list[selected_index]["index"]
        self.selected_camera.set(camera_index)
        self.video.camera_index = camera_index
        
        # 如果之前在运行，重新启动摄像头
        if was_running:
            self.toggle_camera()
            
        self.log(f"已选择摄像头: {self.camera_list[selected_index]['name']}")
        
    def on_view_mode_change(self):
        """处理视图模式变化"""
        mode = self.selected_view_mode.get()
        
        # 映射UI视图模式到VideoCapture类中的视图模式
        mode_mapping = {
            "正常视图": "normal",
            "左目视图": "left",
            "右目视图": "right",
            "深度视图": "depth",
            "3D立体视图": "anaglyph"
        }
        
        # 设置视图模式
        if mode in mode_mapping:
            self.video.set_view_mode(mode_mapping[mode])
            
        self.log(f"视图模式已切换为: {mode}")

    def disable_controls(self):
        """禁用所有控件"""
        for tab in self.notebook.tabs():
            for widget in tab.winfo_children():
                if isinstance(widget, (tk.Button, tk.Entry, ttk.Button, ttk.Entry)):
                    widget.configure(state="disabled")
                    
        # 禁用菜单项
        for index in range(self.menu_bar.index("end") or 0):
            self.menu_bar.entryconfigure(index, state="disabled")
    
    def enable_controls(self):
        """启用所有控件"""
        for tab in self.notebook.tabs():
            for widget in tab.winfo_children():
                if isinstance(widget, (tk.Button, ttk.Button)):
                    widget.configure(state="normal")
                elif isinstance(widget, (tk.Entry, ttk.Entry)):
                    widget.configure(state="normal")
                    
        # 启用菜单项
        for index in range(self.menu_bar.index("end") or 0):
            self.menu_bar.entryconfigure(index, state="normal")


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