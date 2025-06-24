#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import threading
import os

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 图像处理中的参数
        self.log_level = rospy.get_param('~log_level', 1)  # 0=详细, 1=标准, 2=只有错误
        self.camera_device = rospy.get_param('~camera_device', '/dev/video0')  # 使用单个摄像头
        self.resolution_width = rospy.get_param('~resolution_width', 1280)
        self.resolution_height = rospy.get_param('~resolution_height', 480)
        self.fps = rospy.get_param('~fps', 30)
        
        # 检查摄像头设备是否存在
        self.camera_exists = os.path.exists(self.camera_device)
        
        # 如果指定的设备不存在，尝试查找可用的摄像头设备
        if not self.camera_exists:
            self.find_available_cameras()
        
        # 线程锁
        self.camera_lock = threading.Lock()
        
        # 存储最新的图像
        self.camera_image = None
        self.camera_timestamp = 0
        
        # 统计信息
        self.frame_counter = 0
        self.last_log_time = time.time()
        self.camera_fps = 0
        self.camera_frame_count = 0
        
        # 发布器 - 发布到多个话题以兼容现有系统
        self.camera_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        
        # 启动摄像头线程
        self.running = True
        
        if self.camera_exists:
            self.camera_thread = threading.Thread(target=self.camera_loop)
            self.camera_thread.start()
            rospy.loginfo("摄像头线程已启动")
        else:
            rospy.logerr("未找到可用的摄像头设备！")
    
    def find_available_cameras(self):
        """查找系统中可用的摄像头设备"""
        available_cameras = []
        
        # 在Linux系统中，查找/dev/video*设备
        for i in range(10):  # 通常摄像头设备编号不会超过10
            device_path = f"/dev/video{i}"
            if os.path.exists(device_path):
                available_cameras.append(device_path)
        
        rospy.loginfo(f"找到的可用摄像头设备: {available_cameras}")
        
        # 如果有可用摄像头，使用第一个
        if available_cameras:
            self.camera_device = available_cameras[0]
            self.camera_exists = True
            rospy.loginfo(f"自动选择摄像头设备: {self.camera_device}")
    
    def log_info(self, msg):
        """根据日志级别决定是否输出信息"""
        if self.log_level <= 1:
            rospy.loginfo(msg)
    
    def log_debug(self, msg):
        """只在详细日志模式输出调试信息"""
        if self.log_level == 0:
            rospy.logdebug(msg)
    
    def camera_loop(self):
        """摄像头读取循环"""
        cap = cv2.VideoCapture(self.camera_device)
        if not cap.isOpened():
            rospy.logerr("无法打开摄像头: %s", self.camera_device)
            self.camera_exists = False
            return
        
        # 设置摄像头参数
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # 尝试MJPG格式，提高帧率
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
        
        # 获取实际设置的参数
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        
        rospy.loginfo("摄像头已启动：%s", self.camera_device)
        rospy.loginfo("实际分辨率: %dx%d, 实际帧率: %.1f", actual_width, actual_height, actual_fps)
        
        # 是否需要裁剪左右两部分（针对宽屏图像的情况）
        wide_screen = (actual_width >= actual_height * 1.9)
        if wide_screen:
            rospy.loginfo("检测到宽幅图像，将自动显示左半部分")
        
        retry_count = 0
        max_retries = 5
        
        while self.running and not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                # 如果是宽幅图像（宽高比大于1.9:1），只取左半部分
                if wide_screen and frame is not None:
                    frame_width = frame.shape[1]
                    half_width = frame_width // 2
                    # 只使用左半部分
                    frame = frame[:, :half_width].copy()
                
                with self.camera_lock:
                    self.camera_image = frame.copy()
                    self.camera_timestamp = time.time()
                    self.camera_frame_count += 1
                
                # 发布图像
                try:
                    # 当前时间戳
                    timestamp = rospy.Time.now()
                    
                    # 发布到图像话题
                    camera_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    camera_msg.header.stamp = timestamp
                    camera_msg.header.frame_id = "camera_link"
                    self.camera_pub.publish(camera_msg)
                    
                    retry_count = 0  # 成功读取，重置重试计数
                except CvBridgeError as e:
                    rospy.logerr("发布图像时出错: %s", e)
            else:
                rospy.logwarn("摄像头读取失败")
                retry_count += 1
                if retry_count > max_retries:
                    rospy.logerr("摄像头连续读取失败 %d 次，尝试重新打开", max_retries)
                    cap.release()
                    time.sleep(1.0)
                    cap = cv2.VideoCapture(self.camera_device)
                    if not cap.isOpened():
                        rospy.logerr("无法重新打开摄像头，中止")
                        break
                    retry_count = 0
            
            # 定期报告统计信息
            current_time = time.time()
            if current_time - self.last_log_time > 5.0:
                if self.camera_frame_count > 0:
                    self.camera_fps = self.camera_frame_count / (current_time - self.last_log_time)
                
                self.log_info(f"摄像头帧率: {self.camera_fps:.2f} FPS")
                
                self.camera_frame_count = 0
                self.last_log_time = current_time
            
            # 限制循环速度，避免CPU占用过高
            time.sleep(1.0 / (self.fps * 2))
        
        cap.release()
        rospy.loginfo("摄像头已关闭")
    
    def shutdown(self):
        """关闭节点"""
        self.running = False
        rospy.loginfo("正在关闭摄像头节点...")

if __name__ == '__main__':
    try:
        camera_node = CameraNode()
        rospy.on_shutdown(camera_node.shutdown)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("用户中断，退出")
    except Exception as e:
        rospy.logerr("未知错误: %s", e)