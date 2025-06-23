#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import threading

class StereoMerger:
    def __init__(self):
        rospy.init_node('stereo_merger', anonymous=True)
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 图像处理中的参数
        self.log_level = rospy.get_param('~log_level', 1)  # 0=详细, 1=标准, 2=只有错误
        self.left_device = rospy.get_param('~left_device', '/dev/video0')
        self.right_device = rospy.get_param('~right_device', '/dev/video1')
        self.resolution_width = rospy.get_param('~resolution_width', 640)
        self.resolution_height = rospy.get_param('~resolution_height', 480)
        self.fps = rospy.get_param('~fps', 30)
        
        # 线程锁
        self.left_lock = threading.Lock()
        self.right_lock = threading.Lock()
        
        # 存储最新的左右图像
        self.left_image = None
        self.right_image = None
        self.left_timestamp = 0
        self.right_timestamp = 0
        
        # 统计信息
        self.frame_counter = 0
        self.last_log_time = time.time()
        self.left_fps = 0
        self.right_fps = 0
        self.left_frame_count = 0
        self.right_frame_count = 0
        
        # 发布器
        self.merged_pub = rospy.Publisher('/stereo_camera/image_merged', Image, queue_size=1)
        self.left_pub = rospy.Publisher('/left_camera/image_raw', Image, queue_size=1)
        self.right_pub = rospy.Publisher('/right_camera/image_raw', Image, queue_size=1)
        
        # 启动摄像头线程
        self.running = True
        self.left_thread = threading.Thread(target=self.left_camera_loop)
        self.right_thread = threading.Thread(target=self.right_camera_loop)
        self.merge_thread = threading.Thread(target=self.merge_loop)
        
        self.left_thread.start()
        self.right_thread.start()
        self.merge_thread.start()
        
        rospy.loginfo("双目摄像头合并器已初始化")
        rospy.loginfo("左摄像头: %s, 右摄像头: %s", self.left_device, self.right_device)
        rospy.loginfo("分辨率: %dx%d, 帧率: %d", self.resolution_width, self.resolution_height, self.fps)
    
    def log_info(self, msg):
        """根据日志级别决定是否输出信息"""
        if self.log_level <= 1:
            rospy.loginfo(msg)
    
    def log_debug(self, msg):
        """只在详细日志模式输出调试信息"""
        if self.log_level == 0:
            rospy.logdebug(msg)
    
    def left_camera_loop(self):
        """左摄像头读取循环"""
        cap = cv2.VideoCapture(self.left_device)
        if not cap.isOpened():
            rospy.logerr("无法打开左摄像头: %s", self.left_device)
            return
        
        # 设置摄像头参数
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        
        rospy.loginfo("左摄像头已启动")
        
        while self.running and not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                with self.left_lock:
                    self.left_image = frame.copy()
                    self.left_timestamp = time.time()
                    self.left_frame_count += 1
                
                # 发布左图像
                try:
                    left_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    left_msg.header.stamp = rospy.Time.now()
                    left_msg.header.frame_id = "left_camera_link"
                    self.left_pub.publish(left_msg)
                except CvBridgeError as e:
                    rospy.logerr("发布左图像时出错: %s", e)
            else:
                rospy.logwarn("左摄像头读取失败")
            
            time.sleep(1.0 / self.fps)
        
        cap.release()
        rospy.loginfo("左摄像头已关闭")
    
    def right_camera_loop(self):
        """右摄像头读取循环"""
        cap = cv2.VideoCapture(self.right_device)
        if not cap.isOpened():
            rospy.logerr("无法打开右摄像头: %s", self.right_device)
            return
        
        # 设置摄像头参数
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution_height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
        
        rospy.loginfo("右摄像头已启动")
        
        while self.running and not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                with self.right_lock:
                    self.right_image = frame.copy()
                    self.right_timestamp = time.time()
                    self.right_frame_count += 1
                
                # 发布右图像
                try:
                    right_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    right_msg.header.stamp = rospy.Time.now()
                    right_msg.header.frame_id = "right_camera_link"
                    self.right_pub.publish(right_msg)
                except CvBridgeError as e:
                    rospy.logerr("发布右图像时出错: %s", e)
            else:
                rospy.logwarn("右摄像头读取失败")
            
            time.sleep(1.0 / self.fps)
        
        cap.release()
        rospy.loginfo("右摄像头已关闭")
    
    def merge_loop(self):
        """图像合并循环"""
        while self.running and not rospy.is_shutdown():
            left_img = None
            right_img = None
            
            # 获取最新的左右图像
            with self.left_lock:
                if self.left_image is not None:
                    left_img = self.left_image.copy()
            
            with self.right_lock:
                if self.right_image is not None:
                    right_img = self.right_image.copy()
            
            # 如果两个图像都可用，则合并
            if left_img is not None and right_img is not None:
                try:
                    # 确保两个图像尺寸一致
                    if left_img.shape != right_img.shape:
                        # 调整右图像尺寸以匹配左图像
                        right_img = cv2.resize(right_img, (left_img.shape[1], left_img.shape[0]))
                    
                    # 水平合并左右图像
                    merged_image = np.hstack((left_img, right_img))
                    
                    # 在合并图像上添加标识文字
                    cv2.putText(merged_image, "LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(merged_image, "RIGHT", (left_img.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # 发布合并图像
                    merged_msg = self.bridge.cv2_to_imgmsg(merged_image, "bgr8")
                    merged_msg.header.stamp = rospy.Time.now()
                    merged_msg.header.frame_id = "stereo_camera_link"
                    self.merged_pub.publish(merged_msg)
                    
                    self.frame_counter += 1
                    
                except Exception as e:
                    rospy.logerr("合并图像时出错: %s", e)
            
            # 定期报告统计信息
            current_time = time.time()
            if current_time - self.last_log_time > 5.0:
                if self.left_frame_count > 0:
                    self.left_fps = self.left_frame_count / (current_time - self.last_log_time)
                if self.right_frame_count > 0:
                    self.right_fps = self.right_frame_count / (current_time - self.last_log_time)
                
                self.log_info(f"左摄像头帧率: {self.left_fps:.2f} FPS, 右摄像头帧率: {self.right_fps:.2f} FPS")
                self.log_info(f"合并图像帧率: {self.frame_counter / (current_time - self.last_log_time):.2f} FPS")
                
                self.left_frame_count = 0
                self.right_frame_count = 0
                self.frame_counter = 0
                self.last_log_time = current_time
            
            time.sleep(1.0 / self.fps)
    
    def shutdown(self):
        """关闭节点"""
        self.running = False
        rospy.loginfo("正在关闭双目摄像头合并器...")

if __name__ == '__main__':
    try:
        merger = StereoMerger()
        rospy.on_shutdown(merger.shutdown)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("用户中断，退出")
    except Exception as e:
        rospy.logerr("未知错误: %s", e) 