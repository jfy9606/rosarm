#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraNode:
    """摄像头节点，用于获取摄像头图像并发布到ROS话题"""
    
    def __init__(self):
        """初始化摄像头节点"""
        rospy.init_node('camera_node')
        
        # 获取参数
        self.device = rospy.get_param('~device', '/dev/video0')
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 30)
        self.retry_count = rospy.get_param('~retry_count', 5)
        self.retry_delay = rospy.get_param('~retry_delay', 2.0)
        
        # 创建图像发布器
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        
        # 创建OpenCV桥接器
        self.bridge = CvBridge()
        
        # 初始化摄像头
        self.init_camera()
        
        # 设置定时器
        self.timer = rospy.Timer(rospy.Duration(1.0/self.fps), self.timer_callback)
        
        rospy.loginfo(f"摄像头节点已初始化，设备: {self.device}")
    
    def init_camera(self):
        """初始化摄像头设备"""
        self.use_test_image = False
        attempts = 0
        
        while attempts < self.retry_count and not rospy.is_shutdown():
            try:
                # 尝试不同的方式打开摄像头
                # 首先尝试使用设备路径
                if os.path.exists(self.device):
                    self.cap = cv2.VideoCapture(self.device)
                    rospy.loginfo(f"尝试打开摄像头设备: {self.device}")
                else:
                    # 如果设备路径不存在，尝试使用索引
                    try:
                        device_index = int(self.device.split('video')[-1])
                        self.cap = cv2.VideoCapture(device_index)
                        rospy.loginfo(f"尝试打开摄像头索引: {device_index}")
                    except (ValueError, IndexError):
                        # 如果无法解析索引，尝试使用默认设备
                        self.cap = cv2.VideoCapture(0)
                        rospy.loginfo("尝试打开默认摄像头 (索引 0)")
                
                if not self.cap.isOpened():
                    attempts += 1
                    rospy.logwarn(f"无法打开摄像头设备，尝试 {attempts}/{self.retry_count}")
                    if attempts < self.retry_count:
                        rospy.sleep(self.retry_delay)
                    continue
                
                # 设置分辨率
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                
                # 检查摄像头是否工作
                ret, frame = self.cap.read()
                if not ret:
                    attempts += 1
                    rospy.logwarn(f"摄像头读取失败，尝试 {attempts}/{self.retry_count}")
                    if attempts < self.retry_count:
                        rospy.sleep(self.retry_delay)
                    continue
                
                self.use_test_image = False
                rospy.loginfo("摄像头已成功初始化")
                return
                
            except Exception as e:
                attempts += 1
                rospy.logerr(f"初始化摄像头时出错: {str(e)}")
                if attempts < self.retry_count:
                    rospy.logwarn(f"将在 {self.retry_delay} 秒后重试...")
                    rospy.sleep(self.retry_delay)
        
        # 如果所有尝试都失败，使用测试图像
        rospy.logwarn("所有摄像头初始化尝试都失败，将使用测试图像")
        self.use_test_image = True
    
    def create_test_image(self):
        """创建测试图像"""
        # 创建黑色背景
        img = np.zeros((self.height, self.width, 3), np.uint8)
        
        # 添加一些彩色形状
        # 红色圆形
        cv2.circle(img, (int(self.width/4), int(self.height/2)), 50, (0, 0, 255), -1)
        # 绿色矩形
        cv2.rectangle(img, (int(self.width/2), int(self.height/4)), 
                     (int(3*self.width/4), int(3*self.height/4)), (0, 255, 0), -1)
        # 蓝色三角形
        pts = np.array([[int(3*self.width/4), int(self.height/4)], 
                        [int(7*self.width/8), int(self.height/2)],
                        [int(3*self.width/4), int(3*self.height/4)]], np.int32)
        cv2.fillPoly(img, [pts], (255, 0, 0))
        
        # 添加文本
        cv2.putText(img, "Test Image", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(img, "No Camera Available", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 添加时间戳
        timestamp = rospy.Time.now().to_sec()
        cv2.putText(img, f"Time: {timestamp:.2f}", (20, self.height-20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        return img
    
    def timer_callback(self, event):
        """定时器回调函数，用于获取图像并发布"""
        if self.use_test_image:
            # 使用测试图像
            frame = self.create_test_image()
        else:
            # 从摄像头获取图像
            try:
                ret, frame = self.cap.read()
                
                if not ret:
                    rospy.logwarn("无法获取图像，使用测试图像代替")
                    frame = self.create_test_image()
                    
                    # 尝试重新初始化摄像头
                    self.init_camera()
            except Exception as e:
                rospy.logerr(f"读取摄像头时出错: {str(e)}")
                frame = self.create_test_image()
                
                # 尝试重新初始化摄像头
                self.init_camera()
        
        try:
            # 转换为ROS图像消息并发布
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "camera"
            
            self.image_pub.publish(img_msg)
            
        except Exception as e:
            rospy.logerr(f"发布图像时出错: {str(e)}")
    
    def shutdown(self):
        """关闭节点"""
        if not self.use_test_image and hasattr(self, 'cap'):
            self.cap.release()
        
        rospy.loginfo("摄像头节点已关闭")

def main():
    """主函数"""
    camera_node = CameraNode()
    
    try:
        rospy.loginfo("等待摄像头数据...")
        rospy.spin()
    except KeyboardInterrupt:
        pass
    
    camera_node.shutdown()

if __name__ == '__main__':
    main()