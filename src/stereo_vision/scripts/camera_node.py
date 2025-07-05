#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
import warnings
from cv_bridge import CvBridge

class CameraNode:
    """双目摄像头节点，可以处理双目摄像头或分割单个图像为左右两部分"""
    
    def __init__(self):
        """初始化相机节点"""
        rospy.init_node('camera_node', anonymous=True)
        
        # 获取参数
        self.device_id = rospy.get_param('~device_id', 0)  # 摄像头设备ID
        self.width = rospy.get_param('~width', 1280)  # 图像宽度
        self.height = rospy.get_param('~height', 480)  # 图像高度
        self.fps = rospy.get_param('~fps', 30)  # 帧率
        self.publish_rate = rospy.get_param('~publish_rate', 30.0)  # 发布频率
        self.pixel_format = rospy.get_param('~pixel_format', 'MJPEG')  # 像素格式
        self.retry_count = rospy.get_param('~retry_count', 5)  # 摄像头初始化尝试次数
        self.retry_delay = rospy.get_param('~retry_delay', 2.0)  # 尝试间隔
        self.max_errors = rospy.get_param('~max_errors', 50)  # 最大连续错误次数，超过后重新初始化
        
        # 初始化摄像头连接状态和错误计数
        self.use_test_image = False
        self.error_count = 0
        
        # 创建图像转换桥
        self.bridge = CvBridge()
        
        # 创建图像发布者
        self.image_pub = rospy.Publisher('left_camera/image_raw', Image, queue_size=1)
        self.right_image_pub = rospy.Publisher('right_camera/image_raw', Image, queue_size=1)
        
        # 初始化摄像头
        self.init_camera()
        
        # 创建定时器，定期发布图像
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        
        # 注册节点关闭回调
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("摄像头节点已初始化，准备发布图像")
    
    def init_camera(self):
        """初始化摄像头连接"""
        attempts = 0
        
        while attempts < self.retry_count:
            try:
                # 尝试打开摄像头
                self.cap = cv2.VideoCapture(self.device_id)
                
                if not self.cap.isOpened():
                    attempts += 1
                    rospy.logwarn(f"无法打开摄像头设备，尝试 {attempts}/{self.retry_count}")
                    if attempts < self.retry_count:
                        rospy.sleep(self.retry_delay)
                    continue
                
                # 设置分辨率
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                
                # 设置像素格式
                if self.pixel_format.lower() == 'mjpeg':
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                    rospy.loginfo("设置像素格式为MJPEG")
                elif self.pixel_format.lower() == 'yuyv':
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
                    rospy.loginfo("设置像素格式为YUYV")
                
                # 检查并打印实际的摄像头设置
                actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
                actual_fourcc = self.cap.get(cv2.CAP_PROP_FOURCC)
                actual_fourcc_str = "".join([chr((int(actual_fourcc) >> 8*i) & 0xFF) for i in range(4)])
                
                rospy.loginfo(f"摄像头实际设置: 分辨率={actual_width}x{actual_height}, FPS={actual_fps}, FOURCC={actual_fourcc_str}")
                
                # 如果实际格式与请求格式不符，尝试强制设置
                if self.pixel_format.lower() == 'mjpeg' and actual_fourcc_str != 'MJPG':
                    rospy.logwarn(f"请求的MJPEG格式未应用，实际格式为{actual_fourcc_str}，尝试再次设置...")
                    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                    # 重新检查
                    actual_fourcc = self.cap.get(cv2.CAP_PROP_FOURCC)
                    actual_fourcc_str = "".join([chr((int(actual_fourcc) >> 8*i) & 0xFF) for i in range(4)])
                    rospy.loginfo(f"重设后的格式: FOURCC={actual_fourcc_str}")
                
                # 检查摄像头是否工作
                # 多次尝试读取帧，避免JPEG错误
                success = False
                for i in range(5):  # 尝试5次读取
                    try:
                        with warnings.catch_warnings():
                            warnings.simplefilter("ignore")  # 忽略警告
                            ret, frame = self.cap.read()
                            if ret:
                                success = True
                                break
                    except Exception as e:
                        rospy.logwarn(f"读取帧尝试 {i+1}/5 失败: {str(e)}")
                    rospy.sleep(0.1)  # 短暂延迟
                
                if not success:
                    attempts += 1
                    rospy.logwarn(f"摄像头读取失败，尝试 {attempts}/{self.retry_count}")
                    if attempts < self.retry_count:
                        rospy.sleep(self.retry_delay)
                    continue
                
                rospy.loginfo(f"成功读取第一帧，尺寸: {frame.shape[1]}x{frame.shape[0]}")
                self.use_test_image = False
                self.error_count = 0  # 重置错误计数
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
            # 分割测试图像为左右两部分
            height, width = frame.shape[:2]
            mid = width // 2
            left_frame = frame[:, :mid]
            right_frame = frame[:, mid:]
        else:
            # 从摄像头获取图像
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")  # 忽略警告
                    ret, frame = self.cap.read()
                
                if not ret:
                    self.error_count += 1
                    # 减少日志输出频率，只在错误计数是5的倍数时输出
                    if self.error_count % 5 == 0:
                        rospy.logwarn(f"无法获取图像，错误计数: {self.error_count}/{self.max_errors}")
                    
                    if self.error_count >= self.max_errors:
                        rospy.logwarn("错误次数过多，尝试重新初始化摄像头")
                        self.init_camera()
                    
                    frame = self.create_test_image()
                    # 分割测试图像为左右两部分
                    height, width = frame.shape[:2]
                    mid = width // 2
                    left_frame = frame[:, :mid]
                    right_frame = frame[:, mid:]
                else:
                    self.error_count = 0  # 成功读取，重置错误计数
                    # 分割图像为左右两部分
                    height, width = frame.shape[:2]
                    mid = width // 2
                    left_frame = frame[:, :mid]
                    right_frame = frame[:, mid:]
            except Exception as e:
                self.error_count += 1
                # 减少日志输出频率，只在错误计数是5的倍数时输出
                if self.error_count % 5 == 0:
                    rospy.logerr(f"读取摄像头时出错: {str(e)}，错误计数: {self.error_count}/{self.max_errors}")
                
                if self.error_count >= self.max_errors:
                    rospy.logwarn("错误次数过多，尝试重新初始化摄像头")
                    self.init_camera()
                
                frame = self.create_test_image()
                # 分割测试图像为左右两部分
                height, width = frame.shape[:2]
                mid = width // 2
                left_frame = frame[:, :mid]
                right_frame = frame[:, mid:]
        
        try:
            # 转换为ROS图像消息并发布
            timestamp = rospy.Time.now()
            
            # 发布左图像（作为主图像）
            left_msg = self.bridge.cv2_to_imgmsg(left_frame, "bgr8")
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = "camera_left"
            self.image_pub.publish(left_msg)
            
            # 发布右图像
            right_msg = self.bridge.cv2_to_imgmsg(right_frame, "bgr8")
            right_msg.header.stamp = timestamp
            right_msg.header.frame_id = "camera_right"
            self.right_image_pub.publish(right_msg)
            
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
