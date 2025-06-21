#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time

class StereoProcessor:
    def __init__(self):
        rospy.init_node('stereo_processor', anonymous=True)
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 图像处理中的参数
        self.log_level = rospy.get_param('~log_level', 1)  # 0=详细, 1=标准, 2=只有错误
        self.save_test_image = rospy.get_param('~save_test_image', False)  # 是否保存测试图像
        self.frame_counter = 0  # 用于跟踪处理的帧数
        self.last_log_time = time.time()  # 上次日志输出时间
        self.error_count = 0  # 错误计数器
        self.max_errors = 10  # 最大连续错误次数
        
        # 发布器 - 直接发布原始图像
        self.processed_pub = rospy.Publisher('/stereo_camera/image_processed', Image, queue_size=1)
        
        # 订阅原始图像
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback, queue_size=1)
        
        rospy.loginfo("立体相机处理器已初始化 - 日志级别: %d", self.log_level)
    
    def log_info(self, msg):
        """根据日志级别决定是否输出信息"""
        if self.log_level <= 1:
            rospy.loginfo(msg)
    
    def log_debug(self, msg):
        """只在详细日志模式输出调试信息"""
        if self.log_level == 0:
            rospy.logdebug(msg)
    
    def image_callback(self, data):
        try:
            # 计算帧率并定期报告
            self.frame_counter += 1
            current_time = time.time()
            if current_time - self.last_log_time > 5.0:  # 每5秒报告一次
                fps = self.frame_counter / (current_time - self.last_log_time)
                self.log_info(f"帧率: {fps:.2f} FPS")
                self.frame_counter = 0
                self.last_log_time = current_time
            
            # 将ROS图像消息转换为OpenCV图像
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                # 成功处理了图像，重置错误计数
                self.error_count = 0
            except CvBridgeError as e:
                self.error_count += 1
                rospy.logerr("CvBridge转换错误: %s (连续错误: %d)", e, self.error_count)
                if self.error_count > self.max_errors:
                    rospy.logwarn("连续错误过多，尝试使用备用解码方法")
                    # 尝试使用不同的编码方式
                    try:
                        cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                        self.error_count = 0
                    except:
                        return
                else:
                    return
            
            # 获取图像尺寸
            height, width, channels = cv_image.shape
            
            # 打印图像尺寸信息 (只在第一次接收图像时打印)
            if not hasattr(self, 'image_size_reported'):
                rospy.loginfo("接收到原始图像，尺寸: %dx%d, 通道: %d", width, height, channels)
                self.image_size_reported = True
            
            # 检查图像是否有效
            if width == 0 or height == 0:
                rospy.logerr("收到无效图像尺寸: %dx%d", width, height)
                return
            
            # 检查图像是否为全黑或单色
            if self.check_blank_image(cv_image):
                rospy.logwarn_once("警告: 检测到空白或单色图像")
            
            # 处理图像
            processed_image = cv_image.copy()
            
            # 简单的图像增强
            try:
                # 应用轻微的高斯模糊减少噪点
                processed_image = cv2.GaussianBlur(processed_image, (3, 3), 0)
                
                # 轻微提高对比度和亮度
                processed_image = cv2.convertScaleAbs(processed_image, alpha=1.1, beta=5)
                
                # 如果是双目相机图像（宽高比大于2:1），则分割为左右两部分
                if width > height * 1.5:
                    half_width = width // 2
                    processed_image = processed_image[:, :half_width].copy()
            except Exception as e:
                rospy.logerr("图像增强处理错误: %s", e)
            
            # 将OpenCV图像转换回ROS消息
            try:
                processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
                
                # 复制时间戳和帧ID
                processed_msg.header.stamp = data.header.stamp
                processed_msg.header.frame_id = "camera_link"
                
                # 发布处理后的图像
                self.processed_pub.publish(processed_msg)
                
                self.log_debug("已处理并发布图像")
            except CvBridgeError as e:
                rospy.logerr("发布图像时出错: %s", e)
            
        except Exception as e:
            rospy.logerr("处理图像时出错: %s", e)

    def check_blank_image(self, image):
        """检查图像是否为空白(全黑)或单一颜色"""
        try:
            # 检查图像是否为全黑
            if np.mean(image) < 5.0:
                return True
                
            # 检查各通道的方差和是否很小(单一颜色)
            if np.sum(np.var(image, axis=(0, 1))) < 10.0:
                return True
                
            return False
        except Exception as e:
            rospy.logerr("检查空白图像时出错: %s", e)
            return False

if __name__ == '__main__':
    try:
        processor = StereoProcessor()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("用户中断，退出")
    except Exception as e:
        rospy.logerr("未知错误: %s", e) 