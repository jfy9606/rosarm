#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Pose, PoseArray, Twist
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge, CvBridgeError

class VisualServoBridge:
    """
    增强型桥接节点，将视觉系统的检测结果转换为机械臂控制命令
    整合了特征检测、图像处理和目标跟踪功能
    """
    
    def __init__(self):
        """初始化视觉伺服桥接节点"""
        rospy.init_node('visual_servo_bridge', anonymous=True)
        
        # 初始化参数
        self.detection_topic = rospy.get_param('~detection_topic', '/detections/poses')
        self.target_pose_topic = rospy.get_param('~target_pose_topic', '/trajectory/target_pose')
        self.image_topic = rospy.get_param('~image_topic', '/stereo_camera/image_raw')
        self.feature_detection_method = rospy.get_param('~feature_detection_method', 'aruco')
        self.enabled = rospy.get_param('~enabled', False)
        self.control_rate = rospy.get_param('~control_rate', 10)  # Hz
        
        # 初始化控制参数
        self.gain = rospy.get_param('~gain', 0.5)
        self.max_velocity = rospy.get_param('~max_velocity', 0.2)
        self.min_velocity = rospy.get_param('~min_velocity', 0.01)
        self.target_tolerance = rospy.get_param('~target_tolerance', 0.01)
        
        # 初始化OpenCV桥接
        self.bridge = CvBridge()
        
        # 初始化特征检测器
        self._init_feature_detector()
        
        # 创建服务
        self.enable_service = rospy.Service('/visual_servo_bridge/enable', SetBool, self.enable_callback)
        
        # 订阅视觉系统的检测结果
        self.detection_sub = rospy.Subscriber(self.detection_topic, PoseArray, self.detection_callback)
        
        # 订阅图像和关节状态
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # 创建发布器
        self.target_pose_pub = rospy.Publisher(self.target_pose_topic, Pose, queue_size=10)
        self.status_pub = rospy.Publisher('/visual_servo_bridge/status', Bool, queue_size=10, latch=True)
        self.velocity_pub = rospy.Publisher('/visual_servo_bridge/velocity', Twist, queue_size=10)
        self.image_pub = rospy.Publisher('/visual_servo_bridge/image', Image, queue_size=10)
        
        # 初始化状态变量
        self.current_image = None
        self.current_joint_state = None
        self.target_features = None
        self.current_features = None
        self.target_reached = False
        
        # 创建控制定时器
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_timer_callback)
        
        # 初始化状态
        self.publish_status()
        
        rospy.loginfo(f"视觉伺服桥接节点初始化完成")
        rospy.loginfo(f"订阅检测话题: {self.detection_topic}")
        rospy.loginfo(f"订阅图像话题: {self.image_topic}")
        rospy.loginfo(f"发布目标位姿话题: {self.target_pose_topic}")
        rospy.loginfo(f"桥接状态: {'启用' if self.enabled else '禁用'}")
    
    def _init_feature_detector(self):
        """初始化特征检测器"""
        if self.feature_detection_method == 'aruco':
            # 初始化ArUco检测器
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        elif self.feature_detection_method == 'sift':
            # 初始化SIFT检测器
            self.sift = cv2.SIFT_create()
            self.flann = cv2.FlannBasedMatcher({'algorithm': 0, 'trees': 5}, {'checks': 50})
        else:
            rospy.logwarn(f"未知的特征检测方法: {self.feature_detection_method}，使用ArUco")
            self.feature_detection_method = 'aruco'
            # 初始化ArUco检测器
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
    
    def publish_status(self):
        """发布当前桥接状态"""
        status_msg = Bool()
        status_msg.data = self.enabled
        self.status_pub.publish(status_msg)
    
    def enable_callback(self, req):
        """处理启用/禁用桥接的服务请求"""
        self.enabled = req.data
        
        if self.enabled:
            rospy.loginfo("视觉伺服桥接已启用")
        else:
            rospy.loginfo("视觉伺服桥接已禁用")
            # 发布零速度命令
            self.publish_velocity(np.zeros(6))
        
        # 发布状态
        self.publish_status()
        
        return SetBoolResponse(True, f"视觉伺服桥接{'启用' if self.enabled else '禁用'}")
    
    def joint_state_callback(self, msg):
        """处理关节状态消息"""
        self.current_joint_state = msg
    
    def image_callback(self, msg):
        """处理图像消息"""
        if not self.enabled:
            return
            
        try:
            # 将ROS图像消息转换为OpenCV格式
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测特征
            features = self.detect_features(self.current_image)
            
            if features is not None and len(features) > 0:
                # 设置当前特征
                self.current_features = features
                
                # 如果目标特征未设置，使用当前特征作为目标
                if self.target_features is None:
                    self.target_features = features
                    rospy.loginfo(f"目标特征已设置: {self.target_features}")
            
            # 在图像上绘制特征
            annotated_image = self.draw_features(self.current_image, features)
            
            # 将OpenCV图像转换回ROS图像消息
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = msg.header
            
            # 发布标注图像
            self.image_pub.publish(annotated_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV桥接错误: {str(e)}")
    
    def detection_callback(self, msg):
        """处理视觉系统的检测结果"""
        if not self.enabled or not msg.poses:
            return
        
        try:
            # 选择第一个检测结果作为目标
            # 在实际应用中，可以根据需要添加更复杂的目标选择逻辑
            target_pose = msg.poses[0]
            
            # 发布目标位姿给机械臂控制系统
            self.target_pose_pub.publish(target_pose)
            
            rospy.loginfo(f"发送目标位姿: 位置({target_pose.position.x:.2f}, {target_pose.position.y:.2f}, {target_pose.position.z:.2f})")
            
        except Exception as e:
            rospy.logerr(f"处理检测结果时出错: {str(e)}")
    
    def detect_features(self, image):
        """
        在图像中检测特征
        
        Args:
            image: 输入图像
            
        Returns:
            features: 检测到的特征点
        """
        if image is None:
            return None
        
        if self.feature_detection_method == 'aruco':
            # 检测ArUco标记
            corners, ids, rejected = self.aruco_detector.detectMarkers(image)
            
            if ids is not None and len(ids) > 0:
                # 提取角点
                features = []
                for corner in corners:
                    # 使用标记中心
                    center = np.mean(corner[0], axis=0)
                    features.append(center)
                
                return np.array(features)
            else:
                return None
        elif self.feature_detection_method == 'sift':
            # 转换为灰度图
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 检测SIFT特征
            keypoints, descriptors = self.sift.detectAndCompute(gray, None)
            
            if keypoints and len(keypoints) > 0:
                # 提取关键点坐标
                features = np.array([kp.pt for kp in keypoints])
                
                # 限制特征数量
                max_features = 10
                if len(features) > max_features:
                    # 选择响应最高的特征
                    responses = np.array([kp.response for kp in keypoints])
                    indices = np.argsort(responses)[-max_features:]
                    features = features[indices]
                
                return features
            else:
                return None
        else:
            rospy.logwarn(f"未知的特征检测方法: {self.feature_detection_method}")
            return None
    
    def draw_features(self, image, features):
        """
        在图像上绘制特征
        
        Args:
            image: 输入图像
            features: 特征点
            
        Returns:
            annotated_image: 带有特征标注的图像
        """
        if image is None:
            return None
        
        # 创建图像副本
        annotated_image = image.copy()
        
        # 绘制当前特征
        if features is not None and len(features) > 0:
            for i, (x, y) in enumerate(features):
                # 绘制圆圈
                cv2.circle(annotated_image, (int(x), int(y)), 5, (0, 255, 0), 2)
                
                # 绘制索引
                cv2.putText(
                    annotated_image,
                    str(i),
                    (int(x) + 10, int(y) + 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
        
        # 绘制目标特征
        if self.target_features is not None and len(self.target_features) > 0:
            for i, (x, y) in enumerate(self.target_features):
                # 绘制圆圈
                cv2.circle(annotated_image, (int(x), int(y)), 5, (0, 0, 255), 2)
                
                # 绘制索引
                cv2.putText(
                    annotated_image,
                    f"T{i}",
                    (int(x) + 10, int(y) - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 0, 255),
                    2
                )
        
        # 绘制状态
        status_text = "桥接: " + ("启用" if self.enabled else "禁用")
        cv2.putText(
            annotated_image,
            status_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0) if self.enabled else (0, 0, 255),
            2
        )
        
        return annotated_image
    
    def control_timer_callback(self, event):
        """控制定时器回调函数"""
        if not self.enabled:
            return
        
        # 如果目标已达到，停止控制
        if self.target_reached:
            rospy.loginfo("目标已达到")
            self.enabled = False
            self.publish_status()
            self.publish_velocity(np.zeros(6))
            return
        
        # 如果没有特征可用，返回
        if self.current_features is None or self.target_features is None:
            return
        
        # 计算特征误差
        error = self.compute_feature_error()
        
        # 如果误差足够小，目标已达到
        if error < self.target_tolerance:
            self.target_reached = True
            rospy.loginfo("目标已达到")
            self.publish_velocity(np.zeros(6))
            return
        
        # 计算并发布速度命令
        velocity = self.compute_velocity_command(error)
        self.publish_velocity(velocity)
    
    def compute_feature_error(self):
        """
        计算特征误差
        
        Returns:
            error_norm: 误差范数
        """
        if self.current_features is None or self.target_features is None:
            return float('inf')
        
        # 检查特征数组是否具有相同的形状
        if self.current_features.shape != self.target_features.shape:
            rospy.logerr(f"特征数组具有不同的形状: {self.current_features.shape} vs {self.target_features.shape}")
            return float('inf')
        
        # 计算误差
        error = self.current_features.flatten() - self.target_features.flatten()
        
        # 计算误差范数
        error_norm = np.linalg.norm(error)
        
        return error_norm
    
    def compute_velocity_command(self, error):
        """
        计算速度命令
        
        Args:
            error: 误差值
            
        Returns:
            velocity: 速度命令 [vx, vy, vz, wx, wy, wz]
        """
        # 简化的速度计算，仅使用比例控制
        # 在实际应用中，可以使用更复杂的控制算法
        velocity = np.zeros(6)
        
        # 设置线性速度（x, y方向）
        velocity[0] = -self.gain * error * 0.01
        velocity[1] = -self.gain * error * 0.01
        
        # 应用速度限制
        velocity_norm = np.linalg.norm(velocity[:3])
        if velocity_norm > self.max_velocity:
            velocity[:3] = velocity[:3] * self.max_velocity / velocity_norm
        elif velocity_norm < self.min_velocity and velocity_norm > 0:
            velocity[:3] = velocity[:3] * self.min_velocity / velocity_norm
        
        return velocity
    
    def publish_velocity(self, velocity):
        """
        发布速度命令
        
        Args:
            velocity: 速度命令 [vx, vy, vz, wx, wy, wz]
        """
        # 创建Twist消息
        twist = Twist()
        
        # 设置线性速度
        twist.linear.x = velocity[0]
        twist.linear.y = velocity[1]
        twist.linear.z = velocity[2]
        
        # 设置角速度
        twist.angular.x = velocity[3]
        twist.angular.y = velocity[4]
        twist.angular.z = velocity[5]
        
        # 发布速度
        self.velocity_pub.publish(twist)

def main():
    """主函数"""
    try:
        # 创建视觉伺服桥接节点
        bridge = VisualServoBridge()
        
        # 处理回调
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 