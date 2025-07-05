#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import time
from geometry_msgs.msg import PoseArray, Pose, Point
from std_msgs.msg import String, Bool, Float64, Int32
from sensor_msgs.msg import JointState, Image
from servo_wrist.msg import SerControl
import cv_bridge
import cv2

class VisualServoController:
    """
    视觉伺服控制器，用于根据双目摄像头的深度信息控制机械臂接近物体并抓取
    """
    
    def __init__(self):
        """初始化视觉伺服控制器"""
        rospy.init_node('visual_servo_controller', anonymous=True)
        
        # 初始化成员变量
        self.left_image = None
        self.right_image = None
        self.depth_image = None
        self.detected_objects = []
        self.current_joint_values = [0.0] * 6
        self.arm_ready = False
        self.visual_servo_enabled = False
        self.servo_target_object = None
        self.vacuum_on = False
        self.vacuum_power = 50  # 默认吸盘功率50%
        
        # 相机内参
        self.camera_intrinsic = np.array([
            [525.0, 0.0, 320.0],
            [0.0, 525.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        
        # 双目相机基线（单位：米）
        self.baseline = 0.06  # 假设双目相机基线为6cm
        
        # 创建CV bridge
        self.bridge = cv_bridge.CvBridge()
        
        # 设置订阅者
        self.left_image_sub = rospy.Subscriber('/stereo_camera/left/image_raw', Image, 
                                             self.left_image_callback)
        self.right_image_sub = rospy.Subscriber('/stereo_camera/right/image_raw', Image, 
                                              self.right_image_callback)
        # 订阅原始深度图，而不是彩色深度图
        self.depth_image_sub = rospy.Subscriber('/stereo_camera/depth/raw', Image, 
                                              self.depth_image_callback)
        self.detection_sub = rospy.Subscriber('/object_detection/detections', Image,
                                           self.detection_callback)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState,
                                             self.joint_state_callback)
        self.command_sub = rospy.Subscriber('/visual_servo/command', String,
                                         self.command_callback)
        
        # 设置发布者
        self.joint_command_pub = rospy.Publisher('/arm1/joint_command', JointState, queue_size=1)
        self.vacuum_command_pub = rospy.Publisher('/vacuum/command', Bool, queue_size=1)
        self.vacuum_power_pub = rospy.Publisher('/vacuum/power', Int32, queue_size=1)
        self.status_pub = rospy.Publisher('/visual_servo/status', String, queue_size=1)
        
        # 设置定时器，用于视觉伺服控制
        self.timer = rospy.Timer(rospy.Duration(0.1), self.visual_servo_control)
        
        rospy.loginfo("视觉伺服控制器初始化完成")
    
    def detection_callback(self, msg):
        """处理物体检测结果"""
        # 清空之前的检测结果
        self.detected_objects = []
        
        # 如果没有检测到物体，直接返回
        if len(msg.poses) == 0:
            return
        
        # 解析类别信息，假设frame_id中包含逗号分隔的类别名称
        class_names = []
        if msg.header.frame_id:
            class_names = msg.header.frame_id.split(',')
        
        # 处理每个检测到的物体
        for i, pose in enumerate(msg.poses):
            # 创建物体信息字典
            obj = {
                'id': class_names[i] if i < len(class_names) else f"object_{i}",
                'position': [pose.position.x, pose.position.y, pose.position.z],
                'orientation': [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                'pose': pose
            }
            self.detected_objects.append(obj)
        
        # 如果启用了视觉伺服控制且有目标物体，更新目标物体信息
        if self.visual_servo_enabled and self.servo_target_object:
            for obj in self.detected_objects:
                if obj['id'] == self.servo_target_object['id']:
                    self.servo_target_object = obj
                    break
    
    def joint_state_callback(self, msg):
        """处理关节状态信息"""
        # 更新当前关节值
        if len(msg.position) >= 6:
            self.current_joint_values = list(msg.position)
            
            # 计算末端位置（简化版本，实际应使用正向运动学）
            # 这里假设机械臂是SCARA类型，末端位置可以从关节角度计算
            # 实际应用中应使用完整的运动学模型
            theta1 = self.current_joint_values[0]  # 底座旋转角度
            d2 = self.current_joint_values[1]      # 伸缩关节长度
            theta3 = self.current_joint_values[2]  # 肩部关节角度
            theta4 = self.current_joint_values[3]  # 肘部关节角度
            d6 = self.current_joint_values[5]      # 末端伸缩长度
            
            # 简化的正向运动学计算（实际应使用DH参数和完整的运动学模型）
            x = d2 * math.cos(theta1) + d6 * math.cos(theta1 + theta3 + theta4)
            y = d2 * math.sin(theta1) + d6 * math.sin(theta1 + theta3 + theta4)
            z = 0.5 - 0.2 * math.sin(theta3) - 0.15 * math.sin(theta3 + theta4)  # 假设的Z轴计算
            
            self.current_end_position = [x, y, z]
            
            # 机械臂就绪标志
            self.arm_ready = True
    
    def left_image_callback(self, msg):
        """处理左相机图像"""
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(f"左相机图像转换错误: {e}")
    
    def right_image_callback(self, msg):
        """处理右相机图像"""
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(f"右相机图像转换错误: {e}")
    
    def depth_image_callback(self, msg):
        """处理深度图像回调"""
        try:
            # 处理原始深度图像
            if msg.encoding == "32FC1":
                # 原始深度图 - 直接使用
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                rospy.logdebug("接收到原始深度图数据，单位为米")
            elif msg.encoding == "16UC1":
                # 需要将16位深度数据转换为实际距离
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                scale_factor = 0.001  # 通常为1mm
                self.depth_image = self.depth_image.astype(np.float32) * scale_factor
                rospy.logdebug("接收到16位深度图数据，已转换为米单位")
            else:
                rospy.logwarn(f"不支持的深度图像编码格式: {msg.encoding}")
                self.depth_image = None
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(f"深度图像转换错误: {e}")
            self.depth_image = None
    
    def command_callback(self, msg):
        """处理命令消息"""
        command = msg.data.strip()
        parts = command.split()
        
        if not parts:
            return
        
        cmd = parts[0].lower()
        
        # 处理不同类型的命令
        if cmd == "servo":
            # 启动视觉伺服控制：servo [object_id]
            if len(parts) > 1:
                object_id = parts[1]
                self.start_visual_servo(object_id)
            else:
                rospy.logwarn("视觉伺服命令缺少物体ID")
        
        elif cmd == "stop":
            # 停止视觉伺服控制
            self.stop_visual_servo()
        
        elif cmd == "vacuum":
            # 控制真空吸盘：vacuum [on|off] [power]
            if len(parts) > 1:
                state = parts[1].lower()
                power = int(parts[2]) if len(parts) > 2 else self.vacuum_power
                
                if state == "on":
                    self.set_vacuum(True, power)
                elif state == "off":
                    self.set_vacuum(False, power)
                else:
                    rospy.logwarn(f"无效的真空吸盘状态: {state}")
            else:
                rospy.logwarn("真空吸盘命令缺少状态参数")
        
        elif cmd == "home":
            # 回到初始位置
            self.go_home()
    
    def set_vacuum(self, on_state, power=None):
        """设置真空吸盘状态"""
        self.vacuum_on = on_state
        
        # 如果提供了功率值，更新功率
        if power is not None:
            self.vacuum_power = max(0, min(100, power))  # 限制在0-100范围内
            self.vacuum_power_pub.publish(Int32(self.vacuum_power))
        
        # 发布真空吸盘命令
        self.vacuum_command_pub.publish(Bool(on_state))
        
        rospy.loginfo(f"真空吸盘状态设置为: {'开启' if on_state else '关闭'}, 功率: {self.vacuum_power}%")
    
    def go_home(self):
        """回到初始位置"""
        rospy.loginfo("机械臂回到初始位置")
        
        # 停止视觉伺服控制
        self.visual_servo_enabled = False
        
        # 设置关节角度到初始位置
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = ["arm1_joint1", "arm1_joint2", "arm1_joint3", "arm1_joint4", "arm1_joint5", "arm1_joint6"]
        joint_msg.position = [0, 0, 0, 0, math.pi/2, 5]  # 初始位置
        
        # 发布关节命令
        self.joint_command_pub.publish(joint_msg)
        
        # 发布状态消息
        self.status_pub.publish("机械臂已回到初始位置")
    
    def start_visual_servo(self, object_id):
        """启动视觉伺服控制，接近指定ID的物体"""
        # 查找目标物体
        target_obj = None
        for obj in self.detected_objects:
            if obj['id'] == object_id:
                target_obj = obj
                break
        
        if not target_obj:
            rospy.logwarn(f"未找到ID为 {object_id} 的物体")
            return
        
        rospy.loginfo(f"启动视觉伺服控制，目标物体: {object_id}")
        
        # 设置目标物体
        self.servo_target_object = target_obj
        
        # 启用视觉伺服控制
        self.visual_servo_enabled = True
        
        # 发布状态消息
        self.status_pub.publish(f"视觉伺服控制已启动，目标物体: {object_id}")
    
    def stop_visual_servo(self):
        """停止视觉伺服控制"""
        if self.visual_servo_enabled:
            rospy.loginfo("停止视觉伺服控制")
            self.visual_servo_enabled = False
            self.servo_target_object = None
            
            # 发布状态消息
            self.status_pub.publish("视觉伺服控制已停止")
    
    def estimate_object_distance(self, object_center):
        """
        估计物体距离
        
        Args:
            object_center: 物体在图像中的中心位置 [x, y]
            
        Returns:
            估计的距离，单位米
        """
        # 如果有深度图像，直接从深度图像获取距离
        if self.depth_image is not None:
            x, y = int(object_center[0]), int(object_center[1])
            
            # 确保坐标在图像范围内
            if 0 <= x < self.depth_image.shape[1] and 0 <= y < self.depth_image.shape[0]:
                # 获取深度值（假设深度图像的值是以米为单位）
                depth = self.depth_image[y, x]
                
                # 检查深度值是否有效
                if depth > 0 and not math.isnan(depth):
                    return depth
        
        # 如果没有深度图像或深度值无效，使用双目视差估计距离
        if self.left_image is not None and self.right_image is not None:
            # 这里应该实现双目立体匹配算法来计算视差和深度
            # 由于算法复杂，这里只是一个简化示例
            # 实际应用中应使用完整的立体匹配算法
            
            # 将图像转换为灰度图
            left_gray = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(self.right_image, cv2.COLOR_BGR2GRAY)
            
            # 创建立体匹配器
            stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
            
            # 计算视差图
            disparity = stereo.compute(left_gray, right_gray)
            
            # 获取物体中心位置的视差
            x, y = int(object_center[0]), int(object_center[1])
            
            # 确保坐标在图像范围内
            if 0 <= x < disparity.shape[1] and 0 <= y < disparity.shape[0]:
                # 获取视差值
                d = disparity[y, x]
                
                # 检查视差值是否有效
                if d > 0:
                    # 根据视差计算深度
                    # 深度 = 基线 * 焦距 / 视差
                    depth = self.baseline * self.camera_intrinsic[0, 0] / d
                    return depth
        
        # 如果无法估计距离，返回一个默认值
        return 0.5  # 默认距离0.5米
    
    def visual_servo_control(self, event):
        """视觉伺服控制回调函数，定时执行"""
        # 如果未启用视觉伺服控制或没有目标物体，直接返回
        if not self.visual_servo_enabled or not self.servo_target_object or not self.arm_ready:
            return
        
        # 获取目标物体位置
        target_pos = self.servo_target_object['position']
        
        # 计算物体在图像中的位置
        # 假设物体位置是相对于相机坐标系的，需要转换为图像坐标
        # 这里使用简化的针孔相机模型
        fx = self.camera_intrinsic[0, 0]
        fy = self.camera_intrinsic[1, 1]
        cx = self.camera_intrinsic[0, 2]
        cy = self.camera_intrinsic[1, 2]
        
        # 计算物体在图像中的位置
        object_x = int(fx * target_pos[0] / target_pos[2] + cx)
        object_y = int(fy * target_pos[1] / target_pos[2] + cy)
        
        # 估计物体距离
        object_distance = self.estimate_object_distance([object_x, object_y])
        
        # 限制日志输出频率
        if hasattr(self, 'last_log_time') and (rospy.Time.now() - self.last_log_time).to_sec() < 1.0:
            pass
        else:
            rospy.loginfo(f"目标物体距离: {object_distance:.3f}米")
            self.last_log_time = rospy.Time.now()
        
        # 如果距离小于阈值，认为已接近物体
        if object_distance < self.distance_threshold:
            rospy.loginfo(f"已接近目标物体，距离: {object_distance:.3f}米")
            
            # 启动真空吸盘进行抓取
            self.set_vacuum(True)
            
            # 等待抓取完成
            rospy.sleep(0.5)
            
            # 停止视觉伺服控制
            self.visual_servo_enabled = False
            
            # 抬升物体
            current_pos = self.current_end_position
            self.move_to_position([current_pos[0], current_pos[1], current_pos[2] + 0.1])
            
            # 发布状态消息
            self.status_pub.publish(f"已抓取物体: {self.servo_target_object['id']}")
            return
        
        # 计算当前末端位置与目标物体的距离
        dx = target_pos[0] - self.current_end_position[0]
        dy = target_pos[1] - self.current_end_position[1]
        dz = target_pos[2] - self.current_end_position[2]
        
        # 计算移动方向（单位向量）
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance > 0:
            direction = [dx / distance, dy / distance, dz / distance]
        else:
            direction = [0, 0, 1]  # 默认向上移动
        
        # 计算下一个位置（按照设定的接近速度移动）
        next_pos = [
            self.current_end_position[0] + direction[0] * self.approach_speed,
            self.current_end_position[1] + direction[1] * self.approach_speed,
            self.current_end_position[2] + direction[2] * self.approach_speed
        ]
        
        # 移动到下一个位置
        self.move_to_position(next_pos)
    
    def move_to_position(self, position):
        """移动机械臂末端到指定位置"""
        # 这里应该实现逆运动学计算，将位置转换为关节角度
        # 由于逆运动学复杂，这里只是一个简化示例
        # 实际应用中应使用完整的运动学模型
        
        # 简化的逆运动学计算（仅作示例，实际应使用完整的运动学模型）
        x, y, z = position
        
        # 计算底座旋转角度
        theta1 = math.atan2(y, x)
        
        # 计算伸缩关节长度（简化）
        d2 = math.sqrt(x**2 + y**2) * 0.8
        
        # 其他关节角度（简化）
        theta3 = math.asin((0.5 - z) / 0.2)
        theta4 = -theta3 / 2
        
        # 末端伸缩长度
        d6 = 5
        
        # 创建关节命令消息
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = ["arm1_joint1", "arm1_joint2", "arm1_joint3", "arm1_joint4", "arm1_joint5", "arm1_joint6"]
        joint_msg.position = [theta1, d2, theta3, theta4, math.pi/2, d6]
        
        # 发布关节命令
        self.joint_command_pub.publish(joint_msg)
        
        # 限制日志输出频率
        if hasattr(self, 'last_move_log_time') and (rospy.Time.now() - self.last_move_log_time).to_sec() < 1.0:
            pass
        else:
            rospy.loginfo(f"移动机械臂到位置: [{x:.2f}, {y:.2f}, {z:.2f}]")
            self.last_move_log_time = rospy.Time.now()

def main():
    """主函数"""
    try:
        # 创建视觉伺服控制器
        controller = VisualServoController()
        
        # 进入ROS主循环
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 