#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import os
import sys
import time
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse

# 尝试导入ultralytics
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    rospy.logwarn("无法导入ultralytics，将使用模拟检测")

# 通过环境变量获取Python路径
# 获取PYTHONPATH环境变量并将其添加到sys.path
if 'PYTHONPATH' in os.environ:
    env_paths = os.environ['PYTHONPATH'].split(':')
    for path in env_paths:
        if path and path not in sys.path and os.path.exists(path):
            sys.path.append(path)
            rospy.loginfo(f"从PYTHONPATH添加路径: {path}")

# 获取Python可执行文件的site-packages路径
try:
    import site
    site_packages = site.getsitepackages()
    for path in site_packages:
        if path not in sys.path and os.path.exists(path):
            sys.path.append(path)
            rospy.loginfo(f"添加site-packages路径: {path}")
except Exception as e:
    rospy.logwarn(f"获取site-packages路径失败: {e}")

# 添加用户site-packages路径
user_site = site.getusersitepackages() if hasattr(site, 'getusersitepackages') else None
if user_site and user_site not in sys.path and os.path.exists(user_site):
    sys.path.append(user_site)
    rospy.loginfo(f"添加用户site-packages路径: {user_site}")

class YoloDetector:
    """
    YOLO目标检测器，用于检测图像中的物体并发布检测结果
    """
    
    def __init__(self, model_name="yolov8n.pt", conf_threshold=0.25):
        """
        初始化YOLO检测器
        
        Args:
            model_name: YOLO模型文件名
            conf_threshold: 检测置信度阈值
        """
        rospy.init_node('yolo_detector', anonymous=True)
        
        # 初始化参数
        self.model_name = rospy.get_param('~model_path', model_name)
        self.conf_threshold = rospy.get_param('~conf_threshold', conf_threshold)
        self.image_topic = rospy.get_param('~image_topic', '/stereo_camera/image_raw')
        self.detection_enabled = rospy.get_param('~detection_enabled', True)
        
        # 初始化OpenCV桥接器
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        self.model_loaded = False
        self.model = None
        self.class_names = {}
        
        # 尝试加载YOLO模型
        if YOLO_AVAILABLE:
            try:
                # 尝试加载指定的模型
                if os.path.exists(str(self.model_name)):
                    self.model = YOLO(str(self.model_name))
                    self.model_loaded = True
                    rospy.loginfo(f"成功加载YOLO模型: {self.model_name}")
                else:
                    # 如果指定模型不存在，尝试加载默认模型
                    rospy.logwarn(f"模型文件不存在: {self.model_name}，尝试使用默认模型yolov8n.pt")
                    try:
                        self.model = YOLO('yolov8n.pt')
                        self.model_loaded = True
                        rospy.loginfo("成功加载默认YOLO模型")
                    except Exception as e:
                        rospy.logerr(f"加载默认YOLO模型失败: {str(e)}")
                
                # 获取类别名称
                if self.model_loaded and hasattr(self.model, 'names'):
                    self.class_names = self.model.names
                    rospy.loginfo(f"类别: {self.class_names}")
            except Exception as e:
                rospy.logerr(f"加载YOLO模型时出错: {str(e)}")
        else:
            rospy.logwarn("未安装ultralytics，无法使用YOLO模型")
        
        # 创建服务
        self.enable_service = rospy.Service('/yolo_detector/enable', SetBool, self.enable_detection_callback)
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.loginfo(f"已订阅图像话题: {self.image_topic}")
        
        # 订阅控制话题
        self.control_sub = rospy.Subscriber('/yolo/status', Bool, self.control_callback)
        
        # 创建检测结果发布器
        self.detection_pub = rospy.Publisher('/detections/image', Image, queue_size=10)
        self.detection_image_pub = rospy.Publisher('/detections/image', Image, queue_size=10)  # 确保这个发布器存在
        self.poses_pub = rospy.Publisher('/detections/poses', PoseArray, queue_size=10)
        self.status_pub = rospy.Publisher('/yolo/status', Bool, queue_size=10, latch=True)
        
        # 发布初始状态
        self.publish_status()
        
        # 创建定时器，定期发布状态
        rospy.Timer(rospy.Duration(5), self.publish_status_timer)
        
        rospy.loginfo(f"YOLO目标检测器已初始化")
    
    def enable_detection_callback(self, req):
        """处理启用/禁用检测的服务请求"""
        self.detection_enabled = req.data
        rospy.loginfo(f"YOLO检测状态更改为: {'启用' if self.detection_enabled else '禁用'}")
        
        # 发布状态更新
        self.publish_status()
        
        return SetBoolResponse(True, f"YOLO检测已{'启用' if self.detection_enabled else '禁用'}")
    
    def publish_status_timer(self, event):
        """定期发布状态"""
        self.publish_status()
    
    def publish_status(self):
        """发布当前检测状态"""
        status_msg = Bool()
        status_msg.data = self.detection_enabled and self.model_loaded
        self.status_pub.publish(status_msg)
    
    def control_callback(self, msg):
        """处理控制消息"""
        enabled = msg.data
        if enabled != self.detection_enabled:
            self.detection_enabled = enabled
            rospy.loginfo(f"YOLO检测状态更改为: {'启用' if self.detection_enabled else '禁用'}")
    
    def image_callback(self, msg):
        """处理图像消息"""
        if not self.model_loaded or not self.detection_enabled:
            # 如果模型未加载或检测被禁用，直接转发原始图像
            self.detection_pub.publish(msg)
            return
        
        try:
            # 使用detect_and_publish方法处理图像并发布结果
            self.detect_and_publish(msg)
        except Exception as e:
            # 异常也需要限流，避免刷屏
            current_time = rospy.Time.now()
            if not hasattr(self, 'last_error_time') or (current_time - self.last_error_time).to_sec() > 10.0:
                rospy.logerr(f"处理图像时出错: {str(e)}")
                self.last_error_time = current_time
    
    def detect_objects(self, image):
        """
        使用YOLO模型检测图像中的物体
        
        Args:
            image: OpenCV格式的图像
            
        Returns:
            detections: 检测结果列表，每个元素包含 [x1, y1, x2, y2, conf, class_id]
        """
        # 使用实际的YOLO模型进行检测
        try:
            # 检查图像是否有效
            if image is None or image.size == 0:
                rospy.logwarn("收到无效图像")
                return []
            
            # 静默检测，不输出任何日志
            with open(os.devnull, 'w') as f:
                # 暂时将stdout重定向到null设备
                original_stdout = sys.stdout
                sys.stdout = f
                
                # 执行检测
                if self.model is not None:
                    results = self.model(image, verbose=False)
                else:
                    return []
                
                # 恢复stdout
                sys.stdout = original_stdout
            
            # 处理并合并检测结果
            detections = self.process_detections(results, self.conf_threshold)
            
            return detections
            
        except Exception as e:
            # 异常也需要限流，避免刷屏
            current_time = rospy.Time.now()
            if not hasattr(self, 'last_detect_error_time') or (current_time - self.last_detect_error_time).to_sec() > 10.0:
                rospy.logerr(f"YOLO检测出错: {str(e)}")
                self.last_detect_error_time = current_time
            return []
    
    def draw_detections(self, image, detections):
        """
        在图像上绘制检测结果
        
        Args:
            image: OpenCV格式的图像
            detections: 检测结果列表，每个元素包含 [x1, y1, x2, y2, conf, class_id]
            
        Returns:
            annotated_image: 带有检测框的图像
        """
        # 创建图像副本
        annotated_image = image.copy()
        
        # 遍历检测结果
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            
            # 确保坐标是整数
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # 获取类别名称
            class_id = int(class_id)
            class_name = self.class_names[class_id] if class_id in self.class_names else f"class_{class_id}"
            
            # 设置颜色（基于类别ID）
            color = (int(255 * (class_id % 3 / 3.0)), 
                     int(255 * ((class_id + 1) % 3 / 3.0)), 
                     int(255 * ((class_id + 2) % 3 / 3.0)))
            
            # 绘制矩形框
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
            
            # 标注类别名称和置信度
            label = f"{class_name}: {conf:.2f}"
            
            # 获取文本框大小
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            
            # 确保文本框在图像范围内
            y1 = max(y1, text_height + baseline)
            
            # 绘制文本框背景
            cv2.rectangle(
                annotated_image, 
                (x1, y1 - text_height - baseline),
                (x1 + text_width, y1),
                color, 
                -1
            )
            
            # 绘制文本
            cv2.putText(
                annotated_image,
                label,
                (x1, y1 - baseline),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2
            )
        
        return annotated_image

    def publish_poses(self, detections, header):
        """
        发布检测到的物体的位姿
        
        Args:
            detections: 检测结果列表，每个元素包含 [x1, y1, x2, y2, conf, class_id]
            header: 图像消息的header，用于设置时间戳和坐标系
        """
        # 创建姿态数组消息
        pose_array = PoseArray()
        pose_array.header = header
        
        # 遍历检测结果
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            
            # 计算矩形框中心坐标
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            
            # 创建姿态消息
            pose = Pose()
            
            # 设置位置 - 这里我们假设深度为1米，实际应用中应该使用真实深度
            # 在stereo_detection_node.py中会使用真实深度替换这些值
            pose.position.x = center_x / 100.0  # 转换到米单位（假设图像单位为像素）
            pose.position.y = center_y / 100.0
            pose.position.z = 1.0  # 默认深度1米
            
            # 设置方向（单位四元数，代表无旋转）
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            
            # 将姿态添加到数组中
            pose_array.poses.append(pose)
            
        # 发布姿态数组
        self.poses_pub.publish(pose_array)
            
    def process_detections(self, results, min_confidence=0.3):
        """
        处理YOLO检测结果
        
        Args:
            results: YOLO检测结果对象
            min_confidence: 最小置信度阈值
            
        Returns:
            detections: 检测结果列表，每个元素包含 [x1, y1, x2, y2, conf, class_id]
        """
        detections = []
        
        # 遍历所有检测结果
        for result in results:
            boxes = result.boxes
            
            # 遍历所有边界框
            for i in range(len(boxes)):
                # 获取边界框坐标
                box = boxes[i].xyxy[0].cpu().numpy()  # 获取xyxy格式的边界框
                x1, y1, x2, y2 = box[:4]
                
                # 获取置信度
                conf = float(boxes[i].conf[0].cpu().numpy())
                
                # 获取类别ID
                class_id = int(boxes[i].cls[0].cpu().numpy())
                
                # 检查置信度是否高于阈值
                if conf >= min_confidence:
                    # 添加到检测结果列表
                    detections.append([x1, y1, x2, y2, conf, class_id])
        
        # 合并重叠的检测框
        detections = self.merge_overlapping_detections(detections)
        
        return detections
    
    def merge_overlapping_detections(self, detections, iou_threshold=None, same_class_only=None):
        """
        合并重叠的检测框
        
        Args:
            detections: 检测结果列表，每个元素包含 [x1, y1, x2, y2, conf, class_id]
            iou_threshold: IoU阈值，高于此阈值的框会被合并
            same_class_only: 是否只合并相同类别的框
            
        Returns:
            merged_detections: 合并后的检测结果列表
        """
        # 如果没有设置阈值，使用默认值
        if iou_threshold is None:
            iou_threshold = 0.5
        
        # 如果没有设置same_class_only，使用默认值
        if same_class_only is None:
            same_class_only = True
        
        # 如果检测结果少于2个，无需合并
        if len(detections) < 2:
            return detections
        
        # 按置信度降序排序
        detections.sort(key=lambda x: x[4], reverse=True)
        
        # 转换为numpy数组，方便处理
        dets = np.array(detections)
        
        # 初始化保留框的索引列表
        keep = []
        
        # 遍历所有框
        while len(dets) > 0:
            # 保留当前置信度最高的框
            keep.append(dets[0])
            
            # 如果只剩一个框，结束循环
            if len(dets) == 1:
                break
            
            # 计算当前框与其他框的IoU
            ious = np.array([self.calculate_iou(dets[0], det) for det in dets[1:]])
            
            # 如果只合并相同类别的框，检查类别是否相同
            if same_class_only:
                # 获取类别ID
                class_id = dets[0][5]
                
                # 创建类别掩码
                class_mask = dets[1:, 5] == class_id
                
                # 应用类别掩码
                ious = ious * class_mask
            
            # 找出IoU低于阈值的框的索引
            indices = np.where(ious < iou_threshold)[0]
            
            # 更新dets，只保留IoU低于阈值的框
            dets = dets[1:][indices]
        
        return keep
    
    def calculate_iou(self, box1, box2):
        """
        计算两个框的IoU
        
        Args:
            box1: 第一个框 [x1, y1, x2, y2, conf, class_id]
            box2: 第二个框 [x1, y1, x2, y2, conf, class_id]
            
        Returns:
            iou: IoU值
        """
        # 提取坐标
        box1_x1, box1_y1, box1_x2, box1_y2 = box1[:4]
        box2_x1, box2_y1, box2_x2, box2_y2 = box2[:4]
        
        # 计算交集区域的坐标
        x1 = max(box1_x1, box2_x1)
        y1 = max(box1_y1, box2_y1)
        x2 = min(box1_x2, box2_x2)
        y2 = min(box1_y2, box2_y2)
        
        # 计算交集区域的面积
        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        
        # 计算两个框的面积
        box1_area = (box1_x2 - box1_x1) * (box1_y2 - box1_y1)
        box2_area = (box2_x2 - box2_x1) * (box2_y2 - box2_y1)
        
        # 计算并集区域的面积
        union = box1_area + box2_area - intersection
        
        # 计算IoU
        iou = intersection / union if union > 0 else 0
        
        return iou
    
    def detect_and_publish(self, image_msg):
        """
        检测图像中的物体并发布结果
        
        Args:
            image_msg: ROS图像消息
        """
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # 检测物体
            detections = self.detect_objects(cv_image)
            
            # 在图像上绘制检测结果
            annotated_image = self.draw_detections(cv_image, detections)
            
            # 将OpenCV图像转换回ROS图像消息
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            detection_msg.header = image_msg.header
            
            # 发布检测结果图像
            self.detection_pub.publish(detection_msg)
            
            # 发布物体位姿
            self.publish_poses(detections, image_msg.header)
            
        except Exception as e:
            rospy.logerr(f"检测和发布时出错: {str(e)}")

def main():
    """主函数"""
    try:
        # 创建YOLO检测器
        detector = YoloDetector()
        
        # 循环处理
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 