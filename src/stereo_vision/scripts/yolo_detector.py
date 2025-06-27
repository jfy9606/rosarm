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
from ultralytics import YOLO

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
    """YOLO目标检测器，用于检测图像中的物体"""
    
    def __init__(self, model_name="yolov8n.pt", conf_threshold=0.25):
        """
        初始化YOLOv8检测器
        
        Args:
            model_name: YOLO模型名称或路径
            conf_threshold: 置信度阈值
        """
        # 初始化ROS节点
        rospy.init_node('yolo_detector', anonymous=True)
        
        # 保存参数
        self.conf_threshold = conf_threshold
        self.model_name = model_name
        
        # 重叠检测合并参数
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)  # IoU阈值
        self.same_class_only = rospy.get_param('~same_class_only', False)  # 是否只合并相同类别的检测
        
        # 获取图像话题参数
        self.image_topic = rospy.get_param('~image_topic', '/stereo_camera/image_merged')
        
        # 创建图像转换桥
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        try:
            # 尝试加载模型
            rospy.loginfo(f"正在加载YOLO模型: {model_name}")
            
            # 尝试在多个路径中查找模型
            model_paths = [
                model_name,  # 直接使用提供的路径
                os.path.join(os.path.dirname(os.path.abspath(__file__)), model_name),  # 当前脚本目录
                os.path.join(os.path.expanduser('~'), '.local', 'lib', 'python3.8', 'site-packages', 'ultralytics', 'models', model_name),  # 安装目录
            ]
            
            model_loaded = False
            for path in model_paths:
                try:
                    if os.path.exists(path):
                        rospy.loginfo(f"找到模型路径: {path}")
                        self.model = YOLO(path)
                        model_loaded = True
                        break
                except Exception as e:
                    rospy.logwarn(f"在路径 {path} 加载模型失败: {e}")
            
            if not model_loaded:
                rospy.loginfo("尝试直接从ultralytics加载模型")
                self.model = YOLO(model_name)
                model_loaded = True
            
            rospy.loginfo(f"YOLO模型加载成功: {model_name}")
            
            # 获取类别名称
            self.class_names = self.model.names
            rospy.loginfo(f"加载了 {len(self.class_names)} 个类别")
            
            # 设置模型加载标志为True
            self.model_loaded = True
        except Exception as e:
            rospy.logerr(f"加载YOLO模型失败: {str(e)}")
            self.model_loaded = False
            
        # 创建OpenCV桥接器
        self.bridge = CvBridge()
        
        # 打印Python路径以便调试
        rospy.loginfo(f"Python路径: {sys.path}")
        
        # 上次日志记录时间
        self.last_log_time = rospy.Time.now()
        
        # 订阅图像话题（使用合成图像）
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.loginfo(f"已订阅图像话题: {self.image_topic}")
        
        # 订阅控制话题
        self.control_sub = rospy.Subscriber('/yolo/status', Bool, self.control_callback)
        
        # 创建检测结果发布器
        self.detection_pub = rospy.Publisher('/detections/image', Image, queue_size=10)
        self.detection_image_pub = rospy.Publisher('/detections/image', Image, queue_size=10)  # 确保这个发布器存在
        self.poses_pub = rospy.Publisher('/detections/poses', PoseArray, queue_size=10)
        
        rospy.loginfo(f"YOLO目标检测器已初始化")
    
    def control_callback(self, msg):
        """处理控制消息"""
        # 这里可以保留控制回调，但不再处理模拟模式
        enabled = msg.data
        rospy.loginfo(f"YOLO检测状态更改为: {'启用' if enabled else '禁用'}")
    
    def image_callback(self, msg):
        """处理图像消息"""
        if not self.model_loaded:
            rospy.logerr("YOLO模型未加载，无法进行检测")
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
                results = self.model(image, verbose=False)
                
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
            
            # 设置位置
            pose.position.x = center_x / 100.0  # 转换到米单位（假设图像单位为像素）
            pose.position.y = center_y / 100.0
            pose.position.z = 0.0  # Z坐标设为0
            
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
            for box in boxes:
                # 获取置信度
                conf = float(box.conf)
                
                # 仅保留高于阈值的结果
                if conf >= min_confidence:
                    # 获取边界框坐标
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    # 获取类别ID
                    class_id = int(box.cls)
                    
                    # 将检测结果添加到列表中
                    detections.append([x1, y1, x2, y2, conf, class_id])
        
        # 合并重叠的检测结果
        detections = self.merge_overlapping_detections(detections)
                
        return detections
            
    def merge_overlapping_detections(self, detections, iou_threshold=None, same_class_only=None):
        """
        合并重叠的检测结果
        
        Args:
            detections: 检测结果列表，每个元素包含 [x1, y1, x2, y2, conf, class_id]
            iou_threshold: IoU阈值，如果为None则使用默认值
            same_class_only: 是否只合并相同类别的检测，如果为None则使用默认值
            
        Returns:
            merged_detections: 合并后的检测结果列表
        """
        # 如果检测结果少于2个，直接返回
        if len(detections) < 2:
            return detections
        
        # 使用默认参数或传入的参数
        iou_threshold = iou_threshold if iou_threshold is not None else self.iou_threshold
        same_class_only = same_class_only if same_class_only is not None else self.same_class_only
        
        # 将检测结果转换为NumPy数组以便处理
        dets = np.array(detections)
        
        # 按照置信度排序（降序）
        indices = np.argsort(-dets[:, 4])
        dets = dets[indices]
        
        # 创建保留标记数组
        keep = np.ones(len(dets), dtype=bool)
        
        # 遍历检测结果
        for i in range(len(dets) - 1):
            # 如果当前检测已被标记为删除，跳过
            if not keep[i]:
                continue
                
            # 获取当前检测的边界框和类别
            box1 = dets[i, :4]
            class1 = dets[i, 5]
            
            # 计算当前检测与后续检测的IoU
            for j in range(i + 1, len(dets)):
                # 如果后续检测已被标记为删除，跳过
                if not keep[j]:
                    continue
                    
                # 获取后续检测的边界框和类别
                box2 = dets[j, :4]
                class2 = dets[j, 5]
                
                # 如果只合并相同类别的检测且类别不同，跳过
                if same_class_only and class1 != class2:
                    continue
                
                # 计算IoU
                iou = self.calculate_iou(box1, box2)
                
                # 如果IoU超过阈值，标记后续检测为删除
                if iou > iou_threshold:
                    keep[j] = False
        
        # 返回保留的检测结果
        return dets[keep].tolist()
    
    def calculate_iou(self, box1, box2):
        """
        计算两个边界框的IoU（交并比）
        
        Args:
            box1: 第一个边界框 [x1, y1, x2, y2]
            box2: 第二个边界框 [x1, y1, x2, y2]
            
        Returns:
            iou: IoU值，范围[0, 1]
        """
        # 计算交集矩形
        x_left = max(box1[0], box2[0])
        y_top = max(box1[1], box2[1])
        x_right = min(box1[2], box2[2])
        y_bottom = min(box1[3], box2[3])
        
        # 如果边界框没有重叠，返回0
        if x_right < x_left or y_bottom < y_top:
            return 0.0
        
        # 计算交集面积
        intersection_area = (x_right - x_left) * (y_bottom - y_top)
        
        # 计算并集面积
        box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
        box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union_area = box1_area + box2_area - intersection_area
        
        # 计算IoU
        iou = intersection_area / union_area
        
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
            
            # 发布姿态数据
            self.publish_poses(detections, image_msg.header)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV桥接错误: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理图像和发布结果时出错: {str(e)}")

def main():
    try:
        # 获取YOLO模型名称
        model_name = rospy.get_param('~model', 'yolov8n.pt')
        
        # 获取置信度阈值
        conf_threshold = rospy.get_param('~conf', 0.25)
        
        # 创建并启动检测器
        detector = YoloDetector(model_name, conf_threshold)
        
        # 进入ROS主循环
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("YOLO检测节点已关闭")
        pass

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr(f"YOLO检测节点出现异常: {str(e)}") 