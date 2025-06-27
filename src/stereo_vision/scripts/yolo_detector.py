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
    
    def __init__(self, model_name="yolov8n.pt", conf_threshold=0.3, simulation_mode=False):
        """
        初始化YOLOv8检测器
        
        Args:
            model_name: YOLO模型名称或路径
            conf_threshold: 置信度阈值
            simulation_mode: 是否使用模拟模式
        """
        # 初始化ROS节点
        rospy.init_node('yolo_detector', anonymous=True)
        
        # 保存参数
        self.conf_threshold = conf_threshold
        self.simulation_mode = simulation_mode
        self.model_name = model_name
        
        # 重叠检测合并参数
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.6)  # 更高的IoU阈值，更容易合并重叠检测
        self.same_class_only = rospy.get_param('~same_class_only', True)  # 是否只合并相同类别的检测
        
        # 创建图像转换桥
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        if not self.simulation_mode:
            try:
                # 尝试加载模型
                rospy.loginfo(f"正在加载YOLO模型: {model_name}")
                self.model = YOLO(model_name)
                rospy.loginfo(f"YOLO模型加载成功: {model_name}")
                
                # 获取类别名称
                self.class_names = self.model.names
                rospy.loginfo(f"加载了 {len(self.class_names)} 个类别")
                
                # 设置模型加载标志为True
                self.model_loaded = True
            except Exception as e:
                rospy.logerr(f"加载YOLO模型失败: {str(e)}")
                rospy.logwarn("切换到模拟模式")
                self.simulation_mode = True
                self.class_names = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 
                                  6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 10: 'fire hydrant', 
                                  11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat', 
                                  16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear', 
                                  22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 
                                  27: 'tie', 28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard', 
                                  32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove', 
                                  36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 
                                  40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 
                                  45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 
                                  50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 
                                  55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed', 
                                  60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 
                                  65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave', 
                                  69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 
                                  74: 'clock', 75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 
                                  79: 'toothbrush'}
        else:
            # 模拟模式下使用COCO类别
            rospy.loginfo("使用模拟模式")
            self.class_names = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 
                              6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light', 10: 'fire hydrant', 
                              11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat', 
                              16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow', 20: 'elephant', 21: 'bear', 
                              22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 
                              27: 'tie', 28: 'suitcase', 29: 'frisbee', 30: 'skis', 31: 'snowboard', 
                              32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove', 
                              36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle', 
                              40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 
                              45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange', 
                              50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 
                              55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed', 
                              60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 
                              65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave', 
                              69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 
                              74: 'clock', 75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 
                              79: 'toothbrush'}
        
        # 创建OpenCV桥接器
        self.bridge = CvBridge()
        
        # 初始化YOLO模型
        self.model_loaded = False
        
        # 打印Python路径以便调试
        rospy.loginfo(f"Python路径: {sys.path}")
        
        # 上次日志记录时间
        self.last_log_time = rospy.Time.now()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # 订阅控制话题
        self.control_sub = rospy.Subscriber('/yolo/status', Bool, self.control_callback)
        
        # 创建检测结果发布器
        self.detection_pub = rospy.Publisher('/detections/image', Image, queue_size=10)
        self.detection_image_pub = rospy.Publisher('/detections/image', Image, queue_size=10)  # 确保这个发布器存在
        self.poses_pub = rospy.Publisher('/detections/poses', PoseArray, queue_size=10)
        
        rospy.loginfo(f"YOLO目标检测器已初始化，检测状态: {'启用' if not self.simulation_mode else '禁用'}, 模式: {'模拟' if self.simulation_mode else '正常'}")
    
    def control_callback(self, msg):
        """处理控制消息"""
        new_state = msg.data
        if new_state != not self.simulation_mode:
            self.simulation_mode = not new_state
            rospy.loginfo(f"YOLO检测已{'启用' if not self.simulation_mode else '禁用'}")
    
    def image_callback(self, msg):
        """处理图像消息"""
        if self.simulation_mode:
            # 如果使用模拟模式，直接转发原始图像
            self.detection_pub.publish(msg)
            return
            
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
        if self.simulation_mode:
            # 使用模拟的检测结果
            return self.simulate_detections(image)
        else:
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
                # 出错时回退到模拟模式
                return self.simulate_detections(image)
    
    def simulate_detections(self, image):
        """
        生成模拟的检测结果
        
        Args:
            image: OpenCV格式的图像
            
        Returns:
            detections: 模拟的检测结果列表
        """
        # 创建一些模拟的检测结果
        height, width = image.shape[:2]
        
        # 模拟检测到的物体
        detections = []
        
        # 模拟常见的COCO对象
        common_objects = [39, 41, 64, 73, 67, 62, 0]  # 瓶子、杯子、鼠标、笔记本、手机等
        
        # 模拟检测到的物体1（在图像左侧）
        if np.random.random() > 0.3:  # 70%的概率检测到
            x1 = int(width * 0.1)
            y1 = int(height * 0.4)
            x2 = int(width * 0.3)
            y2 = int(height * 0.6)
            conf = np.random.uniform(0.6, 0.9)
            class_id = np.random.choice(common_objects)  # 随机选择常见物体
            detections.append([x1, y1, x2, y2, conf, class_id])
        
        # 模拟检测到的物体2（在图像右侧）
        if np.random.random() > 0.5:  # 50%的概率检测到
            x1 = int(width * 0.6)
            y1 = int(height * 0.3)
            x2 = int(width * 0.8)
            y2 = int(height * 0.7)
            conf = np.random.uniform(0.5, 0.8)
            class_id = np.random.choice(common_objects)  # 随机选择常见物体
            detections.append([x1, y1, x2, y2, conf, class_id])
        
        return detections
    
    def draw_detections(self, image, detections):
        """
        在图像上绘制检测结果
        
        Args:
            image: OpenCV格式的图像
            detections: 检测结果列表
            
        Returns:
            result_image: 绘制了检测框的图像
        """
        result_image = image.copy()
        
        # 为不同类别设置不同颜色
        colors = [
            (0, 255, 0),    # 绿色
            (0, 0, 255),    # 红色
            (255, 0, 0),    # 蓝色
            (255, 255, 0),  # 青色
            (0, 255, 255),  # 黄色
            (255, 0, 255),  # 紫色
            (128, 0, 0),    # 深蓝色
            (0, 128, 0),    # 深绿色
            (0, 0, 128),    # 深红色
            (128, 128, 0),  # 橄榄色
            (128, 0, 128),  # 紫色
            (0, 128, 128),  # 青绿色
            (192, 192, 192),# 银色
            (128, 128, 128),# 灰色
            (64, 64, 64)    # 深灰色
        ]
        
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            
            # 转换为整数坐标
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # 获取类别名称和颜色
            class_name = self.class_names[int(class_id)] if int(class_id) < len(self.class_names) else f"class{int(class_id)}"
            color = colors[int(class_id) % len(colors)]
            
            # 绘制边界框
            cv2.rectangle(result_image, (x1, y1), (x2, y2), color, 2)
            
            # 绘制半透明背景
            overlay = result_image.copy()
            cv2.rectangle(overlay, (x1, y1 - 30), (x1 + len(class_name) * 11 + 70, y1), color, -1)
            cv2.addWeighted(overlay, 0.7, result_image, 0.3, 0, result_image)
            
            # 绘制类别名称和置信度
            label = f"{class_name} {conf:.2f}"
            cv2.putText(result_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        return result_image
    
    def publish_poses(self, detections, header):
        """
        发布检测到的物体位姿
        
        Args:
            detections: 检测结果列表
            header: 图像消息头
        """
        pose_array = PoseArray()
        pose_array.header = header
        
        # 收集类别名称，添加到frame_id中
        class_names = []
        
        # 如果没有检测到物体，发布空的PoseArray
        if not detections:
            rospy.loginfo("没有检测到物体，发布空的PoseArray")
            self.poses_pub.publish(pose_array)
            return
        
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            
            # 计算物体中心点
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # 估计深度（这里使用模拟值，实际应该从深度图或立体视觉获取）
            # 假设深度与物体大小成反比
            object_width = x2 - x1
            object_height = y2 - y1
            object_size = np.sqrt(object_width * object_height)
            depth = 100.0 / (object_size / 100.0)  # 模拟深度值（厘米）
            
            # 创建位姿
            pose = Pose()
            
            # 设置位置（假设相机坐标系：X向右，Y向下，Z向前）
            # 将像素坐标转换为相机坐标系（简化版本）
            image_width = 640  # 假设图像宽度
            image_height = 480  # 假设图像高度
            fx = 500.0  # 假设相机焦距
            fy = 500.0
            cx = image_width / 2
            cy = image_height / 2
            
            # 计算相机坐标系中的位置
            x = (center_x - cx) * depth / fx
            y = (center_y - cy) * depth / fy
            z = depth
            
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            
            # 设置姿态（默认朝下）
            pose.orientation.w = 1.0
            
            pose_array.poses.append(pose)
            
            # 获取类别名称并添加到列表
            class_name = self.class_names[int(class_id)] if int(class_id) < len(self.class_names) else f"class{int(class_id)}"
            class_names.append(class_name)
        
        # 将类别名称添加到frame_id
        if class_names:
            pose_array.header.frame_id = ','.join(class_names)
            # 调试输出 - 显示检测到的物体类别
            rospy.loginfo(f"发布检测结果: {len(class_names)}个物体 - {','.join(class_names)}")
        
        # 发布位姿数组
        self.poses_pub.publish(pose_array)
        
        # 调试输出 - 确认消息已发布
        rospy.loginfo(f"已发布PoseArray消息，包含{len(pose_array.poses)}个物体位姿")

    def process_detections(self, results, min_confidence=0.3):
        """
        处理YOLO检测结果，提取位置信息
        
        Args:
            results: YOLO检测结果
            min_confidence: 最小置信度阈值
            
        Returns:
            检测结果列表，每个元素为 [x1, y1, x2, y2, conf, class_id]
        """
        # 检查结果是否有效
        if results is None or len(results) == 0:
            rospy.loginfo("没有检测到物体")
            return []
        
        # 提取检测结果
        boxes = results[0].boxes
        
        # 如果没有检测到物体，返回空列表
        if len(boxes) == 0:
            rospy.loginfo("没有检测到物体")
            return []
        
        # 提取边界框、置信度和类别
        detections = []
        for box in boxes:
            # 获取边界框坐标
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            
            # 获取置信度
            conf = box.conf[0].cpu().numpy()
            
            # 获取类别ID
            class_id = int(box.cls[0].cpu().numpy())
            
            # 仅保留置信度高于阈值的检测结果
            if conf >= min_confidence:
                detections.append([x1, y1, x2, y2, conf, class_id])
        
        # 合并重叠的检测结果
        merged_detections = self.merge_overlapping_detections(detections)
        
        # 记录检测和合并结果
        rospy.loginfo(f"检测到 {len(detections)} 个物体，合并后 {len(merged_detections)} 个物体")
        
        return merged_detections
    
    def merge_overlapping_detections(self, detections, iou_threshold=None, same_class_only=None):
        """
        合并重叠的检测结果
        
        Args:
            detections: 检测结果列表，每个元素为 [x1, y1, x2, y2, conf, class_id]
            iou_threshold: IoU阈值，高于此值的检测将被合并，默认使用类的参数
            same_class_only: 是否只合并相同类别的检测结果，默认使用类的参数
            
        Returns:
            合并后的检测结果列表
        """
        if len(detections) <= 1:
            return detections
        
        # 使用类参数或传入参数
        if iou_threshold is None:
            iou_threshold = self.iou_threshold
        if same_class_only is None:
            same_class_only = self.same_class_only
        
        # 记录合并参数
        rospy.loginfo(f"合并重叠检测: IoU阈值={iou_threshold}, 仅相同类别={same_class_only}")
        
        # 按置信度降序排序
        detections.sort(key=lambda x: x[4], reverse=True)
        
        # 标记需要保留的检测结果
        keep = [True] * len(detections)
        
        # 合并重叠的检测结果
        for i in range(len(detections)):
            if not keep[i]:
                continue
                
            # 获取当前检测结果
            x1i, y1i, x2i, y2i, confi, class_i = detections[i]
            area_i = (x2i - x1i) * (y2i - y1i)
            
            # 与其他检测结果比较
            for j in range(i + 1, len(detections)):
                if not keep[j]:
                    continue
                    
                # 获取比较的检测结果
                x1j, y1j, x2j, y2j, confj, class_j = detections[j]
                
                # 如果只合并相同类别的检测结果，且类别不同，则跳过
                if same_class_only and class_i != class_j:
                    continue
                
                # 计算交集
                xx1 = max(x1i, x1j)
                yy1 = max(y1i, y1j)
                xx2 = min(x2i, x2j)
                yy2 = min(y2i, y2j)
                
                # 计算交集面积
                w = max(0, xx2 - xx1)
                h = max(0, yy2 - yy1)
                inter = w * h
                
                # 计算并集面积
                area_j = (x2j - x1j) * (y2j - y1j)
                union = area_i + area_j - inter
                
                # 计算IoU
                iou = inter / union if union > 0 else 0
                
                # 如果IoU大于阈值，合并检测结果（保留置信度高的）
                if iou > iou_threshold:
                    keep[j] = False
                    rospy.loginfo(f"合并检测: 类别 {int(class_i)} 和 {int(class_j)}, IoU={iou:.2f}")
        
        # 返回保留的检测结果
        merged = [detections[i] for i in range(len(detections)) if keep[i]]
        rospy.loginfo(f"合并前: {len(detections)} 个检测, 合并后: {len(merged)} 个检测")
        return merged

    def detect_and_publish(self, image_msg):
        """
        检测图像中的物体并发布结果
        
        Args:
            image_msg: ROS图像消息
        """
        try:
            # 将ROS图像转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # 检测物体
            detections = self.detect_objects(cv_image)
            
            # 记录检测结果
            if len(detections) > 0:
                class_names = [self.class_names[int(det[5])] if int(det[5]) < len(self.class_names) else f"class{int(det[5])}" for det in detections]
                rospy.loginfo(f"检测到 {len(detections)} 个物体: {', '.join(class_names)}")
            else:
                rospy.loginfo("未检测到物体")
            
            # 绘制检测结果
            annotated_image = self.draw_detections(cv_image, detections)
            
            # 发布检测结果图像
            detection_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            detection_image_msg.header = image_msg.header
            self.detection_image_pub.publish(detection_image_msg)
            
            # 发布物体位姿
            self.publish_poses(detections, image_msg.header)
            
        except Exception as e:
            # 限制错误日志频率
            current_time = rospy.Time.now()
            if not hasattr(self, 'last_error_time') or (current_time - self.last_error_time).to_sec() > 5.0:
                rospy.logerr(f"检测和发布过程中出错: {str(e)}")
                self.last_error_time = current_time

def main():
    """主函数"""
    try:
        # 获取ROS参数
        model_name = rospy.get_param('~model', 'yolov8s.pt')
        conf_threshold = rospy.get_param('~conf', 0.3)
        simulation_mode = rospy.get_param('~simulation_mode', False)
        
        # 创建YOLOv8检测器
        detector = YoloDetector(model_name, conf_threshold, simulation_mode)
        
        # 启动ROS节点
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("YOLO检测器节点已关闭")
    except Exception as e:
        rospy.logerr(f"YOLO检测器节点出错: {str(e)}")
        
if __name__ == '__main__':
    main() 