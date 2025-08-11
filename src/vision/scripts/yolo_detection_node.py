#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import numpy as np

try:
    from ultralytics import YOLO
    import torch
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


class YOLODetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        # 检查依赖
        if not YOLO_AVAILABLE:
            self.get_logger().error("ultralytics and torch are required but not installed. Please install: pip install ultralytics torch")
            return
        
        # 参数声明
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('output_overlay_topic', '/vision/overlay')
        self.declare_parameter('output_detections_topic', '/vision/detections')
        self.declare_parameter('device', 'auto')  # 'auto', 'cpu', 'cuda'
        
        # 获取参数
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        overlay_topic = self.get_parameter('output_overlay_topic').get_parameter_value().string_value
        detections_topic = self.get_parameter('output_detections_topic').get_parameter_value().string_value
        device_setting = self.get_parameter('device').get_parameter_value().string_value
        
        # 确定设备
        if device_setting == 'auto':
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = device_setting
        
        self.get_logger().info(f"Using device: {self.device}")
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 加载YOLO模型
        self.model = None
        self.class_names = []
        if not self.load_yolo_model():
            self.get_logger().error("Failed to load YOLO model")
            return
        
        # 订阅相机话题
        self.camera_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # 发布器
        self.overlay_pub = self.create_publisher(Image, overlay_topic, 10)
        self.detections_pub = self.create_publisher(String, detections_topic, 10)
        
        # 检测结果缓存
        self.detection_results = []
        
        self.get_logger().info(f"YOLO Detection Node started. Subscribed to {camera_topic}, publishing to {overlay_topic} and {detections_topic}")
    
    def load_yolo_model(self):
        """加载YOLO模型"""
        try:
            if self.model_path and self.model_path.strip():
                # 使用指定的模型路径
                self.get_logger().info(f"Loading YOLO model from: {self.model_path}")
                self.model = YOLO(self.model_path)
            else:
                # 尝试自动下载yolo11n.pt
                self.get_logger().info("No model path specified, attempting to load/download yolo11n.pt")
                try:
                    self.model = YOLO('yolo11n.pt')
                except Exception as e:
                    self.get_logger().warn(f"Failed to load yolo11n.pt: {e}, trying yolov8n.pt as fallback")
                    self.model = YOLO('yolov8n.pt')
            
            # 移动模型到指定设备
            self.model.to(self.device)
            
            # 获取类别名称
            if hasattr(self.model, 'names'):
                self.class_names = list(self.model.names.values())
            else:
                # 默认COCO类别（如果模型没有names属性）
                self.class_names = [
                    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
                    'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
                    'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
                    'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
                    'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
                    'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake',
                    'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
                    'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
                    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
                ]
            
            self.get_logger().info(f"YOLO model loaded successfully. Model supports {len(self.class_names)} classes.")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            return False
    
    def image_callback(self, msg):
        """处理输入图像并执行检测"""
        if not self.model:
            return
        
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 执行YOLO推理
            results = self.model(cv_image, conf=self.confidence_threshold, device=self.device)
            
            # 处理检测结果
            self.detection_results = []
            overlay_image = cv_image.copy()
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for i, box in enumerate(boxes):
                        # 提取边界框信息
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        conf = box.conf[0].cpu().numpy()
                        cls_id = int(box.cls[0].cpu().numpy())
                        
                        # 获取类别名称
                        cls_name = self.class_names[cls_id] if cls_id < len(self.class_names) else f"class_{cls_id}"
                        
                        # 计算中心点
                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)
                        
                        # 保存检测结果（与纯Python版本格式对齐）
                        detection = {
                            'id': i + 1,
                            'class': cls_name,
                            'confidence': float(conf),
                            'bbox': [int(x1), int(y1), int(x2), int(y2)],
                            'center_x': center_x,
                            'center_y': center_y
                        }
                        self.detection_results.append(detection)
                        
                        # 绘制边界框（与纯Python版本样式对齐）
                        color = (0, 255, 0)  # 绿色
                        cv2.rectangle(overlay_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                        
                        # 绘制标签："{i+1}:{cls} {conf:.2f}"
                        label = f"{i+1}:{cls_name} {conf:.2f}"
                        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                        cv2.rectangle(overlay_image, (int(x1), int(y1) - label_size[1] - 10), 
                                    (int(x1) + label_size[0], int(y1)), color, -1)
                        cv2.putText(overlay_image, label, (int(x1), int(y1) - 5), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
                        
                        # 绘制中心点
                        cv2.circle(overlay_image, (center_x, center_y), 5, (255, 0, 0), -1)
                        
                        # 在中心点附近显示坐标
                        coord_text = f"({center_x},{center_y})"
                        cv2.putText(overlay_image, coord_text, (center_x + 10, center_y), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # 发布叠加图像
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay_image, "bgr8")
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)
            
            # 发布检测结果（JSON格式）
            detections_json = json.dumps(self.detection_results, indent=2)
            detections_msg = String()
            detections_msg.data = detections_json
            self.detections_pub.publish(detections_msg)
            
            if self.detection_results:
                self.get_logger().info(f"Detected {len(self.detection_results)} objects")
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def get_detection_results(self):
        """获取当前检测结果（与纯Python版本接口对齐）"""
        return self.detection_results


def main(args=None):
    rclpy.init(args=args)
    
    node = YOLODetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()