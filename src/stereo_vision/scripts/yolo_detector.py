#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from stereo_vision.msg import Detection, DetectionArray

class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 获取参数
        self.model_path = rospy.get_param('~model_path', '')
        self.confidence = rospy.get_param('~confidence', 0.5)
        self.input_topic = rospy.get_param('~input_topic', '/camera/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/camera/detections')
        self.visualization_topic = rospy.get_param('~visualization_topic', '/detection_image')
        
        # 检测开关状态
        self.detection_enabled = rospy.get_param('~enabled', True)
        
        # 初始化模型状态
        self.model = None
        self.model_loaded = False
        
        # 尝试加载YOLO模型
        self.load_yolo_model()
        
        # 发布器
        self.detection_pub = rospy.Publisher(self.output_topic, DetectionArray, queue_size=1)
        self.visualization_pub = rospy.Publisher(self.visualization_topic, Image, queue_size=1)
        
        # 订阅图像
        self.image_sub = rospy.Subscriber(self.input_topic, Image, self.image_callback)
        
        # 订阅检测开关控制
        self.control_sub = rospy.Subscriber('/yolo_detection/status', Bool, self.control_callback)
        
        rospy.loginfo("YOLO目标检测器已初始化，检测状态: %s", "启用" if self.detection_enabled else "禁用")
    
    def control_callback(self, msg):
        """接收YOLO检测开关控制信号"""
        old_state = self.detection_enabled
        self.detection_enabled = msg.data
        
        if old_state != self.detection_enabled:
            rospy.loginfo("YOLO检测状态已更改为: %s", "启用" if self.detection_enabled else "禁用")
    
    def load_yolo_model(self):
        """尝试加载YOLO模型"""
        try:
            # 首先尝试使用PyTorch Hub加载
            try:
                if os.path.exists(self.model_path):
                    import torch
                    self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path)
                    rospy.loginfo(f"成功加载YOLO模型: {self.model_path}")
                else:
                    # 如果没有自定义模型，使用预训练模型
                    import torch
                    self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
                    rospy.loginfo("使用预训练的YOLOv5s模型")
                
                # 设置置信度阈值
                self.model.conf = self.confidence
                self.model_loaded = True
                return
            except Exception as e:
                rospy.logwarn(f"无法使用PyTorch Hub加载YOLO模型: {e}")
            
            # 如果PyTorch Hub失败，尝试使用OpenCV DNN
            try:
                rospy.loginfo("尝试使用OpenCV DNN加载模型...")
                
                # 检查是否有ONNX模型可用
                onnx_path = self.model_path.replace('.pt', '.onnx')
                if os.path.exists(onnx_path):
                    self.model = cv2.dnn.readNetFromONNX(onnx_path)
                    rospy.loginfo(f"使用OpenCV DNN加载ONNX模型: {onnx_path}")
                    self.model_type = "opencv_dnn"
                    self.model_loaded = True
                    return
                else:
                    rospy.logwarn(f"找不到ONNX模型: {onnx_path}")
            except Exception as e:
                rospy.logwarn(f"无法使用OpenCV DNN加载模型: {e}")
            
            # 如果所有方法都失败，使用基本的OpenCV检测器
            rospy.logwarn("使用基本的OpenCV级联检测器作为备用")
            try:
                # 加载人脸检测器作为备用
                cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
                if os.path.exists(cascade_path):
                    self.model = cv2.CascadeClassifier(cascade_path)
                    self.model_type = "opencv_cascade"
                    self.model_loaded = True
                    rospy.loginfo("已加载OpenCV级联检测器作为备用")
                    return
            except Exception as e:
                rospy.logerr(f"无法加载备用检测器: {e}")
            
            rospy.logerr("所有模型加载方法都失败")
            self.model_loaded = False
            
        except Exception as e:
            rospy.logerr(f"加载YOLO模型失败: {e}")
            self.model_loaded = False
    
    def image_callback(self, data):
        if not self.model_loaded:
            rospy.logerr_once("YOLO模型未加载，无法进行检测")
            return
            
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 创建检测消息
            detection_array = DetectionArray()
            detection_array.header = data.header
            
            # 在图像上绘制检测结果
            visualization_image = cv_image.copy()
            
            # 检查是否启用了检测
            if self.detection_enabled:
                # 根据加载的模型类型执行检测
                if hasattr(self, 'model_type'):
                    if self.model_type == "opencv_dnn":
                        self.detect_with_opencv_dnn(cv_image, detection_array, visualization_image)
                    elif self.model_type == "opencv_cascade":
                        self.detect_with_opencv_cascade(cv_image, detection_array, visualization_image)
                else:
                    # 默认使用PyTorch模型
                    self.detect_with_pytorch(cv_image, detection_array, visualization_image)
                
                # 发布检测结果
                self.detection_pub.publish(detection_array)
                
                if len(detection_array.detections) > 0:
                    rospy.loginfo(f"检测到 {len(detection_array.detections)} 个目标")
                
                # 在图像上添加状态信息
                status_text = f"YOLO检测: 启用 | 检测到: {len(detection_array.detections)}"
                cv2.putText(visualization_image, status_text, (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                # 检测已禁用，添加状态信息
                status_text = "YOLO检测: 禁用"
                cv2.putText(visualization_image, status_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # 发布可视化图像
            visualization_msg = self.bridge.cv2_to_imgmsg(visualization_image, "bgr8")
            visualization_msg.header = data.header
            self.visualization_pub.publish(visualization_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge错误: {e}")
        except Exception as e:
            rospy.logerr(f"处理图像时出错: {e}")
    
    def detect_with_pytorch(self, image, detection_array, visualization_image):
        """使用PyTorch YOLOv5模型进行检测"""
        try:
            # 使用YOLO进行检测
            results = self.model(image)
            
            # 处理检测结果
            detections = results.pandas().xyxy[0]  # 获取边界框坐标
            
            for i, detection in detections.iterrows():
                # 提取检测数据
                x_min = int(detection['xmin'])
                y_min = int(detection['ymin'])
                x_max = int(detection['xmax'])
                y_max = int(detection['ymax'])
                confidence = float(detection['confidence'])
                class_id = int(detection['class'])
                class_name = str(detection['name'])
                
                # 创建检测消息
                det = Detection()
                det.class_id = class_id
                det.class_name = class_name
                det.confidence = confidence
                det.x_min = x_min
                det.y_min = y_min
                det.x_max = x_max
                det.y_max = y_max
                det.center_x = (x_min + x_max) / 2
                det.center_y = (y_min + y_max) / 2
                det.width = x_max - x_min
                det.height = y_max - y_min
                
                detection_array.detections.append(det)
                
                # 在可视化图像上绘制边界框
                cv2.rectangle(visualization_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                
                # 添加标签
                label = f"{class_name}: {confidence:.2f}"
                cv2.putText(visualization_image, label, (x_min, y_min - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        except Exception as e:
            rospy.logerr(f"PyTorch检测错误: {e}")
    
    def detect_with_opencv_dnn(self, image, detection_array, visualization_image):
        """使用OpenCV DNN进行检测"""
        try:
            height, width = image.shape[:2]
            
            # 准备输入blob
            blob = cv2.dnn.blobFromImage(image, 1/255.0, (640, 640), swapRB=True, crop=False)
            self.model.setInput(blob)
            
            # 运行前向传播
            outputs = self.model.forward()
            
            # 处理检测结果
            for detection in outputs[0]:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                if confidence > self.confidence:
                    # YOLO返回的是中心坐标和尺寸
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    
                    # 计算边界框坐标
                    x_min = int(center_x - w / 2)
                    y_min = int(center_y - h / 2)
                    x_max = int(center_x + w / 2)
                    y_max = int(center_y + h / 2)
                    
                    # 创建检测消息
                    det = Detection()
                    det.class_id = int(class_id)
                    det.class_name = f"class_{class_id}"
                    det.confidence = float(confidence)
                    det.x_min = x_min
                    det.y_min = y_min
                    det.x_max = x_max
                    det.y_max = y_max
                    det.center_x = center_x
                    det.center_y = center_y
                    det.width = w
                    det.height = h
                    
                    detection_array.detections.append(det)
                    
                    # 在可视化图像上绘制边界框
                    cv2.rectangle(visualization_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    
                    # 添加标签
                    label = f"Class {class_id}: {confidence:.2f}"
                    cv2.putText(visualization_image, label, (x_min, y_min - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        except Exception as e:
            rospy.logerr(f"OpenCV DNN检测错误: {e}")
    
    def detect_with_opencv_cascade(self, image, detection_array, visualization_image):
        """使用OpenCV级联分类器进行检测"""
        try:
            # 转换为灰度图像
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 使用级联分类器检测人脸
            faces = self.model.detectMultiScale(gray, 1.1, 4)
            
            # 处理检测结果
            for i, (x, y, w, h) in enumerate(faces):
                # 创建检测消息
                det = Detection()
                det.class_id = 0
                det.class_name = "face"
                det.confidence = 1.0  # 级联分类器没有置信度值
                det.x_min = x
                det.y_min = y
                det.x_max = x + w
                det.y_max = y + h
                det.center_x = x + w/2
                det.center_y = y + h/2
                det.width = w
                det.height = h
                
                detection_array.detections.append(det)
                
                # 在可视化图像上绘制边界框
                cv2.rectangle(visualization_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # 添加标签
                label = "Face"
                cv2.putText(visualization_image, label, (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        except Exception as e:
            rospy.logerr(f"OpenCV级联分类器检测错误: {e}")

if __name__ == '__main__':
    try:
        detector = YoloDetector()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("用户中断，退出")
    except Exception as e:
        rospy.logerr(f"未知错误: {e}") 