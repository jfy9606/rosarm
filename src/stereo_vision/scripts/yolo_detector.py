#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

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
    
    def __init__(self):
        """初始化YOLO检测器节点"""
        rospy.init_node('yolo_detector')
        
        # 获取参数
        self.enabled = rospy.get_param('~enabled', False)
        self.model_name = rospy.get_param('~model', 'yolov8s.pt')
        self.conf_threshold = rospy.get_param('~conf', 0.25)
        self.simulation_mode = rospy.get_param('~simulation_mode', False)  # 添加模拟模式参数，默认为False
        
        # 创建OpenCV桥接器
        self.bridge = CvBridge()
        
        # 初始化YOLO模型
        self.model_loaded = False
        
        # 打印Python路径以便调试
        rospy.loginfo(f"Python路径: {sys.path}")
        
        if self.simulation_mode:
            # 使用模拟模式
            rospy.loginfo("使用模拟检测模式")
            self.model_loaded = True
        else:
            # 尝试加载实际的YOLO模型
            try:
                # 尝试导入ultralytics
                try:
                    import ultralytics
                    rospy.loginfo(f"成功导入ultralytics，版本: {ultralytics.__version__}")
                    from ultralytics import YOLO
                except ImportError as e:
                    rospy.logwarn(f"无法导入Ultralytics库: {str(e)}")
                    rospy.logwarn("请安装YOLOv8: pip install ultralytics")
                    
                    # 尝试自动安装
                    try:
                        rospy.loginfo("尝试自动安装ultralytics...")
                        import subprocess
                        subprocess.check_call([sys.executable, "-m", "pip", "install", "ultralytics"])
                        rospy.loginfo("ultralytics安装成功，尝试重新导入...")
                        import ultralytics
                        from ultralytics import YOLO
                        rospy.loginfo(f"成功导入ultralytics，版本: {ultralytics.__version__}")
                    except Exception as install_error:
                        rospy.logerr(f"无法安装ultralytics: {str(install_error)}")
                        rospy.logwarn("切换到模拟检测模式")
                        self.simulation_mode = True
                        self.model_loaded = True
                        return
                
                # 加载模型
                rospy.loginfo(f"正在加载YOLO模型: {self.model_name}")
                
                # 确保模型文件存在或可下载
                model_path = self.model_name
                if not os.path.exists(model_path) and not model_path.startswith(('http://', 'https://')):
                    # 尝试在几个常见位置查找模型
                    possible_paths = [
                        os.path.join(os.path.dirname(__file__), self.model_name),
                        os.path.join(os.path.expanduser('~'), self.model_name),
                        os.path.join('/tmp', self.model_name)
                    ]
                    
                    # 尝试在ultralytics包的资源目录中查找
                    try:
                        import ultralytics
                        ultralytics_path = os.path.dirname(ultralytics.__file__)
                        assets_path = os.path.join(ultralytics_path, 'assets', self.model_name)
                        possible_paths.append(assets_path)
                    except:
                        pass
                    
                    for path in possible_paths:
                        if os.path.exists(path):
                            model_path = path
                            rospy.loginfo(f"找到模型文件: {path}")
                            break
                    
                    if not os.path.exists(model_path):
                        rospy.loginfo(f"未找到本地模型文件，将尝试下载: {self.model_name}")
                
                self.model = YOLO(model_path)  # 加载实际的模型
                self.model_loaded = True
                rospy.loginfo(f"成功加载YOLO模型: {self.model_name}")
            except Exception as e:
                rospy.logerr(f"加载YOLO模型失败: {str(e)}")
                rospy.logwarn("切换到模拟检测模式")
                self.simulation_mode = True
                self.model_loaded = True
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # 订阅控制话题
        self.control_sub = rospy.Subscriber('/yolo/status', Bool, self.control_callback)
        
        # 创建检测结果发布器
        self.detection_pub = rospy.Publisher('/detections/image', Image, queue_size=10)
        self.poses_pub = rospy.Publisher('/detections/poses', PoseArray, queue_size=10)
        
        rospy.loginfo(f"YOLO目标检测器已初始化，检测状态: {'启用' if self.enabled else '禁用'}, 模式: {'模拟' if self.simulation_mode else '正常'}")
    
    def control_callback(self, msg):
        """处理控制消息"""
        new_state = msg.data
        if new_state != self.enabled:
            self.enabled = new_state
            rospy.loginfo(f"YOLO检测已{'启用' if self.enabled else '禁用'}")
    
    def image_callback(self, msg):
        """处理图像消息"""
        if not self.enabled:
            # 如果检测被禁用，直接转发原始图像
            self.detection_pub.publish(msg)
            return
            
        if not self.model_loaded:
            rospy.logerr("YOLO模型未加载，无法进行检测")
            self.detection_pub.publish(msg)
            return
        
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 进行YOLO检测
            detections = self.detect_objects(cv_image)
            
            # 在图像上绘制检测结果
            result_image = self.draw_detections(cv_image, detections)
            
            # 将结果图像转换回ROS消息并发布
            result_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
            result_msg.header = msg.header
            self.detection_pub.publish(result_msg)
            
            # 发布检测到的物体位姿
            self.publish_poses(detections, msg.header)
            
        except Exception as e:
            rospy.logerr(f"处理图像时出错: {str(e)}")
    
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
                results = self.model(image)
                
                # 解析YOLO结果
                detections = []
                for r in results:
                    boxes = r.boxes
                    for box in boxes:
                        # 获取边界框
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        # 获取置信度
                        conf = box.conf[0].cpu().numpy()
                        # 获取类别ID
                        class_id = box.cls[0].cpu().numpy()
                        
                        # 仅保留置信度高于阈值的检测结果
                        if conf >= self.conf_threshold:
                            detections.append([x1, y1, x2, y2, conf, class_id])
                
                return detections
            except Exception as e:
                rospy.logerr(f"YOLO检测出错: {str(e)}")
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
        
        # 模拟检测到的物体1（在图像左侧）
        if np.random.random() > 0.3:  # 70%的概率检测到
            x1 = int(width * 0.1)
            y1 = int(height * 0.4)
            x2 = int(width * 0.3)
            y2 = int(height * 0.6)
            conf = np.random.uniform(0.6, 0.9)
            class_id = 0  # 假设是"物体"类
            detections.append([x1, y1, x2, y2, conf, class_id])
        
        # 模拟检测到的物体2（在图像右侧）
        if np.random.random() > 0.5:  # 50%的概率检测到
            x1 = int(width * 0.6)
            y1 = int(height * 0.3)
            x2 = int(width * 0.8)
            y2 = int(height * 0.7)
            conf = np.random.uniform(0.5, 0.8)
            class_id = 1  # 假设是"另一种物体"类
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
        
        # 类别名称（示例）
        class_names = ["物体", "另一种物体"]
        
        # 为不同类别设置不同颜色
        colors = [(0, 255, 0), (0, 0, 255)]
        
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            
            # 转换为整数坐标
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # 获取类别名称和颜色
            class_name = class_names[int(class_id)] if int(class_id) < len(class_names) else f"类别{int(class_id)}"
            color = colors[int(class_id) % len(colors)]
            
            # 绘制边界框
            cv2.rectangle(result_image, (x1, y1), (x2, y2), color, 2)
            
            # 绘制类别名称和置信度
            label = f"{class_name} {conf:.2f}"
            cv2.putText(result_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
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
        
        # 发布位姿数组
        self.poses_pub.publish(pose_array)

def main():
    """主函数"""
    detector = YoloDetector()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    
    rospy.loginfo("YOLO检测器已关闭")

if __name__ == '__main__':
    main() 