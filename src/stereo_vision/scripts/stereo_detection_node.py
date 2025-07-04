#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import time
import os
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
import tf2_ros
import geometry_msgs.msg
from camera_config import (left_camera_matrix, left_distortion,
                          right_camera_matrix, right_distortion,
                          R, T, size,
                          R1, R2, P1, P2, Q,
                          left_map1, left_map2, right_map1, right_map2)
import warnings

# 尝试导入ximgproc，用于高级视差滤波
HAVE_XIMGPROC = False
try:
    import cv2.ximgproc
    HAVE_XIMGPROC = True
    rospy.loginfo("Successfully imported cv2.ximgproc")
except ImportError:
    rospy.logwarn("cv2.ximgproc not available, advanced disparity filtering disabled")
    HAVE_XIMGPROC = False

# 尝试导入ObjectDetection消息
try:
    from stereo_vision.msg import ObjectDetection
except ImportError:
    rospy.logerr("Failed to import ObjectDetection message")
    # 创建一个占位符类，防止代码崩溃
    class ObjectDetection:
        def __init__(self):
            self.header = Header()
            self.class_name = ""
            self.confidence = 0.0
            self.x_min = 0.0
            self.y_min = 0.0
            self.x_max = 0.0
            self.y_max = 0.0
            self.x_3d = 0.0
            self.y_3d = 0.0
            self.z_3d = 0.0
            self.distance = 0.0

class StereoDetectionNode:
    def __init__(self):
        rospy.init_node('stereo_detection_node', anonymous=True)
        
        # Load YOLOv8 model
        model_path = rospy.get_param('~yolo_model_path', 'yolo11n.pt')
        auto_download = rospy.get_param('~auto_download', True)
        
        try:
            # 检查模型文件是否存在
            if os.path.exists(model_path):
                rospy.loginfo(f"正在加载本地YOLO模型: {model_path}")
                self.model = YOLO(model_path)
                rospy.loginfo(f"Successfully loaded YOLO model: {model_path}")
            elif auto_download:
                # 如果模型不存在但启用了自动下载，尝试自动下载
                rospy.loginfo(f"模型文件不存在: {model_path}，尝试自动下载")
                try:
                    self.model = YOLO(model_path)  # 自动下载模型
                    rospy.loginfo(f"成功下载并加载YOLO模型: {model_path}")
                except Exception as e:
                    rospy.logerr(f"自动下载YOLO模型失败: {str(e)}，尝试使用默认模型yolo11n.pt")
                    try:
                        self.model = YOLO('yolo11n.pt')  # 尝试使用默认模型
                        rospy.loginfo("成功加载默认YOLO模型: yolo11n.pt")
                    except Exception as e2:
                        rospy.logerr(f"加载默认YOLO模型失败: {str(e2)}")
                        self.model = None
            else:
                # 如果模型不存在且未启用自动下载，尝试使用默认模型
                rospy.logwarn(f"模型文件不存在: {model_path}，尝试使用默认模型yolo11n.pt")
                try:
                    self.model = YOLO('yolo11n.pt')
                    rospy.loginfo("Successfully loaded default YOLO model")
                except Exception as e:
                    rospy.logerr(f"Failed to load default YOLO model: {str(e)}")
                    self.model = None
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model: {str(e)}")
            rospy.loginfo("Trying default model yolo11n.pt")
            try:
                self.model = YOLO('yolo11n.pt')
                rospy.loginfo("Successfully loaded default YOLO model")
            except Exception as e:
                rospy.logerr(f"Failed to load default YOLO model: {str(e)}")
                self.model = None
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # SGBM parameters for stereo matching - 用于测量距离
        blockSize = 8
        img_channels = 3
        
        # 使用正确的创建方法
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=1,
            numDisparities=64,
            blockSize=blockSize,
            P1=8 * img_channels * blockSize * blockSize,
            P2=32 * img_channels * blockSize * blockSize,
            disp12MaxDiff=-1,
            preFilterCap=140,
            uniquenessRatio=1,
            speckleWindowSize=100,
            speckleRange=100,
            mode=cv2.STEREO_SGBM_MODE_HH)
        
        # 如果ximgproc可用，创建WLS滤波器用于优化距离测量
        self.right_matcher = None
        self.wls_filter = None
        if HAVE_XIMGPROC:
            self.right_matcher = cv2.StereoSGBM_create(
                minDisparity=1,
                numDisparities=64,
                blockSize=blockSize,
                P1=8 * img_channels * blockSize * blockSize,
                P2=32 * img_channels * blockSize * blockSize,
                disp12MaxDiff=-1,
                preFilterCap=140,
                uniquenessRatio=1,
                speckleWindowSize=100,
                speckleRange=100,
                mode=cv2.STEREO_SGBM_MODE_HH)
            
            self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.stereo)
            self.wls_filter.setLambda(8000.0)
            self.wls_filter.setSigmaColor(1.5)
            rospy.loginfo("WLS filter created for enhanced distance measurement")
        
        # Initialize transform broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Publishers
        self.detection_pub = rospy.Publisher('stereo_detections', ObjectDetection, queue_size=10)
        self.result_image_pub = rospy.Publisher('detection_image', Image, queue_size=10)
        self.pose_array_pub = rospy.Publisher('detected_poses', geometry_msgs.msg.PoseArray, queue_size=10)
        self.left_image_pub = rospy.Publisher('left_image', Image, queue_size=10)
        self.right_image_pub = rospy.Publisher('right_image', Image, queue_size=10)
        
        # GUI compatible publishers
        self.gui_detection_pub = rospy.Publisher('/detections/image', Image, queue_size=10)
        self.gui_poses_pub = rospy.Publisher('/detections/poses', geometry_msgs.msg.PoseArray, queue_size=10)
        
        # Set camera parameters
        self.camera_info = CameraInfo()
        self.camera_info.K = left_camera_matrix.flatten().tolist()
        self.camera_info.D = left_distortion.tolist()
        self.camera_info.R = np.eye(3).flatten().tolist()
        self.camera_info.P = P1.flatten().tolist()
        self.camera_info.width = size[0]
        self.camera_info.height = size[1]
        
        # Current view mode (0=left, 1=right)
        self.view_mode = 0
        
        # Subscribe to stereo camera input
        if rospy.get_param('~use_split_feed', True):
            # Using two separate camera feeds
            self.left_sub = rospy.Subscriber('left_camera/image_raw', Image, self.left_callback, queue_size=1)
            self.right_sub = rospy.Subscriber('right_camera/image_raw', Image, self.right_callback, queue_size=1)
            self.left_image = None
            self.right_image = None
        else:
            # Using one combined feed (side-by-side images)
            self.camera_sub = rospy.Subscriber('stereo_camera/image_raw', Image, self.camera_callback, queue_size=1)
        
        # Subscribe to view mode changes
        self.view_mode_sub = rospy.Subscriber('/stereo_vision/view_mode', Header, self.view_mode_callback, queue_size=1)
        
        rospy.loginfo("Stereo detection node initialized")
    
    def view_mode_callback(self, msg):
        """Handle view mode changes"""
        if hasattr(msg, 'frame_id') and msg.frame_id:
            try:
                mode = int(msg.frame_id)
                if 0 <= mode <= 1:  # 只有左右两种视图
                    # 只在模式实际变化时记录日志
                    if self.view_mode != mode:
                        old_mode = self.view_mode
                        self.view_mode = mode
                        mode_names = {0: "左视图", 1: "右视图"}
                        rospy.loginfo(f"视图模式切换: {mode_names.get(old_mode, '未知')} -> {mode_names.get(mode, '未知')}")
                        
                        # 强制刷新视图
                        if self.left_image is not None and self.right_image is not None:
                            self.process_stereo_pair(self.left_image, self.right_image)
                else:
                    rospy.logwarn(f"收到无效的视图模式: {mode}，应为0(左)或1(右)")
            except ValueError:
                rospy.logwarn(f"无法解析视图模式: {msg.frame_id}")
    
    def left_callback(self, msg):
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")  # 忽略警告
                self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                # 检查图像是否有效
                if self.left_image is None or self.left_image.size == 0:
                    rospy.logwarn("收到无效的左相机图像")
                    return
                    
                if self.right_image is not None:
                    self.process_stereo_pair(self.left_image, self.right_image)
        except Exception as e:
            rospy.logerr(f"处理左相机图像时出错: {str(e)}")
    
    def right_callback(self, msg):
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")  # 忽略警告
                self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                # 检查图像是否有效
                if self.right_image is None or self.right_image.size == 0:
                    rospy.logwarn("收到无效的右相机图像")
                    return
                    
                if self.left_image is not None:
                    self.process_stereo_pair(self.left_image, self.right_image)
        except Exception as e:
            rospy.logerr(f"处理右相机图像时出错: {str(e)}")
    
    def camera_callback(self, msg):
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")  # 忽略警告
                # Convert combined image to OpenCV format
                frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                
                # 检查图像是否有效
                if frame is None or frame.size == 0:
                    rospy.logwarn("收到无效的相机图像")
                    return
                
                # Split the image into left and right
                height, width = frame.shape[:2]
                mid = width // 2
                left_image = frame[:, :mid]
                right_image = frame[:, mid:]
                
                # Process the stereo pair
                self.process_stereo_pair(left_image, right_image)
        except Exception as e:
            rospy.logerr(f"处理相机图像时出错: {str(e)}")
    
    def process_stereo_pair(self, left_img, right_img):
        try:
            # 检查图像是否有效
            if left_img is None or left_img.size == 0 or right_img is None or right_img.size == 0:
                rospy.logwarn("处理无效的立体图像对")
                return
            
            # 确保图像尺寸匹配
            if left_img.shape[0] != right_img.shape[0] or left_img.shape[1] != right_img.shape[1]:
                rospy.logwarn(f"左右图像尺寸不匹配: 左{left_img.shape}，右{right_img.shape}")
                # 调整尺寸到相同大小
                min_height = min(left_img.shape[0], right_img.shape[0])
                min_width = min(left_img.shape[1], right_img.shape[1])
                left_img = left_img[:min_height, :min_width]
                right_img = right_img[:min_height, :min_width]
            
            # Publish left and right images
            try:
                left_msg = self.bridge.cv2_to_imgmsg(left_img, "bgr8")
                right_msg = self.bridge.cv2_to_imgmsg(right_img, "bgr8")
                self.left_image_pub.publish(left_msg)
                self.right_image_pub.publish(right_msg)
            except Exception as e:
                rospy.logerr(f"发布原始图像时出错: {str(e)}")
            
            # 计算视差图（用于测量距离，但不显示）
            points_3d = None
            try:
                # Convert to grayscale for stereo matching
                imgL_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
                imgR_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
                
                # Rectify images
                img1_rectified = cv2.remap(imgL_gray, left_map1, left_map2, cv2.INTER_LINEAR)
                img2_rectified = cv2.remap(imgR_gray, right_map1, right_map2, cv2.INTER_LINEAR)
                
                # Compute disparity
                disparity = self.stereo.compute(img1_rectified, img2_rectified)
                
                # 使用WLS滤波器优化视差图（如果可用）
                disp_filtered = disparity
                if HAVE_XIMGPROC and self.right_matcher and self.wls_filter:
                    try:
                        right_disparity = self.right_matcher.compute(img2_rectified, img1_rectified)
                        # 转换为正确的类型
                        filtered_disp = self.wls_filter.filter(disparity, img1_rectified, disparity_map_right=right_disparity)
                        if filtered_disp is not None:
                            disp_filtered = filtered_disp
                    except Exception as e:
                        rospy.logwarn(f"WLS滤波器处理失败: {str(e)}")
                
                # Reproject to 3D
                points_3d = cv2.reprojectImageTo3D(disp_filtered, Q, handleMissingValues=True) * 16
            except Exception as e:
                rospy.logerr(f"计算深度图时出错: {str(e)}")
                # 创建一个空的3D点云
                points_3d = np.zeros((left_img.shape[0], left_img.shape[1], 3), dtype=np.float32)
            
            # 物体检测和距离测量
            annotated_left = None
            annotated_right = None
            
            # 基于当前模式选择要处理的图像
            detection_img = left_img if self.view_mode == 0 else right_img
            
            if self.model:
                try:
                    # 在当前视图上运行YOLO检测
                    results = self.model(detection_img, imgsz=640, conf=0.5)
                    
                    # 处理左视图的标注
                    annotated_left = left_img.copy()
                    if self.view_mode == 0:  # 如果在左视图模式下，使用plot好的图像
                        annotated_left = results[0].plot()
                    
                    # 处理右视图的标注
                    annotated_right = right_img.copy()
                    
                    # 标注检测到的所有物体
                    for box in results[0].boxes:
                        # 获取框坐标
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                        
                        # 检查中心点是否在有效范围内
                        if (center_y < 0 or center_y >= points_3d.shape[0] or
                            center_x < 0 or center_x >= points_3d.shape[1]):
                            continue
                        
                        # 获取深度和3D位置
                        depth = points_3d[center_y][center_x][2] / 1000.0  # 转换为米
                        
                        if depth > 0:
                            # 计算3D位置
                            x_3d = points_3d[center_y][center_x][0] / 10.0  # 转换为厘米
                            y_3d = points_3d[center_y][center_x][1] / 10.0
                            z_3d = points_3d[center_y][center_x][2] / 10.0
                            distance = np.sqrt(x_3d**2 + y_3d**2 + z_3d**2)
                            
                            # 获取类别信息
                            class_id = int(box.cls[0])
                            class_name = results[0].names[class_id]
                            conf = float(box.conf[0])
                            
                            # 距离文本
                            dist_text = f"{distance:.1f}cm"
                            
                            # 如果是左视图模式且需要添加距离信息
                            if self.view_mode == 0 and not hasattr(results[0], 'added_distance'):
                                # 计算文本位置
                                text_pos_y = max(y1 - 10, 20)  # 确保文本不会超出图像顶部
                                
                                # 绘制距离信息
                                text_size = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                                cv2.rectangle(annotated_left, 
                                            (x1, text_pos_y - text_size[1] - 5), 
                                            (x1 + text_size[0] + 10, text_pos_y + 5), 
                                            (0, 0, 0), -1)
                                cv2.putText(annotated_left, dist_text, (x1 + 5, text_pos_y), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                            
                            # 在右视图中绘制物体框和信息
                            cv2.rectangle(annotated_right, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            
                            # 构建右视图标签文本
                            label = f"{class_name} {conf:.2f} {dist_text}"
                            
                            # 计算文本位置
                            text_pos_y = max(y1 - 10, 20)  # 确保文本不会超出图像顶部
                            
                            # 绘制标签
                            text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                            cv2.rectangle(annotated_right, 
                                        (x1, text_pos_y - text_size[1] - 5), 
                                        (x1 + text_size[0] + 10, text_pos_y + 5), 
                                        (0, 0, 0), -1)
                            cv2.putText(annotated_right, label, (x1 + 5, text_pos_y), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                            
                            # 创建检测消息
                            detect_msg = ObjectDetection()
                            detect_msg.header = Header()
                            detect_msg.header.stamp = rospy.Time.now()
                            detect_msg.header.frame_id = "camera_link"
                            detect_msg.class_name = class_name
                            detect_msg.confidence = conf
                            detect_msg.x_min = float(x1)
                            detect_msg.y_min = float(y1)
                            detect_msg.x_max = float(x2)
                            detect_msg.y_max = float(y2)
                            detect_msg.x_3d = float(x_3d)
                            detect_msg.y_3d = float(y_3d)
                            detect_msg.z_3d = float(z_3d)
                            detect_msg.distance = float(distance)
                            self.detection_pub.publish(detect_msg)
                            
                            # 创建TF帧
                            tf_id = f"object_{class_name}_{class_id}"
                            self.publish_tf(tf_id, x_3d/100.0, y_3d/100.0, z_3d/100.0)
                    
                    # 标记已添加距离信息
                    if self.view_mode == 0:
                        results[0].added_distance = True
                except Exception as e:
                    rospy.logerr(f"YOLO检测或标注时出错: {str(e)}")
                    if annotated_left is None:
                        annotated_left = left_img.copy()
                    if annotated_right is None:
                        annotated_right = right_img.copy()
                    cv2.putText(annotated_left, "YOLO detection error", (10, 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                annotated_left = left_img.copy()
                annotated_right = right_img.copy()
                cv2.putText(annotated_left, "YOLO model not available", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # 根据视图模式选择要显示的图像
            display_image = None
            if self.view_mode == 0:
                # 左图像（带物体检测和距离信息）
                display_image = annotated_left
                cv2.putText(display_image, "Left Camera", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                rospy.logdebug("显示左摄像头视图")
            elif self.view_mode == 1:
                # 右图像（带物体检测和距离信息）
                display_image = annotated_right
                cv2.putText(display_image, "Right Camera", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                rospy.logdebug("显示右摄像头视图")
            else:
                # 默认使用左图
                display_image = annotated_left
                rospy.logwarn(f"未知的视图模式: {self.view_mode}，使用左图")
                
            # 更新显示图像到GUI
            try:
                if display_image is not None:
                    gui_msg = self.bridge.cv2_to_imgmsg(display_image, "bgr8")
                    self.gui_detection_pub.publish(gui_msg)
            except Exception as e:
                rospy.logerr(f"发布显示图像时出错: {str(e)}")
            
            # Create pose array for all detections
            pose_array = geometry_msgs.msg.PoseArray()
            pose_array.header = Header()
            pose_array.header.stamp = rospy.Time.now()
            pose_array.header.frame_id = "camera_link"
            
            # Publish result image and pose array
            try:
                if annotated_left is not None:
                    result_msg = self.bridge.cv2_to_imgmsg(annotated_left, "bgr8")
                    self.result_image_pub.publish(result_msg)
                
                self.pose_array_pub.publish(pose_array)
                self.gui_poses_pub.publish(pose_array)
            except Exception as e:
                rospy.logerr(f"发布结果时出错: {str(e)}")
        except Exception as e:
            rospy.logerr(f"处理立体图像对时出错: {str(e)}")
    
    def publish_tf(self, frame_id, x, y, z):
        """Publish TF frame for an object"""
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "camera_link"
        transform.child_frame_id = frame_id
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    try:
        node = StereoDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 