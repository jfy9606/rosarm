#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import threading
import time
from std_msgs.msg import Int32  # 添加用于接收视图模式切换的消息类型

# 增加额外导入(只在深度计算时使用)
try:
    import cv2.ximgproc
    HAVE_XIMGPROC = True
    rospy.loginfo("cv2.ximgproc available, depth processing enabled")
except ImportError:
    HAVE_XIMGPROC = False
    rospy.logwarn("cv2.ximgproc not available, advanced depth processing will be disabled")

class StereoCameraNode:
    """Stereo camera node for capturing and publishing stereo images"""
    
    def __init__(self):
        rospy.init_node('stereo_camera_node', anonymous=True)
        
        # Get parameters
        self.camera_index = int(rospy.get_param('~camera_index', 0))
        self.frame_rate = int(rospy.get_param('~frame_rate', 30))
        self.use_depth = rospy.get_param('~use_depth', True)  # 默认启用深度图
        
        # 如果cv2.ximgproc不可用，则禁用深度图
        if not HAVE_XIMGPROC and self.use_depth:
            rospy.logwarn("Depth processing requested but cv2.ximgproc is not available. Install opencv-contrib-python package.")
            rospy.logwarn("Depth map generation will be disabled.")
            self.use_depth = False
        
        # 视图模式: 0=左图, 1=右图, 2=深度图
        self.view_mode = int(rospy.get_param('~view_mode', 0))
        
        # Initialize bridge
        self.bridge = CvBridge()
        
        # Initialize publishers
        self.left_pub = rospy.Publisher('/stereo_camera/left/image_raw', Image, queue_size=1)
        self.right_pub = rospy.Publisher('/stereo_camera/right/image_raw', Image, queue_size=1)
        self.current_view_pub = rospy.Publisher('/stereo_camera/current_view', Image, queue_size=1)
        
        # 只有在启用深度图且ximgproc可用时才创建深度图发布器
        if self.use_depth and HAVE_XIMGPROC:
            self.depth_pub = rospy.Publisher('/stereo_camera/depth/image_raw', Image, queue_size=1)
            self.raw_depth_pub = rospy.Publisher('/stereo_camera/depth/raw', Image, queue_size=1)
        else:
            self.depth_pub = None
            self.raw_depth_pub = None
            
        self.left_info_pub = rospy.Publisher('/stereo_camera/left/camera_info', CameraInfo, queue_size=1)
        self.right_info_pub = rospy.Publisher('/stereo_camera/right/camera_info', CameraInfo, queue_size=1)
        
        # 订阅视图模式切换消息
        self.view_mode_sub = rospy.Subscriber('/stereo_camera/view_mode', Int32, self.view_mode_callback)
        
        # Initialize camera
        self.camera = None
        self.is_running = False
        self.camera_lock = threading.Lock()
        
        # Initialize stereo processor
        self.stereo_processor = None
        self.stereo_processor_right = None
        self.wls_filter = None
        self.init_stereo_processor()
        
        # Try to open the camera
        self.open_camera()
        
        # Start publishing thread
        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.daemon = True
        self.thread.start()
        
        rospy.loginfo("Stereo Camera Node initialized")
        
        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)
    
    def view_mode_callback(self, msg):
        """处理视图模式切换消息"""
        new_mode = msg.data
        # 确保模式在有效范围内
        if new_mode >= 0 and new_mode <= 2:
            if new_mode != self.view_mode:
                rospy.loginfo(f"切换视图模式到: {new_mode}")
                self.view_mode = new_mode
                
                # 如果深度模式不可用，切换到左图模式
                if self.view_mode == 2 and (not self.use_depth or not HAVE_XIMGPROC):
                    rospy.logwarn("深度图不可用，切换到左图模式")
                    self.view_mode = 0
                    
                # 立即更新当前视图
                if hasattr(self, 'left_frame') and hasattr(self, 'right_frame'):
                    if self.left_frame is not None and self.right_frame is not None:
                        current_view = self.get_current_view(self.left_frame, self.right_frame)
                        self.publish_current_view(current_view)
        else:
            rospy.logwarn(f"无效的视图模式: {new_mode}，有效值为 0(左图)、1(右图)、2(深度图)")
            
    def init_stereo_processor(self):
        """Initialize stereo vision processor based on selected method"""
        # 只在启用深度图且ximgproc可用时初始化
        if not self.use_depth or not HAVE_XIMGPROC:
            rospy.loginfo("Depth processing is disabled")
            return
            
        try:
            rospy.loginfo("Initializing stereo processors for depth calculation")
            
            # 创建视差计算器 - SGBM算法
            self.stereo_processor = cv2.StereoSGBM.create(
                minDisparity=0,
                numDisparities=128,  # 必须是16的倍数
                blockSize=5,
                P1=8 * 3 * 5 ** 2,  # 控制视差平滑度的参数
                P2=32 * 3 * 5 ** 2,  # 控制视差平滑度的参数
                disp12MaxDiff=1,
                uniquenessRatio=15,
                speckleWindowSize=100,
                speckleRange=1,
                preFilterCap=63,
                mode=1  # 使用cv2.STEREO_SGBM_MODE_SGBM_3WAY的值
            )
            
            # 用于视差后处理
            self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(
                matcher_left=self.stereo_processor
            )
            self.stereo_processor_right = cv2.StereoSGBM.create(
                minDisparity=-128,
                numDisparities=128, 
                blockSize=5,
                P1=8 * 3 * 5 ** 2,
                P2=32 * 3 * 5 ** 2,
                disp12MaxDiff=1,
                uniquenessRatio=15,
                speckleWindowSize=100,
                speckleRange=1,
                preFilterCap=63,
                mode=1  # 使用cv2.STEREO_SGBM_MODE_SGBM_3WAY的值
            )
            self.wls_filter.setLambda(8000)
            self.wls_filter.setSigmaColor(1.5)
            
            rospy.loginfo("Stereo processors initialized successfully")
            
        except Exception as e:
            rospy.logerr(f"Error initializing stereo processors: {e}")
            self.use_depth = False
            self.stereo_processor = None
            self.stereo_processor_right = None
            self.wls_filter = None
            
    def open_camera(self):
        """Open the camera and set up parameters"""
        try:
            with self.camera_lock:
                # Close existing camera if open
                if self.camera is not None:
                    try:
                        self.camera.release()
                    except Exception as e:
                        rospy.logwarn(f"Warning while releasing old camera: {e}")
                    self.camera = None
                    
                    # 调用GC帮助释放资源
                    try:
                        import gc
                        gc.collect()
                    except:
                        pass
                    
                    # 给系统一点时间完全释放资源
                    import time
                    time.sleep(0.5)
                
                # 尝试使用GStreamer管道打开相机（如果可用）
                try:
                    # 首先尝试直接打开，这对USB摄像头通常效果更好
                    self.camera = cv2.VideoCapture(self.camera_index)
                    
                    # 如果使用GStreamer打开失败，尝试直接构造管道
                    if not self.camera.isOpened() and cv2.getBuildInformation().find("GStreamer") != -1:
                        # 针对双目摄像头构建的GStreamer管道
                        gst_pipeline = (
                            f"v4l2src device=/dev/video{self.camera_index} ! "
                            f"image/jpeg,width=1280,height=480,framerate={self.frame_rate}/1 ! "
                            f"jpegdec ! videoconvert ! appsink"
                        )
                        rospy.loginfo(f"Trying GStreamer pipeline: {gst_pipeline}")
                        self.camera = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
                except Exception as e:
                    rospy.logwarn(f"GStreamer pipeline failed, falling back to default: {e}")
                    # 如果GStreamer失败，回退到标准方法
                    self.camera = cv2.VideoCapture(self.camera_index)
                
                if not self.camera.isOpened():
                    rospy.logerr(f"Failed to open camera at index {self.camera_index}")
                    return False
                
                # 首先设置图像格式为MJPG，这对于高帧率（30fps）至关重要
                self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                
                # 设置分辨率
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # Width for stereo setup
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Height for stereo setup
                
                # 设置帧率 - 必须在设置完格式和分辨率后设置
                self.camera.set(cv2.CAP_PROP_FPS, self.frame_rate)
                
                # 读取一帧确保摄像头正常工作
                ret, _ = self.camera.read()
                if not ret:
                    rospy.logwarn("First frame read failed, camera may not be initialized properly")
                    # 再尝试一次
                    ret, _ = self.camera.read()
                    if not ret:
                        rospy.logerr("Camera initialization failed, could not read frames")
                        self.camera.release()
                        self.camera = None
                        return False
                
                # 获取实际相机属性
                actual_width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
                actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
                actual_fourcc = self.camera.get(cv2.CAP_PROP_FOURCC)
                
                # 将fourcc转换为可读格式
                fourcc_chars = chr(int(actual_fourcc) & 0xFF) + chr((int(actual_fourcc) >> 8) & 0xFF) + \
                               chr((int(actual_fourcc) >> 16) & 0xFF) + chr((int(actual_fourcc) >> 24) & 0xFF)
                
                rospy.loginfo(f"Camera opened with resolution: {int(actual_width)}x{int(actual_height)} @ {int(actual_fps)}fps, format: {fourcc_chars}")
                
                # 验证是否成功设置了MJPG格式和30fps
                if actual_fps < self.frame_rate - 1:  # 允许1fps的误差
                    rospy.logwarn(f"Failed to set {self.frame_rate}fps, actual fps: {int(actual_fps)}. Check if camera supports MJPG at this resolution.")
                
                # Set running flag
                self.is_running = True
                return True
                
        except Exception as e:
            rospy.logerr(f"Error opening camera: {e}")
            self.is_running = False
            return False
    
    def get_current_view(self, left_image, right_image):
        """基于当前视图模式返回相应的图像"""
        if self.view_mode == 0:
            # 左图模式
            return left_image
        elif self.view_mode == 1:
            # 右图模式
            return right_image
        elif self.view_mode == 2:
            # 深度图模式
            if self.use_depth and HAVE_XIMGPROC:
                depth_map = self.compute_depth_map(left_image, right_image)
                if depth_map is not None:
                    # 创建彩色深度图用于显示
                    return self.create_colored_depth_map(depth_map)
                else:
                    # 如果深度图计算失败，返回左图
                    rospy.logwarn("深度图计算失败，返回左图")
                    return left_image
            else:
                # 如果深度图不可用，返回左图
                return left_image
        
        # 默认返回左图
        return left_image
    
    def compute_depth_map(self, left_image, right_image):
        """计算深度图"""
        if not self.use_depth or not HAVE_XIMGPROC or self.stereo_processor is None:
            return None
            
        try:
            # 转换为灰度图
            left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
            
            # 计算左视差图
            left_disp = self.stereo_processor.compute(left_gray, right_gray)
            
            # 计算右视差图（用于WLS滤波）
            if self.stereo_processor_right:
                right_disp = self.stereo_processor_right.compute(right_gray, left_gray)
                
                # 使用WLS滤波器进行优化
                if self.wls_filter:
                    filtered_disp = self.wls_filter.filter(left_disp, left_gray, disparity_map_right=right_disp)
                    # 处理无效区域
                    _, filtered_disp = cv2.threshold(filtered_disp, 0, np.inf, cv2.THRESH_TOZERO)
                    return filtered_disp
                    
            # 如果没有进行WLS滤波，则返回原始视差图
            # 注意：视差图是16位有符号整数，需要转换以便可视化
            return left_disp
            
        except Exception as e:
            rospy.logerr(f"深度图计算错误: {e}")
            return None
            
    def create_colored_depth_map(self, depth_map):
        """将深度图转换为彩色可视化图像"""
        try:
            # 归一化深度图
            if depth_map.dtype != np.float32:
                depth_map = depth_map.astype(np.float32)
                
            # 处理无效区域
            valid_mask = depth_map > 0
            if valid_mask.sum() > 0:  # 如果有有效数据
                min_val = depth_map[valid_mask].min()
                max_val = depth_map[valid_mask].max()
                
                if max_val > min_val:
                    # 归一化到0-1范围
                    depth_norm = np.zeros_like(depth_map)
                    depth_norm[valid_mask] = (depth_map[valid_mask] - min_val) / (max_val - min_val)
                    
                    # 转换为彩色图像（jet色彩映射）
                    depth_colored = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
                    
                    # 将无效区域设置为黑色
                    depth_colored[~valid_mask] = 0
                    
                    return depth_colored
            
            # 如果没有有效数据，返回全黑图像
            return np.zeros((depth_map.shape[0], depth_map.shape[1], 3), dtype=np.uint8)
            
        except Exception as e:
            rospy.logerr(f"彩色深度图创建错误: {e}")
            return np.zeros((depth_map.shape[0], depth_map.shape[1], 3), dtype=np.uint8)
            
    def publish_depth_images(self, depth_map, left_image):
        """发布原始深度图和彩色深度图"""
        if depth_map is None or self.depth_pub is None or self.raw_depth_pub is None:
            return
            
        try:
            # 检查深度图的类型
            if len(depth_map.shape) == 2:  # 单通道深度图 (32FC1)
                # 将浮点深度图转换为彩色可视化图
                # 归一化深度图
                colored_depth = self.create_colored_depth_map(depth_map)
                
                # 创建彩色深度图
                colored_depth_msg = self.bridge.cv2_to_imgmsg(colored_depth, encoding="bgr8")
                colored_depth_msg.header.stamp = rospy.Time.now()
                
                # 发布彩色深度图
                self.depth_pub.publish(colored_depth_msg)
                
                # 同时发布原始深度图（保留精确的深度信息）
                raw_depth_msg = self.bridge.cv2_to_imgmsg(depth_map.astype(np.float32), encoding="32FC1")
                raw_depth_msg.header.stamp = rospy.Time.now()
                self.raw_depth_pub.publish(raw_depth_msg)
                
            else:  # 彩色深度图 (已经是BGR格式)
                # 发布深度图
                depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding="bgr8")
                depth_msg.header.stamp = rospy.Time.now()
                self.depth_pub.publish(depth_msg)
                
        except CvBridgeError as e:
            rospy.logerr(f"深度图像转换错误: {e}")
        except Exception as e:
            rospy.logerr(f"深度图像转换错误: {e}")
            
    def publish_current_view(self, image):
        """发布当前视图"""
        if image is None:
            return
            
        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            img_msg.header.stamp = rospy.Time.now()
            self.current_view_pub.publish(img_msg)
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")
        except Exception as e:
            rospy.logerr(f"图像转换错误: {e}")

    def publish_loop(self):
        """发布图像的主循环"""
        rate = rospy.Rate(self.frame_rate)  # 使用设置的帧率
        
        while not rospy.is_shutdown():
            try:
                with self.camera_lock:
                    if self.camera is None or not self.camera.isOpened():
                        if not self.is_reconnecting:
                            self.is_reconnecting = True
                            rospy.logwarn("Camera connection lost, attempting to reconnect...")
                            self.open_camera()
                        rate.sleep()
                        continue
                        
                    # 读取帧
                    success, frame = self.camera.read()
                    
                    if not success:
                        frame_count_fail = getattr(self, 'frame_count_fail', 0) + 1
                        self.frame_count_fail = frame_count_fail
                        
                        if frame_count_fail > 5:  # 连续5次失败，尝试重新打开摄像头
                            rospy.logwarn("摄像头读取失败，尝试重新连接...")
                            self.camera.release()
                            self.camera = None
                            self.is_reconnecting = True
                            self.open_camera()
                        rate.sleep()
                        continue
                        
                    # 重置失败计数
                    self.frame_count_fail = 0
                    self.is_reconnecting = False
                    
                    # 对于宽幅摄像头，分割图像为左右两部分
                    height, width = frame.shape[:2]
                    
                    # 如果是宽幅图像，认为左右各占一半
                    if width > height * 1.5:  # 宽高比大于1.5认为是双目图像
                        mid = width // 2
                        left_frame = frame[:, :mid]
                        right_frame = frame[:, mid:]
                        
                        # 确保左右图像尺寸一致
                        if left_frame.shape[1] != right_frame.shape[1]:
                            min_width = min(left_frame.shape[1], right_frame.shape[1])
                            left_frame = left_frame[:, :min_width]
                            right_frame = right_frame[:, :min_width]
                            
                        # 缓存帧用于视图切换
                        self.left_frame = left_frame
                        self.right_frame = right_frame
                        
                        # 获取当前视图
                        current_view = self.get_current_view(left_frame, right_frame)
                        
                        # 发布左右图像
                        try:
                            left_msg = self.bridge.cv2_to_imgmsg(left_frame, "bgr8")
                            right_msg = self.bridge.cv2_to_imgmsg(right_frame, "bgr8")
                            
                            left_msg.header.stamp = rospy.Time.now()
                            right_msg.header.stamp = left_msg.header.stamp
                            
                            self.left_pub.publish(left_msg)
                            self.right_pub.publish(right_msg)
                            
                            # 发布当前视图
                            self.publish_current_view(current_view)
                            
                            # 如果启用了深度图并且ximgproc可用，计算并发布深度图
                            if self.use_depth and HAVE_XIMGPROC:
                                depth_map = self.compute_depth_map(left_frame, right_frame)
                                if depth_map is not None:
                                    self.publish_depth_images(depth_map, left_frame)
                            
                            # 发布相机信息
                            camera_info = CameraInfo()
                            camera_info.header = left_msg.header
                            camera_info.width = left_frame.shape[1]
                            camera_info.height = left_frame.shape[0]
                            self.left_info_pub.publish(camera_info)
                            self.right_info_pub.publish(camera_info)
                            
                        except CvBridgeError as e:
                            rospy.logerr(f"图像转换错误: {e}")
                    else:
                        # 单目图像，只发布左图
                        try:
                            # 缓存帧用于视图切换
                            self.left_frame = frame
                            self.right_frame = frame  # 对于单目相机，左右图相同
                            
                            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                            img_msg.header.stamp = rospy.Time.now()
                            
                            self.left_pub.publish(img_msg)
                            self.current_view_pub.publish(img_msg)
                            
                            # 发布相机信息
                            camera_info = CameraInfo()
                            camera_info.header = img_msg.header
                            camera_info.width = frame.shape[1]
                            camera_info.height = frame.shape[0]
                            self.left_info_pub.publish(camera_info)
                        except CvBridgeError as e:
                            rospy.logerr(f"图像转换错误: {e}")
                            
            except Exception as e:
                rospy.logerr(f"相机处理异常: {e}")
                
            rate.sleep()
    
    def shutdown(self):
        """Clean up on shutdown"""
        rospy.loginfo("Shutting down stereo camera node")
        self.is_running = False
        
        # 确保正确释放GStreamer资源
        with self.camera_lock:
            if self.camera is not None:
                # 1. 先停止管道
                if hasattr(self.camera, 'cap'):
                    if hasattr(self.camera.cap, 'pipeline') and self.camera.cap.pipeline:
                        import gi
                        gi.require_version('Gst', '1.0')
                        from gi.repository import Gst
                        if hasattr(self.camera.cap.pipeline, 'set_state'):
                            self.camera.cap.pipeline.set_state(Gst.State.NULL)
                
                # 2. 释放摄像头资源
                try:
                    self.camera.release()
                except Exception as e:
                    rospy.logerr(f"Error releasing camera: {e}")
                
                # 3. 确保设置为None
                self.camera = None
                
        # 等待一小段时间确保资源彻底释放
        time.sleep(0.5)
        
        # 显式调用GC
        try:
            import gc
            gc.collect()
        except:
            pass
        
        rospy.loginfo("Camera resources released successfully")

if __name__ == '__main__':
    try:
        node = StereoCameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 