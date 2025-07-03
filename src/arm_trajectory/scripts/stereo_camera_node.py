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
        """根据当前视图模式返回相应的图像"""
        if self.view_mode == 0:
            # 左图模式
            return left_image
        elif self.view_mode == 1:
            # 右图模式
            return right_image
        elif self.view_mode == 2 and self.use_depth and HAVE_XIMGPROC:
            # 深度图模式
            depth_map = self.compute_depth_map(left_image, right_image)
            if depth_map is not None:
                return depth_map
            else:
                # 如果深度图计算失败，返回左图
                rospy.logwarn("深度图计算失败，返回左图")
                return left_image
        else:
            # 默认返回左图
            return left_image
    
    def compute_depth_map(self, left_image, right_image):
        """Compute depth map from stereo images using semi-global matching"""
        # 检查是否可以进行深度计算
        if not self.use_depth or not HAVE_XIMGPROC or self.stereo_processor is None:
            return None
            
        try:
            # 确保图像是有效的
            if left_image is None or right_image is None:
                rospy.logwarn("Invalid images for depth computation")
                return None
                
            if left_image.size == 0 or right_image.size == 0:
                rospy.logwarn("Empty images for depth computation")
                return None
                
            # 确保图像是灰度格式
            if len(left_image.shape) == 3:
                left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            else:
                left_gray = left_image
                
            if len(right_image.shape) == 3:
                right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
            else:
                right_gray = right_image
            
            # 检查灰度图是否有效
            if left_gray.size == 0 or right_gray.size == 0:
                rospy.logwarn("Empty grayscale images")
                return None
                
            # 检查灰度图尺寸是否匹配
            if left_gray.shape != right_gray.shape:
                rospy.logwarn(f"Image size mismatch: left={left_gray.shape}, right={right_gray.shape}")
                return None
            
            # 计算左右视差图
            left_disp = self.stereo_processor.compute(left_gray, right_gray).astype(np.float32) / 16.0
            right_disp = self.stereo_processor_right.compute(right_gray, left_gray).astype(np.float32) / 16.0
            
            # 使用WLS滤波器进行后处理优化
            filtered_disp = self.wls_filter.filter(left_disp, left_gray, disparity_map_right=right_disp)
            
            # 视差转深度（基于相机参数）
            # baseline(基线) * focal_length(焦距) / disparity(视差)
            # 假设基线和焦距
            baseline = 0.06  # 6cm，根据实际双目相机参数调整
            focal_length = 500.0  # 像素单位，根据实际相机内参调整
            
            # 避免除以零
            mask = filtered_disp > 0
            depth_map = np.zeros_like(filtered_disp)
            depth_map[mask] = (baseline * focal_length) / filtered_disp[mask]
            
            # 限制深度范围在合理值内
            depth_map = np.clip(depth_map, 0.1, 5.0)
            
            # 根据视图模式返回不同格式的深度图
            if self.view_mode == 2:  # 当用于视图显示时
                # 生成彩色深度图用于显示
                # 归一化视差图
                norm_disp = cv2.normalize(filtered_disp, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                # 生成彩色深度图
                depth_colormap = cv2.applyColorMap(norm_disp, cv2.COLORMAP_JET)
                return depth_colormap
            else:
                # 返回原始深度图（用于保存精确的深度值）
                return depth_map
            
        except Exception as e:
            rospy.logerr(f"Error in depth map computation: {e}")
            return None
    
    def publish_loop(self):
        """Main loop for capturing and publishing images"""
        rate = rospy.Rate(self.frame_rate)
        
        camera_info_left = CameraInfo()
        camera_info_right = CameraInfo()
        
        # Set up basic camera info
        camera_info_left.header.frame_id = "stereo_left"
        camera_info_right.header.frame_id = "stereo_right"
        camera_info_left.height = 480
        camera_info_left.width = 640
        camera_info_right.height = 480
        camera_info_right.width = 640
        
        # Set up camera matrix (approximate values)
        K = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
        camera_info_left.K = K
        camera_info_right.K = K
        
        # Set distortion model
        camera_info_left.distortion_model = "plumb_bob"
        camera_info_right.distortion_model = "plumb_bob"
        
        # 失败计数和恢复机制
        consecutive_failures = 0
        max_failures = 5
        
        while not rospy.is_shutdown():
            if not self.is_running:
                # Try to reopen camera if it's closed
                if not self.open_camera():
                    rate.sleep()
                    continue
            
            # Capture frame
            with self.camera_lock:
                if self.camera is None:
                    rate.sleep()
                    continue
                    
                # 尝试读取帧，处理可能的JPEG损坏错误
                try:
                    ret, frame = self.camera.read()
                except cv2.error as e:
                    # 忽略JPEG错误，并尝试下一帧
                    rospy.logwarn(f"OpenCV error: {e}")
                    ret = False
                except Exception as e:
                    rospy.logerr(f"Unexpected error reading camera: {e}")
                    ret = False
            
            if not ret:
                consecutive_failures += 1
                rospy.logerr(f"Failed to capture frame ({consecutive_failures}/{max_failures})")
                
                # 如果连续失败太多次，重置相机
                if consecutive_failures >= max_failures:
                    rospy.logwarn("Too many consecutive failures, resetting camera...")
                    self.is_running = False
                    with self.camera_lock:
                        if self.camera is not None:
                            try:
                                self.camera.release()
                            except:
                                pass
                            self.camera = None
                
                rate.sleep()
                continue
            
            # 成功获取帧，重置失败计数
            consecutive_failures = 0
            
            try:
                # 检查帧是否有效
                if frame is None or frame.size == 0:
                    rospy.logwarn("Empty frame received")
                    rate.sleep()
                    continue
                
                # Split frame into left and right images
                height, width = frame.shape[:2]
                half_width = width // 2
                
                if width > 1 and half_width > 0:
                    try:
                        # 确保分割索引在有效范围内
                        if half_width > frame.shape[1]:
                            half_width = frame.shape[1] // 2
                        
                        left_image = frame[:, :half_width].copy()  # 使用copy()避免共享内存引起的问题
                        right_image = frame[:, half_width:].copy()
                        
                        # 验证图像大小和数据完整性
                        if left_image.size == 0 or right_image.size == 0:
                            raise ValueError("Invalid split image size")
                    except Exception as e:
                        rospy.logwarn(f"Error splitting frame: {e}")
                        # 如果分割出错，创建空白图像
                        left_image = np.zeros((480, 640, 3), dtype=np.uint8)
                        right_image = np.zeros((480, 640, 3), dtype=np.uint8)
                else:
                    # If we can't split the image, create dummy images
                    left_image = np.zeros((480, 640, 3), dtype=np.uint8)
                    right_image = np.zeros((480, 640, 3), dtype=np.uint8)
                
                # 获取当前视图模式下的图像
                current_view = self.get_current_view(left_image, right_image)
                
                # Create timestamps
                now = rospy.Time.now()
                
                # Publish left image
                left_msg = self.bridge.cv2_to_imgmsg(left_image, "bgr8")
                left_msg.header.stamp = now
                left_msg.header.frame_id = "stereo_left"
                self.left_pub.publish(left_msg)
                
                # Publish right image
                right_msg = self.bridge.cv2_to_imgmsg(right_image, "bgr8")
                right_msg.header.stamp = now
                right_msg.header.frame_id = "stereo_right"
                self.right_pub.publish(right_msg)
                
                # Publish current view image
                current_view_msg = self.bridge.cv2_to_imgmsg(current_view, "bgr8")
                current_view_msg.header.stamp = now
                current_view_msg.header.frame_id = "stereo_camera"
                self.current_view_pub.publish(current_view_msg)
                
                # Publish depth map if enabled
                if self.use_depth and self.depth_pub is not None:
                    depth_map = self.compute_depth_map(left_image, right_image)
                    if depth_map is not None:
                        try:
                            # 检查深度图的类型
                            if len(depth_map.shape) == 2:  # 单通道深度图 (32FC1)
                                # 将浮点深度图转换为彩色可视化图
                                # 归一化深度图
                                min_val, max_val = 0.1, 5.0  # 假设深度范围
                                depth_normalized = np.clip(depth_map, min_val, max_val)
                                depth_normalized = ((depth_normalized - min_val) / (max_val - min_val) * 255).astype(np.uint8)
                                # 创建彩色深度图
                                depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                                # 发布彩色深度图
                                depth_msg = self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8")
                                
                                # 同时发布原始深度图（保留精确的深度信息）
                                try:
                                    raw_depth_msg = self.bridge.cv2_to_imgmsg(depth_map, encoding="32FC1")
                                    raw_depth_msg.header.stamp = now
                                    raw_depth_msg.header.frame_id = "stereo_depth_raw"
                                    if hasattr(self, 'raw_depth_pub') and self.raw_depth_pub is not None:
                                        self.raw_depth_pub.publish(raw_depth_msg)
                                except cv_bridge.CvBridgeError as e:
                                    rospy.logerr(f"原始深度图转换错误: {e}")
                            else:  # 彩色深度图 (已经是BGR格式)
                                depth_msg = self.bridge.cv2_to_imgmsg(depth_map, "bgr8")
                            
                            # 发布深度图
                            depth_msg.header.stamp = now
                            depth_msg.header.frame_id = "stereo_depth"
                            self.depth_pub.publish(depth_msg)
                            
                        except cv_bridge.CvBridgeError as e:
                            rospy.logerr(f"深度图像转换错误: {e}")
                        except Exception as e:
                            rospy.logerr(f"深度图像转换错误: {e}")
                
                # Publish camera info
                camera_info_left.header.stamp = now
                camera_info_right.header.stamp = now
                self.left_info_pub.publish(camera_info_left)
                self.right_info_pub.publish(camera_info_right)
                
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge error: {e}")
            except Exception as e:
                rospy.logerr(f"Error in publish loop: {e}")
            
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