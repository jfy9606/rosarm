#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import threading

# 增加额外导入(只在深度计算时使用)
try:
    import cv2.ximgproc
    HAVE_XIMGPROC = True
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
        self.stereo_method = rospy.get_param('~stereo_method', 'anaglyph')  # anaglyph, wiggle, side_by_side
        self.use_depth = rospy.get_param('~use_depth', False) and HAVE_XIMGPROC  # 是否生成深度图
        
        # Initialize bridge
        self.bridge = CvBridge()
        
        # Initialize publishers
        self.left_pub = rospy.Publisher('/stereo_camera/left/image_raw', Image, queue_size=1)
        self.right_pub = rospy.Publisher('/stereo_camera/right/image_raw', Image, queue_size=1)
        self.merged_pub = rospy.Publisher('/stereo_camera/image_merged', Image, queue_size=1)
        self.depth_pub = rospy.Publisher('/stereo_camera/depth/image_raw', Image, queue_size=1) if self.use_depth else None
        self.left_info_pub = rospy.Publisher('/stereo_camera/left/camera_info', CameraInfo, queue_size=1)
        self.right_info_pub = rospy.Publisher('/stereo_camera/right/camera_info', CameraInfo, queue_size=1)
        
        # Initialize camera
        self.camera = None
        self.is_running = False
        self.camera_lock = threading.Lock()
        
        # Initialize stereo processor
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
    
    def init_stereo_processor(self):
        """Initialize stereo vision processor based on selected method"""
        self.stereo_processor = None
        self.stereo_processor_right = None
        self.wls_filter = None
        
        if self.use_depth and HAVE_XIMGPROC:
            try:
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
                
            except Exception as e:
                rospy.logerr(f"Error initializing stereo processors: {e}")
                self.use_depth = False
                
        # 其他stereo处理相关的初始化在这里进行
        self.wiggle_count = 0
        self.wiggle_max = 10  # wiggle效果的帧数
        self.wiggle_direction = 1  # 1为正向，-1为反向
            
    def open_camera(self):
        """Open the camera and set up parameters"""
        try:
            with self.camera_lock:
                # Close existing camera if open
                if self.camera is not None:
                    self.camera.release()
                
                # Open camera
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
    
    def create_anaglyph(self, left_image, right_image):
        """Create anaglyph (red-cyan) 3D image from stereo pair"""
        # 确保图像是BGR格式
        if len(left_image.shape) == 2:
            left_image = cv2.cvtColor(left_image, cv2.COLOR_GRAY2BGR)
        if len(right_image.shape) == 2:
            right_image = cv2.cvtColor(right_image, cv2.COLOR_GRAY2BGR)
            
        # 拆分通道
        left_b, left_g, left_r = cv2.split(left_image)
        right_b, right_g, right_r = cv2.split(right_image)
        
        # 创建红青立体图像
        anaglyph_r = right_r  # 右图红色通道
        anaglyph_g = left_g   # 左图绿色通道
        anaglyph_b = left_b   # 左图蓝色通道
        
        # 合并通道
        anaglyph = cv2.merge([anaglyph_b, anaglyph_g, anaglyph_r])
        
        return anaglyph
    
    def create_wiggle(self, left_image, right_image):
        """Create wiggle stereo effect by alternating between left and right images"""
        # 根据当前计数器选择显示左图或右图
        if self.wiggle_count < self.wiggle_max / 2:
            image = left_image
        else:
            image = right_image
            
        # 更新计数器
        self.wiggle_count += self.wiggle_direction
        if self.wiggle_count >= self.wiggle_max or self.wiggle_count <= 0:
            self.wiggle_direction *= -1  # 反转方向
            
        return image
    
    def compute_depth_map(self, left_image, right_image):
        """Compute depth map from stereo images using semi-global matching"""
        if not self.use_depth or not HAVE_XIMGPROC or self.stereo_processor is None:
            # 返回一个简单的彩色图像而不是深度图
            return np.zeros_like(left_image)
            
        try:
            # 确保图像是灰度格式
            if len(left_image.shape) == 3:
                left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
            else:
                left_gray = left_image
                
            if len(right_image.shape) == 3:
                right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
            else:
                right_gray = right_image
            
            # 计算左右视差图
            left_disp = self.stereo_processor.compute(left_gray, right_gray).astype(np.float32) / 16.0
            right_disp = self.stereo_processor_right.compute(right_gray, left_gray).astype(np.float32) / 16.0
            
            # 使用WLS滤波器进行后处理优化
            filtered_disp = self.wls_filter.filter(left_disp, left_gray, disparity_map_right=right_disp)
            
            # 归一化视差图以便显示
            norm_disp = cv2.normalize(filtered_disp, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # 生成彩色深度图
            depth_colormap = cv2.applyColorMap(norm_disp, cv2.COLORMAP_JET)
            
            return depth_colormap
            
        except Exception as e:
            rospy.logerr(f"Error in depth map computation: {e}")
            return np.zeros_like(left_image)
    
    def create_side_by_side_3d(self, left_image, right_image):
        """Create side-by-side 3D image with highlighting"""
        # 添加边框以突出立体效果
        border_color = (0, 255, 0)  # 绿色边框
        border_size = 2
        
        # 添加边框
        left_border = cv2.copyMakeBorder(left_image, border_size, border_size, border_size, border_size, 
                                         cv2.BORDER_CONSTANT, value=border_color)
        right_border = cv2.copyMakeBorder(right_image, border_size, border_size, border_size, border_size, 
                                          cv2.BORDER_CONSTANT, value=border_color)
        
        # 水平拼接
        side_by_side = np.hstack([left_border, right_border])
        
        # 在中间画一条绿线
        h, w = side_by_side.shape[:2]
        mid_x = w // 2
        cv2.line(side_by_side, (mid_x, 0), (mid_x, h), (0, 255, 0), 2)
        
        return side_by_side
    
    def process_stereo_images(self, left_image, right_image):
        """Process stereo images based on selected method"""
        if self.stereo_method == 'anaglyph':
            # 生成红青立体图像
            return self.create_anaglyph(left_image, right_image)
        elif self.stereo_method == 'wiggle':
            # 生成摇摆效果
            return self.create_wiggle(left_image, right_image)
        elif self.stereo_method == 'side_by_side':
            # 生成并排显示立体图像
            return self.create_side_by_side_3d(left_image, right_image)
        else:
            # 默认情况下，简单地显示两个图像
            return np.hstack([left_image, right_image])
    
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
                    
                ret, frame = self.camera.read()
            
            if not ret:
                rospy.logerr("Failed to capture frame")
                self.is_running = False
                rate.sleep()
                continue
            
            try:
                # Split frame into left and right images
                height, width = frame.shape[:2]
                half_width = width // 2
                
                if width > 1:
                    left_image = frame[:, :half_width]
                    right_image = frame[:, half_width:]
                else:
                    # If we can't split the image, create dummy images
                    left_image = np.zeros((480, 640, 3), dtype=np.uint8)
                    right_image = np.zeros((480, 640, 3), dtype=np.uint8)
                
                # Process stereo images to create merged view
                merged_image = self.process_stereo_images(left_image, right_image)
                
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
                
                # Publish merged image
                merged_msg = self.bridge.cv2_to_imgmsg(merged_image, "bgr8")
                merged_msg.header.stamp = now
                merged_msg.header.frame_id = "stereo_camera"
                self.merged_pub.publish(merged_msg)
                
                # Compute and publish depth map if enabled
                if self.use_depth and self.depth_pub is not None:
                    depth_map = self.compute_depth_map(left_image, right_image)
                    depth_msg = self.bridge.cv2_to_imgmsg(depth_map, "bgr8")
                    depth_msg.header.stamp = now
                    depth_msg.header.frame_id = "stereo_depth"
                    self.depth_pub.publish(depth_msg)
                
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
        
        with self.camera_lock:
            if self.camera is not None:
                self.camera.release()
                self.camera = None

if __name__ == '__main__':
    try:
        node = StereoCameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 