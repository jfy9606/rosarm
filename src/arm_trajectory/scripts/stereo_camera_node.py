#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import threading
import time
from std_msgs.msg import String

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
        self.stereo_method = rospy.get_param('~stereo_method', 'rectified')  # rectified, anaglyph, wiggle, side_by_side
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
        
        # 订阅立体显示方法参数变更
        self.method_sub = rospy.Subscriber('/stereo_camera/stereo_method', String, self.stereo_method_callback)
        
        # Initialize camera
        self.camera = None
        self.is_running = False
        self.camera_lock = threading.Lock()
        
        # 检测系统中可用的摄像头设备
        self.available_cameras = self.find_available_cameras()
        
        # 如果指定的索引无效但有其他可用摄像头，使用第一个可用的
        if self.camera_index not in self.available_cameras and self.available_cameras:
            rospy.logwarn(f"Camera index {self.camera_index} not available. Using index {self.available_cameras[0]} instead.")
            self.camera_index = self.available_cameras[0]
        
        # 初始化立体配准参数
        self.init_stereo_rectification()
        
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
    
    def find_available_cameras(self):
        """查找系统中所有可用的摄像头设备"""
        available_cameras = []
        
        # 1. 首先检查/dev/video*设备文件
        import glob
        import os
        
        # 获取所有video设备文件
        video_devices = glob.glob('/dev/video*')
        rospy.loginfo(f"Found {len(video_devices)} video device files: {video_devices}")
        
        # 检查每个设备是否可以作为摄像头打开
        for device in video_devices:
            try:
                # 从设备路径提取索引号
                index = int(device.replace('/dev/video', ''))
                
                # 尝试使用OpenCV打开设备
                cap = cv2.VideoCapture(index)
                if cap.isOpened():
                    # 读取一帧测试
                    ret, _ = cap.read()
                    if ret:
                        available_cameras.append(index)
                        rospy.loginfo(f"Camera device {device} (index {index}) is available")
                    else:
                        rospy.logwarn(f"Camera device {device} opened but cannot read frames")
                    cap.release()
                else:
                    rospy.logwarn(f"Cannot open camera device {device}")
            except Exception as e:
                rospy.logwarn(f"Error checking device {device}: {e}")
        
        # 2. 如果没有找到可用摄像头，尝试系统特定方法
        if not available_cameras:
            rospy.logwarn("No cameras found through device files, trying system methods")
            try:
                # 在Linux系统上尝试使用v4l2-ctl列出设备
                import subprocess
                try:
                    result = subprocess.check_output(['v4l2-ctl', '--list-devices'], universal_newlines=True)
                    rospy.loginfo(f"V4L2 device list: {result}")
                    # 解析输出寻找可用设备
                except Exception as e:
                    rospy.logwarn(f"Error running v4l2-ctl: {e}")
            except Exception as e:
                rospy.logwarn(f"Error during system-specific camera detection: {e}")
        
        # 3. 最后，如果其他方法都失败，尝试索引0-9直接打开
        if not available_cameras:
            rospy.logwarn("Trying direct camera indices 0-9")
            for i in range(10):
                try:
                    cap = cv2.VideoCapture(i)
                    if cap.isOpened():
                        ret, _ = cap.read()
                        if ret:
                            available_cameras.append(i)
                            rospy.loginfo(f"Camera index {i} is available")
                        cap.release()
                except Exception:
                    pass
        
        if available_cameras:
            rospy.loginfo(f"Available camera indices: {available_cameras}")
        else:
            rospy.logerr("No available cameras found!")
            
            # 尝试使用fswebcam检查设备
            try:
                import subprocess
                rospy.loginfo("Trying fswebcam to detect devices...")
                try:
                    result = subprocess.check_output(['fswebcam', '-l'], universal_newlines=True)
                    rospy.loginfo(f"fswebcam devices: {result}")
                except Exception as e:
                    rospy.logwarn(f"Error running fswebcam: {e}")
            except Exception:
                pass
                
        return available_cameras
    
    def init_stereo_rectification(self):
        """初始化立体配准参数"""
        # 相机内参矩阵 (近似值)
        self.K_left = np.array([
            [600.0, 0.0, 320.0],
            [0.0, 600.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.K_right = self.K_left.copy()
        
        # 畸变系数 (近似值)
        self.D_left = np.zeros((5, 1))
        self.D_right = np.zeros((5, 1))
        
        # 旋转矩阵 (对于平行摄像头，这是单位矩阵)
        self.R = np.eye(3)
        
        # 平移向量 (假设摄像头间距为6.5cm)
        self.T = np.array([-0.065, 0, 0])
        
        # 图像尺寸
        self.img_size = (640, 480)
        
        # 计算立体配准映射
        try:
            R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
                self.K_left, self.D_left, self.K_right, self.D_right,
                self.img_size, self.R, self.T, alpha=0
            )
            
            # 计算映射矩阵
            self.map1_left, self.map2_left = cv2.initUndistortRectifyMap(
                self.K_left, self.D_left, R1, P1, self.img_size, cv2.CV_32FC1
            )
            self.map1_right, self.map2_right = cv2.initUndistortRectifyMap(
                self.K_right, self.D_right, R2, P2, self.img_size, cv2.CV_32FC1
            )
            
            # 存储用于深度计算的Q矩阵
            self.Q = Q
            
            rospy.loginfo("立体配准映射初始化成功")
        except Exception as e:
            rospy.logerr(f"立体配准映射初始化失败: {e}")
            # 创建默认映射（不做任何变换）
            self.map1_left, self.map2_left = np.meshgrid(
                np.arange(self.img_size[0]),
                np.arange(self.img_size[1])
            )
            self.map1_right, self.map2_right = self.map1_left.copy(), self.map2_left.copy()
            self.Q = np.eye(4)
    
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
                
                # 如果没有可用的摄像头，尝试重新扫描
                if not self.available_cameras:
                    self.available_cameras = self.find_available_cameras()
                    
                    if not self.available_cameras:
                        rospy.logerr("No cameras available after re-scanning")
                        # 创建一个模拟视频源用于测试
                        self.create_dummy_camera()
                        return False
                    else:
                        self.camera_index = self.available_cameras[0]
                
                # 检查camera_index是否在available_cameras中
                if self.camera_index not in self.available_cameras and self.available_cameras:
                    rospy.logwarn(f"Camera index {self.camera_index} is not available. Using index {self.available_cameras[0]}")
                    self.camera_index = self.available_cameras[0]
                
                # 尝试多种打开摄像头的方式
                # 1. 直接使用OpenCV打开
                self.camera = cv2.VideoCapture(self.camera_index)
                
                if not self.camera.isOpened():
                    # 2. 尝试使用设备文件路径
                    device_path = f"/dev/video{self.camera_index}"
                    rospy.loginfo(f"Trying to open camera using device path: {device_path}")
                    self.camera = cv2.VideoCapture(device_path)
                
                if not self.camera.isOpened() and cv2.getBuildInformation().find("GStreamer") != -1:
                    # 3. 尝试使用GStreamer管道
                    # 首先使用通用MJPG格式
                    gst_pipeline = (
                        f"v4l2src device=/dev/video{self.camera_index} ! "
                        f"image/jpeg,width=1280,height=480,framerate={self.frame_rate}/1 ! "
                        f"jpegdec ! videoconvert ! appsink"
                    )
                    rospy.loginfo(f"Trying GStreamer pipeline: {gst_pipeline}")
                    self.camera = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
                
                if not self.camera.isOpened() and cv2.getBuildInformation().find("GStreamer") != -1:
                    # 4. 尝试使用更通用的格式
                    gst_pipeline = (
                        f"v4l2src device=/dev/video{self.camera_index} ! "
                        f"video/x-raw,width=1280,height=480,framerate={self.frame_rate}/1 ! "
                        f"videoconvert ! appsink"
                    )
                    rospy.loginfo(f"Trying generic GStreamer pipeline: {gst_pipeline}")
                    self.camera = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
                
                if not self.camera.isOpened():
                    rospy.logerr(f"Failed to open camera at index {self.camera_index}")
                    # 移除失败的相机索引，避免重复尝试
                    if self.camera_index in self.available_cameras:
                        self.available_cameras.remove(self.camera_index)
                        
                    # 尝试使用模拟摄像头
                    self.create_dummy_camera()
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
                        # 创建模拟摄像头
                        self.create_dummy_camera()
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
            # 创建模拟摄像头
            self.create_dummy_camera()
            return False
            
    def create_dummy_camera(self):
        """创建模拟摄像头用于测试"""
        rospy.logwarn("Creating dummy camera for testing")
        # 标记使用模拟摄像头
        self.using_dummy_camera = True
        # 设置运行状态为真，让系统继续运行
        self.is_running = True
        
        # 创建一个彩色的测试图案
        self.dummy_left = np.zeros((480, 640, 3), dtype=np.uint8)
        self.dummy_right = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 在左右图像上绘制不同的测试图案
        # 左图: 红色网格
        for i in range(0, 480, 40):
            cv2.line(self.dummy_left, (0, i), (640, i), (0, 0, 255), 1)
        for i in range(0, 640, 40):
            cv2.line(self.dummy_left, (i, 0), (i, 480), (0, 0, 255), 1)
            
        # 右图: 蓝色网格 (稍微偏移以模拟视差)
        for i in range(0, 480, 40):
            cv2.line(self.dummy_right, (0, i), (640, i), (255, 0, 0), 1)
        for i in range(10, 650, 40):  # 10像素的偏移
            cv2.line(self.dummy_right, (i, 0), (i, 480), (255, 0, 0), 1)
            
        # 添加文本
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.dummy_left, "DUMMY LEFT CAMERA", (50, 240), font, 1, (255, 255, 255), 2)
        cv2.putText(self.dummy_right, "DUMMY RIGHT CAMERA", (50, 240), font, 1, (255, 255, 255), 2)
        
        # 添加几个3D测试对象 (不同位置的圆，可以测试立体视觉)
        for i in range(5):
            # 随机位置
            x = 100 + i * 100
            y = 100 + i * 50
            # 右图中的偏移量 - 距离越近，视差越大
            disparity = 30 - i * 5  # 0-25的视差范围
            
            # 在左右图像上画圆
            cv2.circle(self.dummy_left, (x, y), 20, (0, 255, 0), -1)
            cv2.circle(self.dummy_right, (x - disparity, y), 20, (0, 255, 0), -1)
            
            # 添加深度标签
            depth_text = f"D{i+1}"
            cv2.putText(self.dummy_left, depth_text, (x-10, y+5), font, 0.6, (0, 0, 0), 2)
            cv2.putText(self.dummy_right, depth_text, (x-disparity-10, y+5), font, 0.6, (0, 0, 0), 2)
        
        rospy.loginfo("Dummy camera created successfully")
    
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
        """Compute depth map from stereo images"""
        if not self.use_depth:
            # 如果未启用深度计算，返回空白图像
            return np.zeros((480, 640), dtype=np.uint8)
        
        try:
            # 确保输入图像为灰度图
            if len(left_image.shape) == 3:
                left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
                right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
            else:
                left_gray = left_image
                right_gray = right_image
            
            if self.stereo_processor is not None:
                # 计算视差图
                disparity_left = self.stereo_processor.compute(left_gray, right_gray)
                
                if self.stereo_processor_right is not None and self.wls_filter is not None:
                    # 计算右视图视差图
                    disparity_right = self.stereo_processor_right.compute(right_gray, left_gray)
                    
                    # 使用WLS滤波器改善视差图
                    filtered_disp = self.wls_filter.filter(disparity_left, left_gray, None, disparity_right)
                    
                    # 规范化视差图以便显示
                    norm_disp = cv2.normalize(filtered_disp, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                else:
                    # 如果没有WLS滤波器，直接规范化左视图视差图
                    norm_disp = cv2.normalize(disparity_left, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                
                # 使用视差图和Q矩阵计算3D点云
                if hasattr(self, 'Q'):
                    try:
                        # 创建伪彩色深度图
                        depth_colormap = cv2.applyColorMap(norm_disp, cv2.COLORMAP_JET)
                        
                        # 在深度图上添加文本说明
                        cv2.putText(depth_colormap, "Depth Map", (10, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                        
                        # 在深度图上添加色条
                        bar_width = 20
                        bar_height = 200
                        bar_x = depth_colormap.shape[1] - bar_width - 10
                        bar_y = (depth_colormap.shape[0] - bar_height) // 2
                        
                        # 创建渐变色条
                        for i in range(bar_height):
                            color_idx = int(255 * (1 - i / bar_height))
                            color = cv2.applyColorMap(np.array([[color_idx]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0]
                            cv2.rectangle(depth_colormap, 
                                        (bar_x, bar_y + i), 
                                        (bar_x + bar_width, bar_y + i + 1), 
                                        (int(color[0]), int(color[1]), int(color[2])), 
                                        -1)
                        
                        # 添加标签
                        cv2.putText(depth_colormap, "Near", (bar_x - 40, bar_y + 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.putText(depth_colormap, "Far", (bar_x - 40, bar_y + bar_height - 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        
                        return depth_colormap
                    except Exception as e:
                        rospy.logwarn(f"Error creating depth colormap: {e}")
                
                # 如果3D重建失败，返回灰度视差图
                return cv2.cvtColor(norm_disp, cv2.COLOR_GRAY2BGR)
            else:
                # 如果没有立体处理器，返回空白图像
                blank = np.zeros_like(left_image)
                cv2.putText(blank, "Depth processing disabled", (50, 240), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                return blank
                
        except Exception as e:
            rospy.logerr(f"Error computing depth map: {e}")
            # 返回带有错误信息的空白图像
            blank = np.zeros_like(left_image)
            cv2.putText(blank, f"Depth error: {str(e)[:30]}", (50, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return blank
    
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
        """Process stereo images according to the selected method"""
        # 获取当前的立体显示方法
        current_method = self.stereo_method
        
        # 首先进行立体配准
        left_rectified = cv2.remap(left_image, self.map1_left, self.map2_left, cv2.INTER_LINEAR)
        right_rectified = cv2.remap(right_image, self.map1_right, self.map2_right, cv2.INTER_LINEAR)
        
        # 在配准后的图像上绘制水平线以显示对齐效果
        if current_method == 'rectified':
            # 创建配准后的合并视图
            merged = np.zeros((480, 1280, 3), dtype=np.uint8)
            merged[:, :640] = left_rectified
            merged[:, 640:] = right_rectified
            
            # 绘制水平线以显示对齐效果
            for y in range(0, 480, 40):
                cv2.line(merged, (0, y), (1280, y), (0, 255, 0), 1)
                
            # 添加文本标签
            cv2.putText(merged, "Rectified Stereo", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            return merged
            
        elif current_method == 'anaglyph':
            # 使用配准后的图像创建红青立体图
            return self.create_anaglyph(left_rectified, right_rectified)
            
        elif current_method == 'wiggle':
            # 使用配准后的图像创建摇摆效果
            return self.create_wiggle(left_rectified, right_rectified)
            
        elif current_method == 'side_by_side':
            # 使用配准后的图像创建并排3D效果
            return self.create_side_by_side_3d(left_rectified, right_rectified)
            
        else:
            # 默认使用配准视图
            merged = np.zeros((480, 1280, 3), dtype=np.uint8)
            merged[:, :640] = left_rectified
            merged[:, 640:] = right_rectified
            return merged
    
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
        
        # 标记错误计数，连续错误过多则重启摄像头
        error_count = 0
        max_errors = 10
        
        # 用于计算FPS的变量
        frame_count = 0
        last_time = time.time()
        fps = 0
        
        while not rospy.is_shutdown() and self.is_running:
            # Check if we have a valid camera
            if hasattr(self, 'using_dummy_camera') and self.using_dummy_camera:
                # 使用模拟摄像头数据
                left_img = self.dummy_left.copy()
                right_img = self.dummy_right.copy()
                
                # 为模拟视频添加动态效果 - 每帧移动圆形
                self.animate_dummy_frames()
                
                # 模拟成功读取帧
                ret = True
            else:
                # 使用真实摄像头
                with self.camera_lock:
                    if self.camera is None or not self.is_running:
                        rate.sleep()
                        continue
                    
                    # Capture frame
                    try:
                        ret, frame = self.camera.read()
                    except Exception as e:
                        rospy.logerr(f"Error reading camera frame: {e}")
                        ret = False
            
                if not ret:
                    error_count += 1
                    rospy.logwarn(f"Failed to read camera frame ({error_count}/{max_errors})")
                    
                    # 如果连续错误次数过多，尝试重新打开摄像头
                    if error_count >= max_errors:
                        rospy.logerr(f"Too many consecutive camera errors. Attempting to reopen camera.")
                        error_count = 0
                        self.open_camera()
                    
                    rate.sleep()
                    continue
                
                error_count = 0  # 成功读取后重置错误计数
                
                # 处理帧数据
                try:
                    # 检查是否为有效的双目图像
                    if frame.shape[1] != 1280 or frame.shape[0] != 480:
                        rospy.logwarn(f"Unexpected frame size: {frame.shape[1]}x{frame.shape[0]}, expecting 1280x480. Skipping frame.")
                        rate.sleep()
                        continue
                    
                    # 检测无效帧 (全黑或异常数据)
                    if np.mean(frame) < 5 or np.isnan(np.sum(frame)):
                        rospy.logwarn("Invalid frame detected (black or NaN values). Skipping.")
                        rate.sleep()
                        continue
                    
                    # 复制帧以避免修改原始数据
                    frame = frame.copy()
                    
                    # 分割为左右图像
                    left_img = frame[:, :640]
                    right_img = frame[:, 640:]
                    
                    # 验证分割后的图像尺寸
                    if left_img.shape[1] != 640 or right_img.shape[1] != 640:
                        rospy.logerr(f"Invalid split frame sizes. Left: {left_img.shape}, Right: {right_img.shape}")
                        rate.sleep()
                        continue
                except Exception as e:
                    rospy.logerr(f"Error processing frame: {e}")
                    rate.sleep()
                    continue
            
            try:
                # 计算和显示FPS
                frame_count += 1
                if frame_count >= 10:
                    current_time = time.time()
                    fps = frame_count / (current_time - last_time)
                    last_time = current_time
                    frame_count = 0
                
                # 在左右图像上添加FPS信息
                if hasattr(self, 'using_dummy_camera') and self.using_dummy_camera:
                    cv2.putText(left_img, "DUMMY MODE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.7, (0, 255, 255), 2)
                
                cv2.putText(left_img, f"FPS: {fps:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.7, (0, 255, 255), 2)
                
                # Process the stereo images according to the selected method
                merged_img = self.process_stereo_images(left_img, right_img)
                
                # Create depth map if requested
                if self.use_depth and self.stereo_processor is not None:
                    try:
                        depth_img = self.compute_depth_map(left_img, right_img)
                        
                        # Convert to ROS image and publish
                        depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding="mono8")
                        depth_msg.header.stamp = rospy.Time.now()
                        depth_msg.header.frame_id = "stereo_depth"
                        self.depth_pub.publish(depth_msg)
                    except Exception as e:
                        rospy.logwarn(f"Error computing depth map: {e}")
                
                # Update header timestamps
                now = rospy.Time.now()
                camera_info_left.header.stamp = now
                camera_info_right.header.stamp = now
                
                # Convert to ROS images
                left_msg = self.bridge.cv2_to_imgmsg(left_img, "bgr8")
                right_msg = self.bridge.cv2_to_imgmsg(right_img, "bgr8")
                merged_msg = self.bridge.cv2_to_imgmsg(merged_img, "bgr8")
                
                # Set headers
                left_msg.header.stamp = now
                left_msg.header.frame_id = "stereo_left"
                right_msg.header.stamp = now
                right_msg.header.frame_id = "stereo_right"
                merged_msg.header.stamp = now
                merged_msg.header.frame_id = "stereo_camera"
                
                # Publish images and camera info
                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)
                self.merged_pub.publish(merged_msg)
                self.left_info_pub.publish(camera_info_left)
                self.right_info_pub.publish(camera_info_right)
                
            except CvBridgeError as e:
                rospy.logerr(f"CV Bridge error: {e}")
            except Exception as e:
                rospy.logerr(f"Error in publish loop: {e}")
            
            # Maintain frame rate
            rate.sleep()
            
    def animate_dummy_frames(self):
        """为模拟摄像头添加简单动画效果"""
        if not hasattr(self, 'dummy_animation_counter'):
            self.dummy_animation_counter = 0
            self.dummy_animation_dir = 1
        
        # 更新计数器
        self.dummy_animation_counter += self.dummy_animation_dir
        
        # 反转方向如果达到边界
        if self.dummy_animation_counter > 20 or self.dummy_animation_counter < -20:
            self.dummy_animation_dir *= -1
        
        # 基于计数器更新圆形位置
        offset = self.dummy_animation_counter
        
        # 清除之前的动态内容
        self.dummy_left = np.zeros((480, 640, 3), dtype=np.uint8)
        self.dummy_right = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 重新绘制网格
        # 左图: 红色网格
        for i in range(0, 480, 40):
            cv2.line(self.dummy_left, (0, i), (640, i), (0, 0, 255), 1)
        for i in range(0, 640, 40):
            cv2.line(self.dummy_left, (i, 0), (i, 480), (0, 0, 255), 1)
            
        # 右图: 蓝色网格 (稍微偏移以模拟视差)
        for i in range(0, 480, 40):
            cv2.line(self.dummy_right, (0, i), (640, i), (255, 0, 0), 1)
        for i in range(10, 650, 40):  # 10像素的偏移
            cv2.line(self.dummy_right, (i, 0), (i, 480), (255, 0, 0), 1)
        
        # 添加文本
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.dummy_left, "DUMMY LEFT CAMERA", (50, 240), font, 1, (255, 255, 255), 2)
        cv2.putText(self.dummy_right, "DUMMY RIGHT CAMERA", (50, 240), font, 1, (255, 255, 255), 2)
        
        # 添加一个动态移动的对象
        for i in range(5):
            # 根据动画位置调整坐标
            x = 100 + i * 100
            y = 100 + i * 50 + offset
            # 右图中的偏移量
            disparity = 30 - i * 5
            
            # 在左右图像上画圆
            cv2.circle(self.dummy_left, (x, y), 20, (0, 255, 0), -1)
            cv2.circle(self.dummy_right, (x - disparity, y), 20, (0, 255, 0), -1)
            
            # 添加深度标签
            depth_text = f"D{i+1}"
            cv2.putText(self.dummy_left, depth_text, (x-10, y+5), font, 0.6, (0, 0, 0), 2)
            cv2.putText(self.dummy_right, depth_text, (x-disparity-10, y+5), font, 0.6, (0, 0, 0), 2)
            
        # 添加时间戳
        timestamp = f"Time: {time.strftime('%H:%M:%S')}"
        cv2.putText(self.dummy_left, timestamp, (10, 30), font, 0.6, (255, 255, 0), 2)
        cv2.putText(self.dummy_right, timestamp, (10, 30), font, 0.6, (255, 255, 0), 2)
    
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
    
    def stereo_method_callback(self, msg):
        """处理立体显示方法的更改"""
        method = msg.data.strip("'\"")  # 去除可能的引号
        if method in ['rectified', 'anaglyph', 'wiggle', 'side_by_side']:
            old_method = self.stereo_method
            self.stereo_method = method
            # 更新全局参数
            rospy.set_param('/stereo_camera/stereo_method', method)
            rospy.loginfo(f"立体显示方法已更改: {old_method} -> {method}")
        else:
            rospy.logwarn(f"不支持的立体显示方法: {method}")

if __name__ == '__main__':
    try:
        node = StereoCameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 