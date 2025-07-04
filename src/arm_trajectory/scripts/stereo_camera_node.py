#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import threading
import time as time_module
from std_msgs.msg import Int32

# 检查是否支持深度处理
try:
    import cv2.ximgproc
    HAVE_XIMGPROC = True
    rospy.loginfo("cv2.ximgproc可用，深度处理已启用")
except ImportError:
    HAVE_XIMGPROC = False
    rospy.logwarn("cv2.ximgproc不可用，高级深度处理将被禁用")

class StereoCameraNode:
    """双目相机节点，用于捕获和发布立体图像"""
    
    def __init__(self):
        rospy.init_node('stereo_camera_node', anonymous=True)
        
        # 获取参数
        self.camera_index = int(rospy.get_param('~camera_index', 0))
        self.frame_rate = int(rospy.get_param('~frame_rate', 30))
        self.use_depth = rospy.get_param('~use_depth', True)
        
        # 如果ximgproc不可用，禁用深度图
        if not HAVE_XIMGPROC and self.use_depth:
            rospy.logwarn("请求了深度处理但cv2.ximgproc不可用。请安装opencv-contrib-python包。")
            rospy.logwarn("深度图生成将被禁用。")
            self.use_depth = False
        
        # 视图模式: 0=左图, 1=右图, 2=深度图
        self.view_mode = int(rospy.get_param('~view_mode', 0))
        self.use_mock_camera = rospy.get_param('~use_mock_camera', False)
        
        # 初始化bridge
        self.bridge = CvBridge()
        
        # 初始化发布器
        self.left_pub = rospy.Publisher('/stereo_camera/left/image_raw', Image, queue_size=1)
        self.right_pub = rospy.Publisher('/stereo_camera/right/image_raw', Image, queue_size=1)
        self.current_view_pub = rospy.Publisher('/stereo_camera/current_view', Image, queue_size=1)
        
        # 仅在启用深度图且ximgproc可用时创建深度图发布器
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
        
        # 初始化相机
        self.camera = None
        self.is_running = False
        self.is_reconnecting = False
        self.frame_count_fail = 0
        self.camera_lock = threading.Lock()
        
        # 初始化帧
        self.left_frame = None
        self.right_frame = None
        
        # 初始化立体处理器
        self.stereo_processor = None
        self.stereo_processor_right = None
        self.wls_filter = None
        self.init_stereo_processor()
        
        # 尝试打开相机
        self.open_camera()
        
        # 启动发布线程
        self.thread = threading.Thread(target=self.publish_loop)
        self.thread.daemon = True
        self.thread.start()
        
        rospy.loginfo("立体相机节点初始化完成")
        
        # 注册关闭钩子
        rospy.on_shutdown(self.shutdown)
    
        def view_mode_callback(self, msg):
        """处理视图模式切换消息"""
        new_mode = msg.data
        # 确保模式在有效范围内
        if 0 <= new_mode <= 2:
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
        """初始化立体视觉处理器"""
        # 只在启用深度图且ximgproc可用时初始化
        if not self.use_depth or not HAVE_XIMGPROC:
            rospy.loginfo("深度处理已禁用")
            return
            
        try:
            rospy.loginfo("正在初始化立体处理器以计算深度")
            
            # 创建视差计算器 - SGBM算法
            self.stereo_processor = cv2.StereoSGBM.create(
                minDisparity=0,
                numDisparities=128,  # 必须是16的倍数
                blockSize=5,
                P1=8 * 3 * 5 ** 2,
                P2=32 * 3 * 5 ** 2,
                disp12MaxDiff=1,
                uniquenessRatio=15,
                speckleWindowSize=100,
                speckleRange=1,
                preFilterCap=63,
                mode=1  # cv2.STEREO_SGBM_MODE_SGBM_3WAY
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
                mode=1
            )
            self.wls_filter.setLambda(8000)
            self.wls_filter.setSigmaColor(1.5)
            
            rospy.loginfo("立体处理器初始化成功")
            
        except Exception as e:
            rospy.logerr(f"初始化立体处理器错误: {e}")
            self.use_depth = False
            self.stereo_processor = None
            self.stereo_processor_right = None
            self.wls_filter = None

        def open_camera(self):
        """打开相机并设置参数"""
        # 如果使用模拟相机模式，则不需要真实相机
        if self.use_mock_camera:
            rospy.loginfo("使用模拟相机模式，跳过真实相机初始化")
            self.is_running = True
            self.is_reconnecting = False
            return True
            
        try:
            with self.camera_lock:
                # 关闭现有相机（如果已打开）
                if self.camera is not None:
                    try:
                        self.camera.release()
                    except Exception as e:
                        rospy.logwarn(f"释放旧相机时出现警告: {e}")
                    self.camera = None
                    
                    # 调用GC帮助释放资源
                    try:
                        import gc
                        gc.collect()
                    except:
                        pass
                    
                    # 给系统时间完全释放资源
                    try:
                        time_module.sleep(2.0)
                    except Exception as e:
                        rospy.logwarn(f"休眠时出错: {e}")
                
                # 获取分辨率参数
                camera_width = int(rospy.get_param('~camera_width', 1280))
                camera_height = int(rospy.get_param('~camera_height', 480))
                camera_format = rospy.get_param('~camera_format', 'MJPEG')
                
                rospy.loginfo(f"尝试设置相机分辨率为 {camera_width}x{camera_height}，格式: {camera_format}")
                
                # 记录相机参数
                self.camera_baseline = 60.0  # 基线距离（毫米）
                self.camera_fov = 72.0  # 视场角（度）
                self.camera_focal_length = 3.6  # 焦距（毫米）
                
                # 尝试多个相机设备
                devices_to_try = ["/dev/video0", "/dev/video1", "/dev/video2"]
                
                for device in devices_to_try:
                    rospy.loginfo(f"尝试打开相机设备: {device}")
                    
                    # 尝试直接打开相机
                    try:
                        # 明确指定V4L2后端
                        self.camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
                        
                        # 检查相机是否成功打开
                        if not self.camera.isOpened():
                            rospy.logwarn(f"无法打开相机设备: {device}")
                            continue
                        
                        # 重要：首先设置FOURCC（格式），然后再设置其他参数
                        if camera_format == 'MJPEG':
                            mjpg_fourcc = cv2.VideoWriter.fourcc('M','J','P','G')
                            self.camera.set(cv2.CAP_PROP_FOURCC, mjpg_fourcc)
                        else:
                            yuyv_fourcc = cv2.VideoWriter.fourcc('Y','U','Y','V')
                            self.camera.set(cv2.CAP_PROP_FOURCC, yuyv_fourcc)

                        # 设置相机参数
                        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, camera_width)
                        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_height)
                        self.camera.set(cv2.CAP_PROP_FPS, self.frame_rate)
                        
                        # 添加诊断信息
                        current_fourcc = int(self.camera.get(cv2.CAP_PROP_FOURCC))
                        current_fourcc_str = "".join([chr((current_fourcc >> 8 * i) & 0xFF) for i in range(4)])
                        rospy.loginfo(f"相机格式: {current_fourcc_str}")
                        rospy.loginfo(f"相机分辨率: {self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)}x{self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
                        rospy.loginfo(f"相机帧率: {self.camera.get(cv2.CAP_PROP_FPS)}")
                        
                        # 给相机时间初始化
                        time_module.sleep(2.0)
    
                            # 尝试读取测试帧
                        for attempt in range(5):
                            try:
                                start_time = time_module.time()
                                ret, test_frame = self.camera.read()
                                end_time = time_module.time()
                                elapsed = end_time - start_time
                                
                                # 记录读取耗时
                                rospy.loginfo(f"相机读取耗时: {elapsed:.3f}秒")
                                
                                # 检查是否成功读取帧
                                if ret and test_frame is not None and test_frame.size > 0:
                                    height, width = test_frame.shape[:2]
                                    rospy.loginfo(f"相机已成功打开，设备: {device}, 分辨率: {width}x{height}, 帧率: {self.frame_rate}")
                                    
                                    # 相机参数记录
                                    self.image_width = width
                                    self.image_height = height
                                    
                                    self.is_running = True
                                    self.is_reconnecting = False
                                    self.frame_count_fail = 0
                                    return True
                                else:
                                    # 如果读取失败，等待后重试
                                    rospy.logwarn(f"读取测试帧失败，尝试 {attempt+1}/5")
                                    time_module.sleep(1.0)
                            except Exception as e:
                                rospy.logwarn(f"读取测试帧异常: {e}")
                                time_module.sleep(1.0)
                        
                        # 如果所有尝试都失败，释放相机
                        rospy.logerr(f"相机设备 {device} 打开成功但无法读取图像")
                        self.camera.release()
                        self.camera = None
                        
                        # 尝试使用GStreamer方法
                        try:
                            import platform
                            if platform.system() == "Linux":
                                rospy.loginfo(f"尝试使用GStreamer打开设备: {device}")
                                if isinstance(device, str) and device.startswith("/dev/video"):
                                    # 设置GStreamer管道
                                    if camera_format == 'MJPEG':
                                        gst_str = f"v4l2src device={device} ! image/jpeg,width={camera_width},height={camera_height},framerate={self.frame_rate}/1 ! jpegdec ! videoconvert ! appsink"
                                    else:  # YUY2
                                        gst_str = f"v4l2src device={device} ! video/x-raw,format=YUY2,width={camera_width},height={camera_height},framerate={self.frame_rate}/1 ! videoconvert ! appsink"
                                    
                                    rospy.loginfo(f"使用GStreamer管道: {gst_str}")
                                    self.camera = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
                                    
                                    # 等待GStreamer初始化
                                    time_module.sleep(2.0)
                                    
                                    if self.camera.isOpened():
                                        # 添加GStreamer状态诊断
                                        current_fourcc = int(self.camera.get(cv2.CAP_PROP_FOURCC))
                                        current_fourcc_str = "".join([chr((current_fourcc >> 8 * i) & 0xFF) for i in range(4)])
                                        rospy.loginfo(f"GStreamer相机格式: {current_fourcc_str}")
                                        rospy.loginfo(f"GStreamer相机分辨率: {self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)}x{self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
                                        
                                        # 测试GStreamer相机
                                        success = False
                                        for i in range(5):
                                            ret, test_frame = self.camera.read()
                                            if ret and test_frame is not None and test_frame.size > 0:
                                                height, width = test_frame.shape[:2]
                                                rospy.loginfo(f"GStreamer成功打开相机: {device}, 分辨率: {width}x{height}")
                                                self.is_running = True
                                                self.is_reconnecting = False
                                                self.frame_count_fail = 0
                                                self.image_width = width
                                                self.image_height = height
                                                success = True
                                                return True
                                            else:
                                                rospy.logwarn(f"GStreamer读取测试帧失败，尝试 {i+1}/5")
                                                time_module.sleep(1.0)
                                        
                                        if not success:
                                            rospy.logwarn("GStreamer无法读取有效帧")
                                            if self.camera:
                                                self.camera.release()
                                                self.camera = None
                        except Exception as e:
                            rospy.logwarn(f"GStreamer尝试失败: {e}")
                            if self.camera and self.camera.isOpened():
                                self.camera.release()
                                self.camera = None
                    except Exception as e:
                        rospy.logwarn(f"打开相机异常: {e}")
                        if self.camera:
                            try:
                                self.camera.release()
                            except:
                                pass
                            self.camera = None
    
                    # 如果所有设备都失败
                rospy.logerr("所有相机设备都无法正常工作")
                # 检查相机设备是否存在
                try:
                    import os
                    if os.path.exists("/dev/video0"):
                        rospy.loginfo("设备/dev/video0存在，但无法正常访问，可能是权限或驱动问题")
                        # 检查设备权限
                        try:
                            import subprocess
                            result = subprocess.run(["ls", "-l", "/dev/video0"], capture_output=True, text=True)
                            rospy.loginfo(f"设备权限: {result.stdout.strip()}")
                            
                            # 检查支持的分辨率
                            try:
                                result = subprocess.run(["v4l2-ctl", "-d", "/dev/video0", "--list-formats-ext"], 
                                                       capture_output=True, text=True)
                                rospy.loginfo(f"支持的格式:\n{result.stdout.strip()}")
                            except:
                                rospy.logwarn("无法获取支持的格式信息，请确保安装v4l-utils")
                            
                        except Exception as e:
                            rospy.logwarn(f"无法检查设备权限: {e}")
                    else:
                        rospy.loginfo("设备/dev/video0不存在，请检查相机连接或驱动")
                except Exception as e:
                    rospy.logwarn(f"检查设备文件异常: {e}")
                    
                self.is_reconnecting = True
                return False
                
        except Exception as e:
            rospy.logerr(f"打开相机时出错: {e}")
            self.is_running = False
            self.is_reconnecting = True
            return False
    
        def get_current_view(self, left_image, right_image):
        """根据当前视图模式返回相应的图像"""
        # 如果任一图像为空，创建占位图像
        if left_image is None or right_image is None:
            return self.create_placeholder_image("等待摄像头连接...", 640, 480)
            
        # 确保图像尺寸一致
        if left_image.shape != right_image.shape:
            try:
                height = min(left_image.shape[0], right_image.shape[0])
                width = min(left_image.shape[1], right_image.shape[1])
                
                if height <= 0 or width <= 0:
                    return self.create_placeholder_image("图像尺寸无效", 640, 480)
                    
                left_image = left_image[:height, :width]
                right_image = right_image[:height, :width]
            except Exception as e:
                rospy.logerr(f"调整图像尺寸错误: {e}")
                return self.create_placeholder_image("图像处理错误", 640, 480)
        
        try:
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
                        # 创建彩色深度图
                        colored_depth = self.create_colored_depth_map(depth_map)
                        return colored_depth
                    else:
                        # 如果深度图计算失败，返回左图
                        rospy.logwarn("深度图计算失败，返回左图")
                        return left_image
                else:
                    # 如果深度图不可用，返回左图
                    rospy.logwarn("深度图不可用，返回左图")
                    return left_image
            
            # 默认返回左图
            return left_image
            
        except Exception as e:
            rospy.logerr(f"获取当前视图错误: {e}")
            return self.create_placeholder_image("视图处理错误", 640, 480)

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
                
                # 使用WLS滤波器优化
                if self.wls_filter:
                    filtered_disp = self.wls_filter.filter(left_disp, left_gray, disparity_map_right=right_disp)
                    # 处理无效区域
                    _, filtered_disp = cv2.threshold(filtered_disp, 0, np.inf, cv2.THRESH_TOZERO)
                    
                    # 转换视差图到实际深度值
                    if hasattr(self, 'camera_baseline') and hasattr(self, 'camera_focal_length'):
                        # 把视差图转换为毫米单位的深度图
                        valid_mask = filtered_disp > 0
                        depth_map = np.zeros_like(filtered_disp)
                        
                        # 估计像素大小
                        if hasattr(self, 'image_width') and hasattr(self, 'camera_fov'):
                            # 通过视场角和图像宽度估计像素大小
                            fov_rad = np.radians(self.camera_fov)
                            pixel_size_mm = 2 * np.tan(fov_rad / 2) * self.camera_focal_length / self.image_width
                            
                            # 转换视差到实际深度值
                            depth_map[valid_mask] = self.camera_baseline * self.camera_focal_length / (filtered_disp[valid_mask] * pixel_size_mm)
                            
                            # 限制深度范围
                            depth_map = np.clip(depth_map, 100, 10000)  # 100mm到10000mm
                            
                            return depth_map
                    
                    return filtered_disp
                    
            # 如果没有WLS滤波，返回原始视差图
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
            # 检查深度图类型
            if len(depth_map.shape) == 2:  # 单通道深度图
                # 将浮点深度图转换为彩色可视化
                colored_depth = self.create_colored_depth_map(depth_map)
                
                # 创建彩色深度图消息
                colored_depth_msg = self.bridge.cv2_to_imgmsg(colored_depth, encoding="bgr8")
                colored_depth_msg.header.stamp = rospy.Time.now()
                
                # 发布彩色深度图
                self.depth_pub.publish(colored_depth_msg)
                
                # 同时发布原始深度图（保留精确深度信息）
                raw_depth_msg = self.bridge.cv2_to_imgmsg(depth_map.astype(np.float32), encoding="32FC1")
                raw_depth_msg.header.stamp = rospy.Time.now()
                self.raw_depth_pub.publish(raw_depth_msg)
                
            else:  # 彩色深度图（已经是BGR格式）
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
        rate = rospy.Rate(self.frame_rate)
        placeholder_image = self.create_placeholder_image("等待相机连接...", 640, 480)
        
        # 如果使用模拟相机，创建模拟图像
        if self.use_mock_camera:
            mock_camera_pattern = rospy.get_param('~mock_camera_pattern', 'checkerboard')
            
            rospy.loginfo("启用模拟相机模式，实际相机将被忽略")
            # 创建左右模拟图像
            if mock_camera_pattern == 'checkerboard':
                self.mock_left_frame = self.create_checkerboard_pattern(640, 480, (200, 200, 200), (50, 50, 50))
                self.mock_right_frame = self.create_checkerboard_pattern(640, 480, (200, 200, 200), (100, 100, 100))
            else:
                # 默认使用纯色图像
                self.mock_left_frame = np.ones((480, 640, 3), dtype=np.uint8) * 100  # 灰色
                self.mock_right_frame = np.ones((480, 640, 3), dtype=np.uint8) * 150  # 浅灰色
        
        consecutive_failures = 0  # 连续失败计数
        try:
            last_diagnostic_time = time_module.time()
        except Exception as e:
            rospy.logwarn(f"初始化诊断时间错误: {e}")
            last_diagnostic_time = 0
        
        while not rospy.is_shutdown():
            try:
                # 模拟相机模式处理
                if self.use_mock_camera:
                    left_frame = self.mock_left_frame.copy()
                    right_frame = self.mock_right_frame.copy()
                    
                    # 添加时间戳
                    try:
                        timestamp = time_module.strftime("%H:%M:%S", time_module.localtime())
                        cv2.putText(left_frame, f"Mock Left {timestamp}", (20, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(right_frame, f"Mock Right {timestamp}", (20, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    except Exception as e:
                        rospy.logwarn(f"时间戳格式化错误: {e}")
                        cv2.putText(left_frame, "Mock Left", (20, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(right_frame, "Mock Right", (20, 30), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # 缓存帧
                    self.left_frame = left_frame
                    self.right_frame = right_frame
                    
                    # 获取当前视图
                    current_view = self.get_current_view(left_frame, right_frame)
                    
                    # 发布模拟图像
                    try:
                        left_msg = self.bridge.cv2_to_imgmsg(left_frame, "bgr8")
                        right_msg = self.bridge.cv2_to_imgmsg(right_frame, "bgr8")
                        
                        left_msg.header.stamp = rospy.Time.now()
                        right_msg.header.stamp = left_msg.header.stamp
                        
                        self.left_pub.publish(left_msg)
                        self.right_pub.publish(right_msg)
                        self.publish_current_view(current_view)
                        
                        # 发布相机信息
                        camera_info = CameraInfo()
                        camera_info.header = left_msg.header
                        camera_info.width = left_frame.shape[1]
                        camera_info.height = left_frame.shape[0]
                        self.left_info_pub.publish(camera_info)
                        self.right_info_pub.publish(camera_info)
                        
                        consecutive_failures = 0
                    except Exception as e:
                        rospy.logerr(f"发布模拟图像错误: {e}")
                        consecutive_failures += 1
                        
                    rate.sleep()
                    continue
    
                    # 实时相机模式
                with self.camera_lock:
                    if self.camera is None or not self.camera.isOpened():
                        if not self.is_reconnecting:
                            self.is_reconnecting = True
                            rospy.logwarn("摄像头连接丢失，尝试重新连接...")
                            self.open_camera()
                        
                        # 发布占位图像
                        self.publish_placeholder_images(placeholder_image)
                        rate.sleep()
                        continue
                        
                    # 读取帧，添加超时检测
                    try:
                        start_time = time_module.time()
                        # 确认相机处于打开状态
                        if not self.camera.isOpened():
                            rospy.logwarn("相机已关闭，尝试重新打开")
                            self.camera.release()
                            self.camera = None
                            self.is_reconnecting = True
                            self.open_camera()
                            self.publish_placeholder_images(placeholder_image)
                            rate.sleep()
                            continue
                            
                        # 先清除缓冲区（对某些驱动可能有帮助）
                        for _ in range(2):  # 丢弃可能滞留的帧
                            self.camera.grab()
                            
                        success, frame = self.camera.read()
                        end_time = time_module.time()
                        elapsed = end_time - start_time
                        
                        # 如果读取超时，重试一次
                        if elapsed > 1.0:
                            rospy.logwarn(f"相机读取超时 ({elapsed:.3f}秒)，尝试重新读取")
                            success, frame = self.camera.read()  # 再次尝试
                    except Exception as e:
                        rospy.logwarn(f"读取帧时出错: {e}")
                        # 如果无法获取时间或读取失败，继续尝试读取帧
                        try:
                            success, frame = self.camera.read()
                            elapsed = 0  # 设置一个默认值
                        except Exception as e2:
                            rospy.logwarn(f"重新读取帧失败: {e2}")
                            success = False
                            frame = None
                            elapsed = 0

                    # 定期输出诊断信息
                    try:
                        current_time = time_module.time()
                        if current_time - last_diagnostic_time > 30.0:  # 每30秒输出一次诊断信息
                            if self.camera and self.camera.isOpened():
                                rospy.loginfo(f"相机状态: isOpened=True, 读取耗时={elapsed:.3f}秒")
                            else:
                                rospy.loginfo("相机状态: isOpened=False")
                            last_diagnostic_time = current_time
                    except Exception as e:
                        rospy.logwarn(f"诊断时间计算错误: {e}")
                    
                    # 检查读取结果
                    if not success or frame is None or frame.size == 0:
                        self.frame_count_fail += 1
                        consecutive_failures += 1
                        rospy.logwarn(f"读取帧失败 ({self.frame_count_fail}/5), 连续失败: {consecutive_failures}")
                        
                        if self.frame_count_fail > 5:  # 连续5次失败，尝试重新打开摄像头
                            rospy.logwarn("摄像头读取失败，尝试重新连接...")
                            if self.camera:
                                self.camera.release()
                            self.camera = None
                            self.is_reconnecting = True
                            self.open_camera()
                        
                        # 发布占位图像
                        self.publish_placeholder_images(placeholder_image)
                        rate.sleep()
                        continue
    
                        # 重置失败计数
                    self.frame_count_fail = 0
                    consecutive_failures = 0
                    self.is_reconnecting = False
                    
                    # 处理双目相机图像
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
                            
                            # 如果启用深度图且ximgproc可用，计算并发布深度图
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
                            # 缓存帧
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
                consecutive_failures += 1
                
                # 如果连续失败次数过多，尝试重新初始化相机
                if consecutive_failures > 10:
                    rospy.logerr(f"连续出错 {consecutive_failures} 次，尝试重新初始化相机")
                    try:
                        with self.camera_lock:
                            if self.camera is not None:
                                self.camera.release()
                                self.camera = None
                            self.is_reconnecting = True
                            self.open_camera()
                            consecutive_failures = 0
                    except Exception as e:
                        rospy.logerr(f"重新初始化相机失败: {e}")
            
            rate.sleep()

        def create_placeholder_image(self, text, width=640, height=480):
        """创建占位图像，显示文本消息"""
        placeholder = np.zeros((height, width, 3), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1.0
        font_thickness = 2
        text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
        
        # 计算文本位置，使其居中
        text_x = (width - text_size[0]) // 2
        text_y = (height + text_size[1]) // 2
        
        # 添加文本
        cv2.putText(placeholder, text, (text_x, text_y), 
                    font, font_scale, (200, 200, 200), font_thickness)
        
        return placeholder
        
    def publish_placeholder_images(self, placeholder):
        """发布占位图像到所有图像话题"""
        try:
            timestamp = rospy.Time.now()
            img_msg = self.bridge.cv2_to_imgmsg(placeholder, "bgr8")
            img_msg.header.stamp = timestamp
            
            # 发布到所有图像话题
            self.left_pub.publish(img_msg)
            self.right_pub.publish(img_msg)
            self.current_view_pub.publish(img_msg)
            
            # 发布相机信息
            camera_info = CameraInfo()
            camera_info.header = img_msg.header
            camera_info.width = placeholder.shape[1]
            camera_info.height = placeholder.shape[0]
            self.left_info_pub.publish(camera_info)
            self.right_info_pub.publish(camera_info)
            
        except Exception as e:
            rospy.logerr(f"发布占位图像错误: {e}")

    def create_checkerboard_pattern(self, width, height, color1=(255, 255, 255), color2=(0, 0, 0), square_size=40):
        """创建棋盘格图案用于模拟相机图像"""
        # 创建空白图像
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 计算行列数
        num_rows = height // square_size + 1
        num_cols = width // square_size + 1
        
        # 绘制棋盘格
        for row in range(num_rows):
            for col in range(num_cols):
                # 计算当前方块坐标
                y1 = row * square_size
                x1 = col * square_size
                y2 = min(y1 + square_size, height)
                x2 = min(x1 + square_size, width)
                
                # 确定颜色
                if (row + col) % 2 == 0:
                    color = color1
                else:
                    color = color2
                    
                # 绘制方块
                img[y1:y2, x1:x2] = color
                
        return img

    def shutdown(self):
        """清理并关闭资源"""
        rospy.loginfo("关闭立体相机节点")
        self.is_running = False
        
        # 释放相机资源
        with self.camera_lock:
            if self.camera is not None:
                try:
                    self.camera.release()
                    rospy.loginfo("相机成功释放")
                except Exception as e:
                    rospy.logerr(f"释放相机出错: {e}")
                
                self.camera = None
                
        # 等待资源完全释放
        try:
            time_module.sleep(0.5)
        except Exception as e:
            rospy.logwarn(f"关闭时休眠错误: {e}")
        
        # 调用GC
        try:
            import gc
            gc.collect()
        except:
            pass
        
        rospy.loginfo("相机资源已成功释放")

if __name__ == '__main__':
    try:
        node = StereoCameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass