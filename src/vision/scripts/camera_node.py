#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os
import sys
import time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse

# Add current directory to path to find modules
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from camera_control import CameraControl, CameraMode

class CameraNode:
    """
    ROS node for camera control and image publishing
    """
    
    def __init__(self):
        """Initialize the camera node"""
        rospy.init_node('camera_node', anonymous=True)
        
        # Initialize parameters for stereo camera (1280x480 combined view)
        self.camera_index = rospy.get_param('~camera_index', 0)
        self.width = rospy.get_param('~width', 1280)
        self.height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 30)
        self.frame_id = rospy.get_param('~frame_id', 'camera')
        self.publish_rate = rospy.get_param('~publish_rate', 30)
        self.camera_enabled = rospy.get_param('~camera_enabled', True)
        self.initial_mode = rospy.get_param('~camera_mode', 0)  # Default to RGB mode
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize camera control
        self.camera = CameraControl(
            camera_index=self.camera_index,
            width=self.width,
            height=self.height,
            fps=self.fps
        )
        
        # Set camera mode
        self.camera.set_camera_mode(self.initial_mode)
        
        # Create service
        self.enable_service = rospy.Service('/camera/enable', SetBool, self.enable_camera_callback)
        
        # Create publishers
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
        self.mode_pub = rospy.Publisher('/camera/mode', Int32, queue_size=10, latch=True)
        
        # Add separate publishers for left and right cameras
        self.left_image_pub = rospy.Publisher('/stereo_camera/left/image_raw', Image, queue_size=10)
        self.right_image_pub = rospy.Publisher('/stereo_camera/right/image_raw', Image, queue_size=10)
        self.left_camera_info_pub = rospy.Publisher('/stereo_camera/left/camera_info', CameraInfo, queue_size=10)
        self.right_camera_info_pub = rospy.Publisher('/stereo_camera/right/camera_info', CameraInfo, queue_size=10)
        
        # Subscribe to mode topic
        self.mode_sub = rospy.Subscriber('/camera/mode', Int32, self.mode_callback)
        
        # Initialize camera info message
        self.camera_info_msg = self.create_camera_info_msg()
        
        # Publish initial mode
        self.publish_mode()
        
        # Create timer for image publishing
        self.timer = None
        
        rospy.loginfo(f"Camera node initialized with index {self.camera_index}")
        
        # Open camera if enabled
        if self.camera_enabled:
            self.start_camera()
    
    def publish_mode(self):
        """Publish current camera mode"""
        mode_msg = Int32()
        mode_msg.data = self.camera.camera_mode.value
        self.mode_pub.publish(mode_msg)
    
    def mode_callback(self, msg):
        """Handle camera mode message"""
        mode = msg.data
        self.camera.set_camera_mode(mode)
        rospy.loginfo(f"Camera mode changed to: {self.camera.camera_mode.name}")
    
    def enable_camera_callback(self, req):
        """Handle enable/disable camera service request"""
        enable = req.data
        
        if enable and not self.camera_enabled:
            # Enable camera
            success = self.start_camera()
            if success:
                self.camera_enabled = True
                return SetBoolResponse(True, "Camera enabled")
            else:
                return SetBoolResponse(False, "Failed to enable camera")
        elif not enable and self.camera_enabled:
            # Disable camera
            self.stop_camera()
            self.camera_enabled = False
            return SetBoolResponse(True, "Camera disabled")
        else:
            # No change needed
            return SetBoolResponse(True, f"Camera already {'enabled' if self.camera_enabled else 'disabled'}")
    
    def start_camera(self):
        """Start camera and publishing timer"""
        # 检查摄像头设备是否存在
        device_path = f"/dev/video{self.camera_index}"
        if not os.path.exists(device_path):
            rospy.logerr(f"Camera device {device_path} does not exist")
            # 尝试寻找其他可用摄像头
            for i in range(10):  # 检查video0-video9
                alt_device = f"/dev/video{i}"
                if os.path.exists(alt_device):
                    rospy.logwarn(f"Found alternative camera device: {alt_device}")
                    self.camera_index = i
                    break
            else:
                rospy.logerr("No camera devices found")
                return False
        
        # 最多尝试3次打开摄像头
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                # 确保之前的实例已关闭
                if hasattr(self.camera, 'camera') and self.camera.camera is not None:
                    self.camera.camera.release()
                    time.sleep(0.5)  # 给系统一些时间完全释放摄像头
                
                # 打开摄像头
                rospy.loginfo(f"Attempting to open camera {self.camera_index} (attempt {attempt+1}/{max_attempts})")
                self.camera.camera = cv2.VideoCapture(self.camera_index)
                
                # 设置MJPEG格式
                self.camera.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                self.camera.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.camera.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.camera.camera.set(cv2.CAP_PROP_FPS, self.fps)
                
                # 读取一帧确认摄像头是否可用
                for _ in range(5):  # 最多尝试读取5次
                    ret, frame = self.camera.camera.read()
                    if ret and frame is not None and frame.size > 0:
                        break
                    time.sleep(0.1)
                
                # 检查摄像头是否打开
                self.camera.camera_open = (ret and frame is not None and frame.size > 0)
                
                if self.camera.camera_open:
                    rospy.loginfo(f"Stereo camera opened successfully with MJPEG format: {self.width}x{self.height}")
                    # 启动定时器发布图像
                    self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
                    rospy.loginfo(f"Camera started with publish rate {self.publish_rate} Hz")
                    return True
                else:
                    rospy.logwarn(f"Failed to read from camera {self.camera_index} on attempt {attempt+1}")
                    if self.camera.camera.isOpened():
                        self.camera.camera.release()
                    time.sleep(1.0)  # 等待1秒后重试
            except Exception as e:
                rospy.logerr(f"Error opening camera: {str(e)}")
                if hasattr(self.camera, 'camera') and self.camera.camera is not None:
                    if self.camera.camera.isOpened():
                        self.camera.camera.release()
                time.sleep(1.0)  # 等待1秒后重试
        
        rospy.logerr(f"Failed to open camera {self.camera_index} after {max_attempts} attempts")
        
        # 创建模拟图像，这样即使没有摄像头也能继续运行
        self._create_dummy_image()
        return False
    
    def stop_camera(self):
        """Stop camera and publishing timer"""
        # Stop timer
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None
        
        # Close camera
        self.camera.close_camera()
        
        rospy.loginfo("Camera stopped")
    
    def timer_callback(self, event):
        """Handle timer event for publishing images"""
        if not self.camera_enabled:
            return
        
        try:
            # Capture frame
            ret, frame = self.camera.camera.read()
            
            if ret:
                timestamp = rospy.Time.now()
                
                # Split the stereo frame into left and right images
                image_width = self.width // 2
                left_image = frame[:, :image_width]
                right_image = frame[:, image_width:]
                
                # Convert OpenCV images to ROS image messages
                left_msg = self.bridge.cv2_to_imgmsg(left_image, "bgr8")
                right_msg = self.bridge.cv2_to_imgmsg(right_image, "bgr8")
                
                # Set headers
                left_msg.header.stamp = timestamp
                left_msg.header.frame_id = self.frame_id + "_left"
                right_msg.header.stamp = timestamp
                right_msg.header.frame_id = self.frame_id + "_right"
                
                # Update camera info timestamps
                left_camera_info = self.create_camera_info_msg()
                left_camera_info.header.stamp = timestamp
                left_camera_info.header.frame_id = self.frame_id + "_left"
                left_camera_info.width = image_width
                
                right_camera_info = self.create_camera_info_msg()
                right_camera_info.header.stamp = timestamp
                right_camera_info.header.frame_id = self.frame_id + "_right"
                right_camera_info.width = image_width
                
                # Publish images and camera info
                self.left_image_pub.publish(left_msg)
                self.right_image_pub.publish(right_msg)
                self.left_camera_info_pub.publish(left_camera_info)
                self.right_camera_info_pub.publish(right_camera_info)
                
                # Also publish original combined image
                combined_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                combined_msg.header.stamp = timestamp
                combined_msg.header.frame_id = self.frame_id
                self.image_pub.publish(combined_msg)
                
                # Update camera info timestamp
                self.camera_info_msg.header.stamp = timestamp
                self.camera_info_pub.publish(self.camera_info_msg)
        
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Error in timer callback: {str(e)}")
    
    def create_camera_info_msg(self):
        """
        Create camera info message with default values
        
        Returns:
            camera_info: CameraInfo message
        """
        camera_info = CameraInfo()
        
        # Set header
        camera_info.header.frame_id = self.frame_id
        
        # Set image size
        camera_info.height = self.height
        camera_info.width = self.width
        
        # Set distortion model
        camera_info.distortion_model = "plumb_bob"
        
        # Set default camera matrix (focal length = width/2, center = width/2, height/2)
        fx = self.width / 2.0
        fy = self.width / 2.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        
        # Set default rectification matrix (identity)
        camera_info.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        
        # Set default projection matrix
        camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        
        # Set default distortion coefficients
        camera_info.D = [0, 0, 0, 0, 0]
        
        return camera_info
    
    def _create_dummy_image(self):
        """创建模拟图像，当摄像头无法使用时使用"""
        # 创建模拟左右两侧图像
        image_width = self.width // 2
        image_height = self.height
        
        # 创建一个灰色背景
        dummy_left = np.ones((image_height, image_width, 3), dtype=np.uint8) * 128
        dummy_right = np.ones((image_height, image_width, 3), dtype=np.uint8) * 128
        
        # 在左图上添加文字
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(dummy_left, "Camera Not Available", (20, image_height//2 - 20), 
                    font, 0.8, (0, 0, 255), 2)
        cv2.putText(dummy_left, "Left View", (20, image_height//2 + 20), 
                    font, 0.8, (255, 0, 0), 2)
        
        # 在右图上添加文字
        cv2.putText(dummy_right, "Camera Not Available", (20, image_height//2 - 20), 
                    font, 0.8, (0, 0, 255), 2)
        cv2.putText(dummy_right, "Right View", (20, image_height//2 + 20), 
                    font, 0.8, (0, 255, 0), 2)
        
        # 合并成一个完整的双目图像
        self.dummy_image = np.hstack([dummy_left, dummy_right])
        
        # 创建模拟定时器来发布图像
        self.camera.camera_open = False  # 标记摄像头实际未打开
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.dummy_timer_callback)
        rospy.loginfo("Created dummy stereo image and started publishing")
    
    def dummy_timer_callback(self, event):
        """定时发布模拟图像的回调函数"""
        if not hasattr(self, 'dummy_image') or self.dummy_image is None:
            return
            
        try:
            timestamp = rospy.Time.now()
            
            # 分割左右图像
            image_width = self.width // 2
            left_image = self.dummy_image[:, :image_width]
            right_image = self.dummy_image[:, image_width:]
            
            # 转换为ROS消息
            left_msg = self.bridge.cv2_to_imgmsg(left_image, "bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(right_image, "bgr8")
            
            # 设置消息头
            left_msg.header.stamp = timestamp
            left_msg.header.frame_id = self.frame_id + "_left"
            right_msg.header.stamp = timestamp
            right_msg.header.frame_id = self.frame_id + "_right"
            
            # 更新相机信息时间戳
            left_camera_info = self.create_camera_info_msg()
            left_camera_info.header.stamp = timestamp
            left_camera_info.header.frame_id = self.frame_id + "_left"
            left_camera_info.width = image_width
            
            right_camera_info = self.create_camera_info_msg()
            right_camera_info.header.stamp = timestamp
            right_camera_info.header.frame_id = self.frame_id + "_right"
            right_camera_info.width = image_width
            
            # 发布图像和相机信息
            self.left_image_pub.publish(left_msg)
            self.right_image_pub.publish(right_msg)
            self.left_camera_info_pub.publish(left_camera_info)
            self.right_camera_info_pub.publish(right_camera_info)
            
            # 同时发布原始合并图像
            combined_msg = self.bridge.cv2_to_imgmsg(self.dummy_image, "bgr8")
            combined_msg.header.stamp = timestamp
            combined_msg.header.frame_id = self.frame_id
            self.image_pub.publish(combined_msg)
            
            # 更新相机信息时间戳
            self.camera_info_msg.header.stamp = timestamp
            self.camera_info_pub.publish(self.camera_info_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error in dummy timer: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Error in dummy timer callback: {str(e)}")
    
    def shutdown(self):
        """Shutdown node"""
        self.stop_camera()
        rospy.loginfo("Camera node shutdown")

def main():
    """Main function"""
    try:
        # Create camera node
        camera_node = CameraNode()
        
        # Register shutdown hook
        rospy.on_shutdown(camera_node.shutdown)
        
        # Process callbacks
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 