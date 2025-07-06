#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse

# 使用包导入
from vision import CameraControl, CameraMode

class CameraNode:
    """
    ROS node for camera control and image publishing
    """
    
    def __init__(self):
        """Initialize the camera node"""
        rospy.init_node('camera_node', anonymous=True)
        
        # Initialize parameters
        self.camera_index = rospy.get_param('~camera_index', 0)
        self.width = rospy.get_param('~width', 640)
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
        # Open camera
        success = self.camera.open_camera()
        
        if success:
            # Start timer for publishing images
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
            rospy.loginfo(f"Camera started with publish rate {self.publish_rate} Hz")
        
        return success
    
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
            frame, timestamp = self.camera.capture_frame()
            
            if frame is not None:
                # Convert OpenCV image to ROS image message
                if len(frame.shape) == 3:
                    # Color image
                    image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                else:
                    # Grayscale image
                    image_msg = self.bridge.cv2_to_imgmsg(frame, "mono8")
                
                # Set header
                image_msg.header.stamp = timestamp
                image_msg.header.frame_id = self.frame_id
                
                # Update camera info timestamp
                self.camera_info_msg.header.stamp = timestamp
                
                # Publish image and camera info
                self.image_pub.publish(image_msg)
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