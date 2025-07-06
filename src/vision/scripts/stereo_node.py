#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import os
import sys
import time
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
from vision.srv import SetViewMode, SetViewModeResponse
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Point, PointStamped

# Add current directory to path to find modules
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

# Import stereo control
from stereo_control import StereoControl, ViewMode as StereoViewMode

class StereoNode:
    """
    ROS node for stereo vision processing
    """
    
    def __init__(self):
        """Initialize the stereo node"""
        rospy.init_node('stereo_node', anonymous=True)
        
        # Initialize parameters
        self.left_image_topic = rospy.get_param('~left_image_topic', '/stereo_camera/left/image_raw')
        self.right_image_topic = rospy.get_param('~right_image_topic', '/stereo_camera/right/image_raw')
        self.left_camera_info_topic = rospy.get_param('~left_camera_info_topic', '/stereo_camera/left/camera_info')
        self.right_camera_info_topic = rospy.get_param('~right_camera_info_topic', '/stereo_camera/right/camera_info')
        self.initial_view_mode = rospy.get_param('~view_mode', 0)  # Default to LEFT view
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize stereo control
        self.stereo_control = StereoControl()
        self.stereo_control.set_view_mode(self.initial_view_mode)
        
        # Initialize calibration status
        self.calibration_received = False
        
        # Create service
        self.set_view_mode_service = rospy.Service('/stereo/set_view_mode', SetViewMode, self.set_view_mode_callback)
        
        # Subscribe to camera info topics
        self.left_camera_info_sub = rospy.Subscriber(self.left_camera_info_topic, CameraInfo, self.left_camera_info_callback)
        self.right_camera_info_sub = rospy.Subscriber(self.right_camera_info_topic, CameraInfo, self.right_camera_info_callback)
        
        # Create synchronized subscribers for stereo images
        self.left_image_sub = message_filters.Subscriber(self.left_image_topic, Image)
        self.right_image_sub = message_filters.Subscriber(self.right_image_topic, Image)
        
        # Create synchronizer
        self.ts = message_filters.TimeSynchronizer([self.left_image_sub, self.right_image_sub], 10)
        self.ts.registerCallback(self.stereo_callback)
        
        # Create publishers
        self.stereo_pub = rospy.Publisher('/stereo/image', Image, queue_size=10)
        self.disparity_pub = rospy.Publisher('/stereo/disparity', Image, queue_size=10)
        self.view_mode_pub = rospy.Publisher('/stereo/view_mode', Int32, queue_size=10, latch=True)
        
        # Subscribe to detections
        self.detections_sub = rospy.Subscriber('/detections/poses', PoseArray, self.detections_callback)
        
        # Create 3D detections publisher
        self.detections_3d_pub = rospy.Publisher('/detections/poses_3d', PoseArray, queue_size=10)
        
        # Initialize camera matrices
        self.camera_matrix_left = None
        self.camera_matrix_right = None
        self.dist_coeffs_left = None
        self.dist_coeffs_right = None
        self.R = None
        self.T = None
        self.E = None
        self.F = None
        self.image_size = None
        
        # Publish initial view mode
        self.publish_view_mode()
        
        rospy.loginfo(f"Stereo node initialized")
    
    def publish_view_mode(self):
        """Publish current view mode"""
        mode_msg = Int32()
        mode_msg.data = self.stereo_control.view_mode.value
        self.view_mode_pub.publish(mode_msg)
    
    def set_view_mode_callback(self, req):
        """Handle set view mode service request"""
        try:
            # Set view mode
            self.stereo_control.set_view_mode(req.mode)
            
            # Publish updated view mode
            self.publish_view_mode()
            
            return SetViewModeResponse(True, f"View mode set to {self.stereo_control.view_mode.name}")
        except Exception as e:
            rospy.logerr(f"Error setting view mode: {str(e)}")
            return SetViewModeResponse(False, f"Error setting view mode: {str(e)}")
    
    def left_camera_info_callback(self, msg):
        """Handle left camera info message"""
        if self.camera_matrix_left is None:
            # Extract camera matrix and distortion coefficients
            self.camera_matrix_left = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs_left = np.array(msg.D)
            
            # Get image size
            self.image_size = (msg.width, msg.height)
            
            # Check if we have all calibration parameters
            self.check_calibration()
    
    def right_camera_info_callback(self, msg):
        """Handle right camera info message"""
        if self.camera_matrix_right is None:
            # Extract camera matrix and distortion coefficients
            self.camera_matrix_right = np.array(msg.K).reshape(3, 3)
            self.dist_coeffs_right = np.array(msg.D)
            
            # Get rectification parameters
            self.R = np.array(msg.R).reshape(3, 3)
            self.P = np.array(msg.P).reshape(3, 4)
            
            # Extract translation vector from projection matrix
            self.T = self.P[:3, 3]
            
            # Compute essential and fundamental matrices
            self.E = np.dot(np.dot(self.R.T, np.cross(np.eye(3), self.T)), self.camera_matrix_right)
            self.F = np.dot(np.dot(np.linalg.inv(self.camera_matrix_left).T, self.E), np.linalg.inv(self.camera_matrix_right))
            
            # Check if we have all calibration parameters
            self.check_calibration()
    
    def check_calibration(self):
        """Check if all calibration parameters are available and initialize stereo rectification"""
        if (self.camera_matrix_left is not None and
            self.camera_matrix_right is not None and
            self.dist_coeffs_left is not None and
            self.dist_coeffs_right is not None and
            self.R is not None and
            self.T is not None and
            self.E is not None and
            self.F is not None and
            self.image_size is not None and
            not self.calibration_received):
            
            # Set calibration parameters
            self.stereo_control.set_calibration_params(
                self.camera_matrix_left, self.camera_matrix_right,
                self.dist_coeffs_left, self.dist_coeffs_right,
                self.R, self.T, self.E, self.F,
                self.image_size
            )
            
            self.calibration_received = True
            rospy.loginfo("Stereo calibration parameters received")
    
    def stereo_callback(self, left_msg, right_msg):
        """Handle stereo image messages"""
        try:
            # Convert ROS image messages to OpenCV format
            left_image = self.bridge.imgmsg_to_cv2(left_msg, "bgr8")
            right_image = self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
            
            # Process stereo images
            output_image, disparity = self.stereo_control.process_stereo_images(left_image, right_image)
            
            # Convert OpenCV images back to ROS image messages
            output_msg = self.bridge.cv2_to_imgmsg(output_image, "bgr8")
            output_msg.header = left_msg.header
            
            # Publish stereo output image
            self.stereo_pub.publish(output_msg)
            
            # Publish disparity if available
            if disparity is not None:
                disparity_msg = self.bridge.cv2_to_imgmsg(disparity, "mono8")
                disparity_msg.header = left_msg.header
                self.disparity_pub.publish(disparity_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Error processing stereo images: {str(e)}")
    
    def detections_callback(self, msg):
        """Handle detection poses message"""
        if not self.calibration_received or self.stereo_control.view_mode != StereoViewMode.DEPTH:
            # Skip if calibration not received or not in depth mode
            return
        
        try:
            # Get latest disparity
            disparity = rospy.wait_for_message('/stereo/disparity', Image, timeout=1.0)
            disparity_cv = self.bridge.imgmsg_to_cv2(disparity, "mono8")
            
            # Create 3D pose array
            poses_3d = PoseArray()
            poses_3d.header = msg.header
            
            # Process each detection
            for pose in msg.poses:
                # Get 2D coordinates from pose
                x = int(pose.position.x * 100.0)  # Convert back to pixels
                y = int(pose.position.y * 100.0)
                
                # Get 3D point from disparity
                point_3d = self.stereo_control.get_3d_point(disparity_cv, x, y)
                
                if point_3d is not None:
                    # Create 3D pose
                    pose_3d = Pose()
                    
                    # Set position
                    pose_3d.position.x = point_3d[0]
                    pose_3d.position.y = point_3d[1]
                    pose_3d.position.z = point_3d[2]
                    
                    # Copy orientation (class ID and confidence)
                    pose_3d.orientation = pose.orientation
                    
                    # Add to pose array
                    poses_3d.poses.append(pose_3d)
            
            # Publish 3D poses
            self.detections_3d_pub.publish(poses_3d)
            
        except rospy.ROSException as e:
            # Timeout waiting for disparity
            pass
        except Exception as e:
            rospy.logerr(f"Error processing detections: {str(e)}")

def main():
    """Main function"""
    try:
        # Create stereo node
        stereo = StereoNode()
        
        # Process callbacks
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 