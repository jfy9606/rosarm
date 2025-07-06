#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import os
import sys
import time
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse

# 尝试导入ultralytics
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    rospy.logwarn("无法导入ultralytics，将使用模拟检测")

# 导入detector_control
from detector_control import DetectorControl

class DetectorNode:
    """
    ROS node for object detection using YOLO
    """
    
    def __init__(self):
        """Initialize the detector node"""
        rospy.init_node('detector_node', anonymous=True)
        
        # Initialize parameters
        self.model_path = rospy.get_param('~model_path', 'yolo11n.pt')
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.25)
        self.image_topic = rospy.get_param('~image_topic', '/stereo_camera/image_raw')
        self.detection_enabled = rospy.get_param('~detection_enabled', True)
        self.auto_download = rospy.get_param('~auto_download', True)
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize detector control
        self.detector = DetectorControl(
            model_path=self.model_path,
            conf_threshold=self.conf_threshold,
            auto_download=self.auto_download
        )
        
        # Create service
        self.enable_service = rospy.Service('/detector_node/enable', SetBool, self.enable_detection_callback)
        
        # Subscribe to image topic
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.loginfo(f"Subscribed to image topic: {self.image_topic}")
        
        # Subscribe to control topic
        self.control_sub = rospy.Subscriber('/detector/status', Bool, self.control_callback)
        
        # Create detection result publishers
        self.detection_pub = rospy.Publisher('/detections/image', Image, queue_size=10)
        self.poses_pub = rospy.Publisher('/detections/poses', PoseArray, queue_size=10)
        self.status_pub = rospy.Publisher('/detector/status', Bool, queue_size=10, latch=True)
        
        # Publish initial status
        self.publish_status()
        
        # Create timer for periodically publishing status
        rospy.Timer(rospy.Duration(5), self.publish_status_timer)
        
        rospy.loginfo(f"Object detector node initialized")
    
    def enable_detection_callback(self, req):
        """Handle enable/disable detection service request"""
        self.detection_enabled = req.data
        rospy.loginfo(f"Detection status changed to: {'enabled' if self.detection_enabled else 'disabled'}")
        
        # Publish status update
        self.publish_status()
        
        return SetBoolResponse(True, f"Detection {'enabled' if self.detection_enabled else 'disabled'}")
    
    def publish_status_timer(self, event):
        """Periodically publish status"""
        self.publish_status()
    
    def publish_status(self):
        """Publish current detection status"""
        status_msg = Bool()
        status_msg.data = self.detection_enabled and self.detector.model_loaded
        self.status_pub.publish(status_msg)
    
    def control_callback(self, msg):
        """Handle control message"""
        enabled = msg.data
        if enabled != self.detection_enabled:
            self.detection_enabled = enabled
            rospy.loginfo(f"Detection status changed to: {'enabled' if self.detection_enabled else 'disabled'}")
    
    def image_callback(self, msg):
        """Handle image message"""
        if not self.detector.model_loaded or not self.detection_enabled:
            # If model not loaded or detection disabled, forward original image
            self.detection_pub.publish(msg)
            return
        
        try:
            # Process image and publish results
            self.detect_and_publish(msg)
        except Exception as e:
            # Throttle error messages
            current_time = rospy.Time.now()
            if not hasattr(self, 'last_error_time') or (current_time - self.last_error_time).to_sec() > 10.0:
                rospy.logerr(f"Error processing image: {str(e)}")
                self.last_error_time = current_time
    
    def detect_and_publish(self, image_msg):
        """
        Detect objects in image and publish results
        
        Args:
            image_msg: ROS image message
        """
        try:
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            # Detect objects
            detections = self.detector.detect_objects(cv_image)
            
            # Draw detections on image
            annotated_image = self.detector.draw_detections(cv_image, detections)
            
            # Convert OpenCV image back to ROS image message
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            detection_msg.header = image_msg.header
            
            # Publish detection result image
            self.detection_pub.publish(detection_msg)
            
            # Publish object poses
            self.publish_poses(detections, image_msg.header)
            
        except Exception as e:
            rospy.logerr(f"Error in detection and publishing: {str(e)}")
    
    def publish_poses(self, detections, header):
        """
        Publish detected object poses
        
        Args:
            detections: Detection results list
            header: Image message header for timestamp and frame ID
        """
        # Create pose array message
        pose_array = PoseArray()
        pose_array.header = header
        
        # Iterate through detection results
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            
            # Calculate rectangle center coordinates
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            
            # Create pose message
            pose = Pose()
            
            # Set position - assuming depth of 1 meter, actual depth will be replaced by stereo_node
            pose.position.x = center_x / 100.0  # Convert to meters (assuming image units are pixels)
            pose.position.y = center_y / 100.0
            pose.position.z = 1.0  # Default depth 1 meter
            
            # Store class ID and confidence in orientation field for transmission
            pose.orientation.x = float(class_id)  # Class ID
            pose.orientation.y = float(conf)      # Confidence
            pose.orientation.z = float(y2 - y1)   # Height
            pose.orientation.w = 1.0
            
            # Add pose to array
            pose_array.poses.append(pose)
            
        # Publish pose array
        self.poses_pub.publish(pose_array)

def main():
    """Main function"""
    try:
        # Create detector node
        detector = DetectorNode()
        
        # Process callbacks
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 