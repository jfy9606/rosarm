#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import SetBool, SetBoolResponse

# 使用包导入
from vision import ViewControl, DisplayViewMode as ViewMode

class ViewNode:
    """
    ROS node for image view and visualization
    """
    
    def __init__(self):
        """Initialize the view node"""
        rospy.init_node('view_node', anonymous=True)
        
        # Initialize parameters
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.detection_topic = rospy.get_param('~detection_topic', '/detections/poses')
        self.initial_view_mode = rospy.get_param('~view_mode', 0)  # Default to NORMAL view
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize view control
        self.view_control = ViewControl()
        self.view_control.set_view_mode(self.initial_view_mode)
        
        # Create service
        self.set_view_mode_service = rospy.Service('/view/set_view_mode', SetBool, self.set_view_mode_callback)
        
        # Subscribe to image topic
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        rospy.loginfo(f"Subscribed to image topic: {self.image_topic}")
        
        # Subscribe to detection topic
        self.detection_sub = rospy.Subscriber(self.detection_topic, PoseArray, self.detection_callback)
        
        # Subscribe to view mode topic
        self.mode_sub = rospy.Subscriber('/view/mode', Int32, self.mode_callback)
        
        # Create publishers
        self.view_pub = rospy.Publisher('/view/image', Image, queue_size=10)
        self.mode_pub = rospy.Publisher('/view/mode', Int32, queue_size=10, latch=True)
        
        # Initialize detections
        self.latest_detections = []
        self.detections_timestamp = rospy.Time(0)
        
        # Initialize info
        self.info = {
            'Resolution': 'Unknown',
            'FPS': 0.0,
            'Detections': 0
        }
        
        # Initialize frame counter for FPS calculation
        self.frame_count = 0
        self.last_fps_time = rospy.Time.now()
        self.fps = 0.0
        
        # Publish initial view mode
        self.publish_view_mode()
        
        rospy.loginfo(f"View node initialized")
    
    def publish_view_mode(self):
        """Publish current view mode"""
        mode_msg = Int32()
        mode_msg.data = self.view_control.view_mode.value
        self.mode_pub.publish(mode_msg)
    
    def set_view_mode_callback(self, req):
        """Handle set view mode service request"""
        try:
            # Set view mode
            self.view_control.set_view_mode(req.data)
            
            # Publish updated view mode
            self.publish_view_mode()
            
            return SetBoolResponse(True, f"View mode set to {self.view_control.view_mode.name}")
        except Exception as e:
            rospy.logerr(f"Error setting view mode: {str(e)}")
            return SetBoolResponse(False, f"Error setting view mode: {str(e)}")
    
    def mode_callback(self, msg):
        """Handle view mode message"""
        mode = msg.data
        self.view_control.set_view_mode(mode)
        rospy.loginfo(f"View mode changed to: {self.view_control.view_mode.name}")
    
    def detection_callback(self, msg):
        """Handle detection poses message"""
        # Convert PoseArray to detection format [x1, y1, x2, y2, conf, class_id]
        detections = []
        
        for pose in msg.poses:
            # Get position (center of detection)
            center_x = pose.position.x * 100.0  # Convert from meters to pixels
            center_y = pose.position.y * 100.0
            
            # Get class ID and confidence from orientation
            class_id = int(pose.orientation.x)
            conf = float(pose.orientation.y)
            height = float(pose.orientation.z)
            
            # Calculate bounding box
            width = height * 0.75  # Assume width is 75% of height
            x1 = center_x - width / 2
            y1 = center_y - height / 2
            x2 = center_x + width / 2
            y2 = center_y + height / 2
            
            # Add to detections list
            detections.append([x1, y1, x2, y2, conf, class_id])
        
        # Update latest detections
        self.latest_detections = detections
        self.detections_timestamp = msg.header.stamp
        
        # Update info
        self.info['Detections'] = len(detections)
    
    def image_callback(self, msg):
        """Handle image message"""
        try:
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Update resolution info
            height, width = cv_image.shape[:2]
            self.info['Resolution'] = f"{width}x{height}"
            
            # Update FPS
            self.frame_count += 1
            current_time = rospy.Time.now()
            time_diff = (current_time - self.last_fps_time).to_sec()
            
            if time_diff >= 1.0:
                self.fps = self.frame_count / time_diff
                self.frame_count = 0
                self.last_fps_time = current_time
            
            self.info['FPS'] = f"{self.fps:.1f}"
            
            # Check if detections are recent enough (within last 0.5 seconds)
            detections_age = (msg.header.stamp - self.detections_timestamp).to_sec()
            detections = self.latest_detections if detections_age < 0.5 else []
            
            # Process image
            processed_image = self.view_control.process_image(
                cv_image,
                detections=detections,
                info=self.info
            )
            
            # Convert OpenCV image back to ROS image message
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            processed_msg.header = msg.header
            
            # Publish processed image
            self.view_pub.publish(processed_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

def main():
    """Main function"""
    try:
        # Create view node
        view_node = ViewNode()
        
        # Process callbacks
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 