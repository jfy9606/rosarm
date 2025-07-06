#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32, Float32
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
        self.left_image_topic = rospy.get_param('~left_image_topic', '/stereo_camera/left/image_raw')
        self.right_image_topic = rospy.get_param('~right_image_topic', '/stereo_camera/right/image_raw')
        self.detection_topic = rospy.get_param('~detection_topic', '/detections/poses')
        self.detection_3d_topic = rospy.get_param('~detection_3d_topic', '/detections/poses_3d')
        self.disparity_topic = rospy.get_param('~disparity_topic', '/stereo/disparity')
        self.initial_view_mode = rospy.get_param('~view_mode', 0)  # Default to NORMAL view
        self.current_view = rospy.get_param('~current_view', 'left')  # 'left' or 'right'
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Initialize view control
        self.view_control = ViewControl()
        self.view_control.set_view_mode(self.initial_view_mode)
        
        # Create services
        self.set_view_mode_service = rospy.Service('/view/set_view_mode', SetBool, self.set_view_mode_callback)
        self.switch_view_service = rospy.Service('/view/switch_view', SetBool, self.switch_view_callback)
        
        # Subscribe to image topics
        self.left_image_sub = rospy.Subscriber(self.left_image_topic, Image, self.left_image_callback)
        self.right_image_sub = rospy.Subscriber(self.right_image_topic, Image, self.right_image_callback)
        self.combined_image_sub = rospy.Subscriber(self.image_topic, Image, self.combined_image_callback)
        rospy.loginfo(f"Subscribed to stereo image topics")
        
        # Subscribe to detection topics
        self.detection_sub = rospy.Subscriber(self.detection_topic, PoseArray, self.detection_callback)
        self.detection_3d_sub = rospy.Subscriber(self.detection_3d_topic, PoseArray, self.detection_3d_callback)
        
        # Subscribe to disparity topic
        self.disparity_sub = rospy.Subscriber(self.disparity_topic, Image, self.disparity_callback)
        
        # Subscribe to view mode topic
        self.mode_sub = rospy.Subscriber('/view/mode', Int32, self.mode_callback)
        
        # Create publishers
        self.view_pub = rospy.Publisher('/view/image', Image, queue_size=10)
        self.mode_pub = rospy.Publisher('/view/mode', Int32, queue_size=10, latch=True)
        self.current_view_pub = rospy.Publisher('/view/current_view', Int32, queue_size=10, latch=True)
        
        # Initialize images
        self.left_image = None
        self.right_image = None
        self.combined_image = None
        self.disparity_image = None
        self.latest_images_timestamp = rospy.Time(0)
        
        # Initialize detections
        self.latest_detections = []
        self.detections_timestamp = rospy.Time(0)
        
        # Initialize 3D detections
        self.latest_3d_detections = []
        self.detections_3d_timestamp = rospy.Time(0)
        
        # Initialize info
        self.info = {
            'Resolution': 'Unknown',
            'FPS': 0.0,
            'Detections': 0,
            'View': 'Left',
            'Distance': 'N/A'
        }
        
        # Initialize frame counter for FPS calculation
        self.frame_count = 0
        self.last_fps_time = rospy.Time.now()
        self.fps = 0.0
        
        # Create timer for publishing processed images
        self.timer = rospy.Timer(rospy.Duration(1.0/30.0), self.timer_callback)
        
        # Publish initial view mode
        self.publish_view_mode()
        self.publish_current_view()
        
        rospy.loginfo(f"View node initialized with stereo support")
    
    def publish_view_mode(self):
        """Publish current view mode"""
        mode_msg = Int32()
        mode_msg.data = self.view_control.view_mode.value
        self.mode_pub.publish(mode_msg)
    
    def publish_current_view(self):
        """Publish current view (left/right)"""
        view_msg = Int32()
        view_msg.data = 0 if self.current_view == 'left' else 1
        self.current_view_pub.publish(view_msg)
    
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
    
    def switch_view_callback(self, req):
        """Handle switch view service request (switch between left and right)"""
        try:
            # Toggle view
            if req.data:
                self.current_view = 'right' if self.current_view == 'left' else 'left'
            else:
                # Explicit set
                self.current_view = 'right' if req.data else 'left'
            
            # Update info
            self.info['View'] = self.current_view.capitalize()
            
            # Publish updated view
            self.publish_current_view()
            
            return SetBoolResponse(True, f"View switched to {self.current_view}")
        except Exception as e:
            rospy.logerr(f"Error switching view: {str(e)}")
            return SetBoolResponse(False, f"Error switching view: {str(e)}")
    
    def mode_callback(self, msg):
        """Handle view mode message"""
        mode = msg.data
        self.view_control.set_view_mode(mode)
        rospy.loginfo(f"View mode changed to: {self.view_control.view_mode.name}")
    
    def left_image_callback(self, msg):
        """Handle left camera image message"""
        try:
            # Convert ROS image message to OpenCV format
            self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_images_timestamp = msg.header.stamp
        except CvBridgeError as e:
            rospy.logerr(f"Left image CV Bridge error: {str(e)}")
    
    def right_image_callback(self, msg):
        """Handle right camera image message"""
        try:
            # Convert ROS image message to OpenCV format
            self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_images_timestamp = msg.header.stamp
        except CvBridgeError as e:
            rospy.logerr(f"Right image CV Bridge error: {str(e)}")
    
    def combined_image_callback(self, msg):
        """Handle combined stereo image message"""
        try:
            # Convert ROS image message to OpenCV format
            self.combined_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Update resolution info
            if self.combined_image is not None:
                height, width = self.combined_image.shape[:2]
                self.info['Resolution'] = f"{width}x{height}"
        except CvBridgeError as e:
            rospy.logerr(f"Combined image CV Bridge error: {str(e)}")
    
    def disparity_callback(self, msg):
        """Handle disparity image message"""
        try:
            # Convert ROS image message to OpenCV format
            self.disparity_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            rospy.logerr(f"Disparity image CV Bridge error: {str(e)}")
    
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
    
    def detection_3d_callback(self, msg):
        """Handle 3D detection poses message"""
        # Store 3D detection poses
        self.latest_3d_detections = msg.poses
        self.detections_3d_timestamp = msg.header.stamp
        
        # Update distance info if we have detections
        if len(msg.poses) > 0:
            # Use the first detection's Z value as the distance
            distance = msg.poses[0].position.z
            self.info['Distance'] = f"{distance:.2f}m"
        else:
            self.info['Distance'] = "N/A"
    
    def timer_callback(self, event):
        """Handle timer event for publishing processed images"""
        # Update FPS
        self.frame_count += 1
        current_time = rospy.Time.now()
        time_diff = (current_time - self.last_fps_time).to_sec()
        
        if time_diff >= 1.0:
            self.fps = self.frame_count / time_diff
            self.frame_count = 0
            self.last_fps_time = current_time
        
        self.info['FPS'] = f"{self.fps:.1f}"
        
        # Process and publish current view
        self.process_and_publish_image()
    
    def process_and_publish_image(self):
        """Process and publish the current view image"""
        # Select current image based on view
        current_image = None
        if self.current_view == 'left' and self.left_image is not None:
            current_image = self.left_image.copy()
        elif self.current_view == 'right' and self.right_image is not None:
            current_image = self.right_image.copy()
        elif self.combined_image is not None:
            # Fallback to combined image
            current_image = self.combined_image.copy()
        
        if current_image is None:
            return
        
        # Check if detections are recent enough (within last 0.5 seconds)
        detections_age = (rospy.Time.now() - self.detections_timestamp).to_sec()
        detections = self.latest_detections if detections_age < 0.5 else []
        
        # Adjust detection coordinates for the current view
        if self.current_view == 'left' or self.current_view == 'right':
            # Adjust coordinates for single view (half width)
            adjusted_detections = []
            single_width = self.combined_image.shape[1] // 2 if self.combined_image is not None else 0
            
            for det in detections:
                x1, y1, x2, y2, conf, class_id = det
                
                # Check if the detection is in the current view
                if (self.current_view == 'left' and x1 < single_width) or \
                   (self.current_view == 'right' and x1 >= single_width):
                    # Adjust x coordinates for right view
                    if self.current_view == 'right' and single_width > 0:
                        x1 -= single_width
                        x2 -= single_width
                    
                    adjusted_detections.append([x1, y1, x2, y2, conf, class_id])
            
            detections = adjusted_detections
        
        # Process image with detections and info
        try:
            processed_image = self.view_control.process_image(
                current_image,
                detections=detections,
                info=self.info
            )
            
            # Add distance overlay to processed image
            if len(self.latest_3d_detections) > 0:
                detections_3d_age = (rospy.Time.now() - self.detections_3d_timestamp).to_sec()
                if detections_3d_age < 0.5:
                    # Display distance information on the image
                    for i, det_3d in enumerate(self.latest_3d_detections):
                        if i >= len(detections):
                            continue
                        
                        x1, y1, x2, y2, _, _ = detections[i]
                        distance = det_3d.position.z
                        
                        # Draw distance text
                        cv2.putText(
                            processed_image,
                            f"{distance:.2f}m",
                            (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 255),  # Yellow
                            2
                        )
            
            # Convert OpenCV image back to ROS image message
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            processed_msg.header.stamp = rospy.Time.now()
            processed_msg.header.frame_id = f"camera_{self.current_view}"
            
            # Publish processed image
            self.view_pub.publish(processed_msg)
            
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