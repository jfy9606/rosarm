#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from vision.msg import Detection, ObjectDetection

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('detection_topic', '/vision/detections')
        self.declare_parameter('object_topic', '/vision/objects')
        self.declare_parameter('debug_image', True)
        
        # Get parameters
        camera_topic = self.get_parameter('camera_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        object_topic = self.get_parameter('object_topic').value
        self.debug_image = self.get_parameter('debug_image').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize publishers and subscribers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10)
        
        self.detection_pub = self.create_publisher(
            Detection,
            detection_topic,
            10)
        
        self.object_pub = self.create_publisher(
            ObjectDetection,
            object_topic,
            10)
            
        if self.debug_image:
            self.debug_pub = self.create_publisher(
                Image,
                '/vision/debug_image',
                10)
        
        self.get_logger().info('Vision node initialized')
        
    def camera_info_callback(self, msg):
        # Process camera info (calibration data)
        self.camera_info = msg
        
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
            
        # Process the image (placeholder for actual vision processing)
        detections = self.process_image(cv_image)
        
        # Publish results
        for detection in detections:
            self.detection_pub.publish(detection)
            
            # Create 3D object detection (placeholder)
            obj_msg = ObjectDetection()
            obj_msg.header = msg.header
            obj_msg.class_name = detection.class_name
            obj_msg.confidence = detection.confidence
            obj_msg.x_min = float(detection.x_min)
            obj_msg.y_min = float(detection.y_min)
            obj_msg.x_max = float(detection.x_max)
            obj_msg.y_max = float(detection.y_max)
            
            # Placeholder for 3D position calculation
            obj_msg.x_3d = 0.0
            obj_msg.y_3d = 0.0
            obj_msg.z_3d = 0.0
            obj_msg.distance = 0.0
            
            self.object_pub.publish(obj_msg)
            
        # Publish debug image if enabled
        if self.debug_image and detections:
            debug_img = cv_image.copy()
            for detection in detections:
                # Draw bounding box
                cv2.rectangle(debug_img, 
                             (detection.x_min, detection.y_min),
                             (detection.x_max, detection.y_max),
                             (0, 255, 0), 2)
                             
                # Draw label
                label = f"{detection.class_name}: {detection.confidence:.2f}"
                cv2.putText(debug_img, label, 
                           (detection.x_min, detection.y_min - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                           
            # Publish debug image
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
            
    def process_image(self, image):
        # Placeholder for actual vision processing
        # This should be replaced with actual object detection code
        
        # Create a sample detection
        detection = Detection()
        detection.class_id = 1
        detection.class_name = "sample_object"
        detection.confidence = 0.95
        
        # Set bounding box (placeholder values)
        height, width, _ = image.shape
        detection.x_min = int(width * 0.25)
        detection.y_min = int(height * 0.25)
        detection.x_max = int(width * 0.75)
        detection.y_max = int(height * 0.75)
        
        # Calculate center and dimensions
        detection.center_x = (detection.x_min + detection.x_max) / 2.0
        detection.center_y = (detection.y_min + detection.y_max) / 2.0
        detection.width = detection.x_max - detection.x_min
        detection.height = detection.y_max - detection.y_min
        
        return [detection]

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 