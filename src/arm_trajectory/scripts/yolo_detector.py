#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import os
import time

class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Get parameters
        self.model_path = rospy.get_param('~model_path', '')
        self.confidence_threshold = float(rospy.get_param('~confidence', 0.5))
        
        # Initialize YOLO model
        self.init_yolo_model()
        
        # Publishers
        self.detected_poses_pub = rospy.Publisher('/stereo_vision/detected_poses', PoseArray, queue_size=1)
        self.detection_image_pub = rospy.Publisher('/yolo_detection/image', Image, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        rospy.loginfo("YOLO Detector Node initialized successfully")
    
    def init_yolo_model(self):
        """Initialize YOLO model if available, otherwise use a placeholder"""
        try:
            # Try to import the Ultralytics package for YOLOv8
            from ultralytics import YOLO
            
            # Check if model path is specified
            if self.model_path and os.path.exists(self.model_path):
                # Load custom model
                self.model = YOLO(self.model_path)
                rospy.loginfo(f"Loaded custom YOLOv8 model from {self.model_path}")
            else:
                # Use pre-trained YOLOv8n model
                self.model = YOLO("yolov8n.pt")
                rospy.loginfo("Loaded default YOLOv8n model")
            
            self.model_loaded = True
            
        except Exception as e:
            rospy.logerr(f"Failed to load YOLOv8 model: {e}")
            rospy.logwarn("Using placeholder detection method")
            self.model_loaded = False
    
    def image_callback(self, msg):
        """Process incoming image and perform object detection"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Perform object detection
            if self.model_loaded:
                detections = self.detect_objects_yolo(cv_image)
            else:
                detections = self.detect_objects_placeholder(cv_image)
            
            # Publish detections
            self.publish_detections(detections, msg.header.stamp)
            
            # Draw detections on image and publish
            result_image = self.draw_detections(cv_image, detections)
            self.detection_image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def detect_objects_yolo(self, image):
        """Detect objects using YOLOv8 model"""
        # Run inference with confidence threshold
        results = self.model.predict(image, conf=self.confidence_threshold, verbose=False)
        
        # Process results - YOLOv8 format is different from YOLOv5
        detections = []
        if len(results) > 0:
            result = results[0]  # Get first result (first image)
            
            # Extract boxes, confidences and class ids
            if hasattr(result, 'boxes') and len(result.boxes) > 0:
                for box in result.boxes:
                    # Get box coordinates (format: x1, y1, x2, y2)
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    
                    # Calculate center
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    
                    # Get confidence and class
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    
                    # Store detection info
                    detections.append({
                        'class': cls_id,
                        'confidence': conf,
                        'bbox': (x1, y1, x2, y2),
                        'center': (center_x, center_y)
                    })
        
        return detections
    
    def detect_objects_placeholder(self, image):
        """Placeholder detection method when YOLO model is not available"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Blur the image to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Threshold the image
        _, threshed = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)
        
        # Find contours
        contours, _ = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process contours
        detections = []
        for i, contour in enumerate(contours):
            # Filter small contours
            if cv2.contourArea(contour) < 500:
                continue
                
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w/2
            center_y = y + h/2
            
            # Store detection info
            detections.append({
                'class': 0,  # Default class
                'confidence': 0.8,  # Default confidence
                'bbox': (x, y, x+w, y+h),
                'center': (center_x, center_y)
            })
        
        return detections
    
    def publish_detections(self, detections, timestamp):
        """Publish detected objects as PoseArray"""
        pose_array = PoseArray()
        pose_array.header.stamp = timestamp
        pose_array.header.frame_id = "camera_frame"
        
        for det in detections:
            pose = Pose()
            # Use the image coordinates as pose position
            pose.position.x = det['center'][0]
            pose.position.y = det['center'][1]
            pose.position.z = 0.0  # No depth information from single camera
            
            # Use orientation quaternion to store class and confidence
            pose.orientation.x = float(det['class'])
            pose.orientation.y = det['confidence']
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            
            pose_array.poses.append(pose)
        
        self.detected_poses_pub.publish(pose_array)
    
    def draw_detections(self, image, detections):
        """Draw bounding boxes and labels on the image"""
        result_img = image.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            
            # Draw bounding box
            cv2.rectangle(result_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f"Class: {det['class']}, {det['confidence']:.2f}"
            cv2.putText(result_img, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return result_img

if __name__ == '__main__':
    try:
        detector = YoloDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 