#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import numpy as np
import cv2
import time
import warnings
import rospy

# Try to import ultralytics
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    rospy.logwarn("Unable to import ultralytics, will use simulated detections")

class DetectorControl:
    """
    Core detector functionality for object detection using YOLO
    """
    
    def __init__(self, model_path="yolo11n.pt", conf_threshold=0.25, auto_download=True):
        """
        Initialize the detector control
        
        Args:
            model_path: YOLO model file path
            conf_threshold: Detection confidence threshold
            auto_download: Whether to automatically download the model if not found
        """
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.auto_download = auto_download
        
        # Initialize model
        self.model = None
        self.model_loaded = False
        self.class_names = {}
        
        # Load YOLO model
        self._load_model()
    
    def _load_model(self):
        """Load the YOLO model"""
        if not YOLO_AVAILABLE:
            rospy.logwarn("Ultralytics not installed, cannot load YOLO model")
            return
            
        try:
            # Try to load the specified model
            if os.path.exists(str(self.model_path)):
                rospy.loginfo(f"Loading local YOLO model: {self.model_path}")
                self.model = YOLO(str(self.model_path))
                self.model_loaded = True
                rospy.loginfo(f"Successfully loaded YOLO model: {self.model_path}")
            elif self.auto_download:
                # If specified model doesn't exist but auto-download is enabled, try to download
                rospy.loginfo(f"Model file not found: {self.model_path}, attempting auto-download")
                try:
                    self.model = YOLO(self.model_path)  # Auto-download model
                    self.model_loaded = True
                    rospy.loginfo(f"Successfully downloaded and loaded YOLO model: {self.model_path}")
                except Exception as e:
                    rospy.logerr(f"Failed to auto-download YOLO model: {str(e)}, trying default model yolo11n.pt")
                    try:
                        self.model = YOLO('yolo11n.pt')  # Try using default model
                        self.model_loaded = True
                        rospy.loginfo("Successfully loaded default YOLO model: yolo11n.pt")
                    except Exception as e2:
                        rospy.logerr(f"Failed to load default YOLO model: {str(e2)}")
                        self.model = None
            else:
                # If model doesn't exist and auto-download is disabled, try default model
                rospy.logwarn(f"Model file not found: {self.model_path}, trying default model yolo11n.pt")
                try:
                    self.model = YOLO('yolo11n.pt')
                    self.model_loaded = True
                    rospy.loginfo("Successfully loaded default YOLO model")
                except Exception as e:
                    rospy.logerr(f"Failed to load default YOLO model: {str(e)}")
            
            # Get class names
            if self.model_loaded and hasattr(self.model, 'names'):
                self.class_names = self.model.names
                rospy.loginfo(f"Classes: {self.class_names}")
                
        except Exception as e:
            rospy.logerr(f"Error loading YOLO model: {str(e)}")
    
    def detect_objects(self, image):
        """
        Detect objects in an image
        
        Args:
            image: OpenCV format image
            
        Returns:
            detections: List of detection results, each containing [x1, y1, x2, y2, conf, class_id]
        """
        if not self.model_loaded or not YOLO_AVAILABLE:
            return []
            
        # Check if image is valid
        if image is None or image.size == 0:
            rospy.logwarn("Received invalid image")
            return []
            
        try:
            # Silence detection logs
            with open(os.devnull, 'w') as f:
                # Temporarily redirect stdout to null device
                original_stdout = sys.stdout
                sys.stdout = f
                
                # Perform detection
                results = self.model(image, verbose=False)
                
                # Restore stdout
                sys.stdout = original_stdout
            
            # Process detection results
            detections = self.process_detections(results, self.conf_threshold)
            
            return detections
            
        except Exception as e:
            # Throttle error messages
            current_time = rospy.Time.now()
            if not hasattr(self, 'last_detect_error_time') or (current_time - self.last_detect_error_time).to_sec() > 10.0:
                rospy.logerr(f"YOLO detection error: {str(e)}")
                self.last_detect_error_time = current_time
            return []
    
    def process_detections(self, results, min_confidence=0.3):
        """
        Process YOLO detection results
        
        Args:
            results: YOLO detection results object
            min_confidence: Minimum confidence threshold
            
        Returns:
            detections: List of detection results, each containing [x1, y1, x2, y2, conf, class_id]
        """
        detections = []
        
        # Iterate through all detection results
        for result in results:
            boxes = result.boxes
            
            # Iterate through all bounding boxes
            for i in range(len(boxes)):
                # Get bounding box coordinates
                box = boxes[i].xyxy[0].cpu().numpy()  # Get xyxy format bounding box
                x1, y1, x2, y2 = box[:4]
                
                # Get confidence
                conf = float(boxes[i].conf[0].cpu().numpy())
                
                # Get class ID
                class_id = int(boxes[i].cls[0].cpu().numpy())
                
                # Check if confidence is above threshold
                if conf >= min_confidence:
                    # Add to detection results list
                    detections.append([x1, y1, x2, y2, conf, class_id])
        
        # Merge overlapping detections
        detections = self.merge_overlapping_detections(detections)
        
        return detections
    
    def merge_overlapping_detections(self, detections, iou_threshold=0.5, same_class_only=True):
        """
        Merge overlapping detection boxes
        
        Args:
            detections: Detection results list, each containing [x1, y1, x2, y2, conf, class_id]
            iou_threshold: IoU threshold, boxes with IoU above this will be merged
            same_class_only: Whether to only merge boxes of the same class
            
        Returns:
            merged_detections: Merged detection results list
        """
        # If fewer than 2 detections, no need to merge
        if len(detections) < 2:
            return detections
        
        # Sort by confidence in descending order
        detections.sort(key=lambda x: x[4], reverse=True)
        
        # Convert to numpy array for easier processing
        dets = np.array(detections)
        
        # Initialize list of indices to keep
        keep = []
        
        # Iterate through all boxes
        while len(dets) > 0:
            # Keep the current highest confidence box
            keep.append(dets[0])
            
            # If only one box left, end loop
            if len(dets) == 1:
                break
            
            # Calculate IoU of current box with all other boxes
            ious = np.array([self.calculate_iou(dets[0], det) for det in dets[1:]])
            
            # If only merging same class boxes, check if classes are the same
            if same_class_only:
                # Get class ID
                class_id = dets[0][5]
                
                # Create class mask
                class_mask = dets[1:, 5] == class_id
                
                # Apply class mask
                ious = ious * class_mask
            
            # Find indices of boxes with IoU below threshold
            indices = np.where(ious < iou_threshold)[0]
            
            # Update dets, only keep boxes with IoU below threshold
            dets = dets[1:][indices]
        
        return keep
    
    def calculate_iou(self, box1, box2):
        """
        Calculate IoU between two boxes
        
        Args:
            box1: First box [x1, y1, x2, y2, conf, class_id]
            box2: Second box [x1, y1, x2, y2, conf, class_id]
            
        Returns:
            iou: IoU value
        """
        # Extract coordinates
        box1_x1, box1_y1, box1_x2, box1_y2 = box1[:4]
        box2_x1, box2_y1, box2_x2, box2_y2 = box2[:4]
        
        # Calculate intersection area coordinates
        x1 = max(box1_x1, box2_x1)
        y1 = max(box1_y1, box2_y1)
        x2 = min(box1_x2, box2_x2)
        y2 = min(box1_y2, box2_y2)
        
        # Calculate intersection area
        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        
        # Calculate box areas
        box1_area = (box1_x2 - box1_x1) * (box1_y2 - box1_y1)
        box2_area = (box2_x2 - box2_x1) * (box2_y2 - box2_y1)
        
        # Calculate union area
        union = box1_area + box2_area - intersection
        
        # Calculate IoU
        iou = intersection / union if union > 0 else 0
        
        return iou
    
    def draw_detections(self, image, detections):
        """
        Draw detection results on an image
        
        Args:
            image: OpenCV format image
            detections: Detection results list, each containing [x1, y1, x2, y2, conf, class_id]
            
        Returns:
            annotated_image: Image with detection boxes drawn
        """
        # Create image copy
        annotated_image = image.copy()
        
        # Iterate through detection results
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            
            # Ensure coordinates are integers
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # Get class name
            class_id = int(class_id)
            class_name = self.class_names[class_id] if class_id in self.class_names else f"class_{class_id}"
            
            # Set color (based on class ID)
            color = (int(255 * (class_id % 3 / 3.0)), 
                     int(255 * ((class_id + 1) % 3 / 3.0)), 
                     int(255 * ((class_id + 2) % 3 / 3.0)))
            
            # Draw rectangle
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
            
            # Annotate class name and confidence
            label = f"{class_name}: {conf:.2f}"
            
            # Get text box size
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            
            # Ensure text box is within image bounds
            y1 = max(y1, text_height + baseline)
            
            # Draw text box background
            cv2.rectangle(
                annotated_image, 
                (x1, y1 - text_height - baseline),
                (x1 + text_width, y1),
                color, 
                -1
            )
            
            # Draw text
            cv2.putText(
                annotated_image,
                label,
                (x1, y1 - baseline),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2
            )
        
        return annotated_image