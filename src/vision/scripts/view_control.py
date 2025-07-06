#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from enum import Enum

class ViewMode(Enum):
    """Enum for different view modes"""
    NORMAL = 0
    ENHANCED = 1
    DEBUG = 2
    CUSTOM = 3

class ViewControl:
    """
    Core view functionality for image display and visualization
    """
    
    def __init__(self):
        """Initialize the view control"""
        # Initialize view mode
        self.view_mode = ViewMode.NORMAL
        
        # Initialize overlay parameters
        self.show_grid = False
        self.show_info = True
        self.show_detections = True
        
        # Initialize grid parameters
        self.grid_size = 50  # Grid cell size in pixels
        self.grid_color = (50, 50, 50)  # Grid color (gray)
        self.grid_thickness = 1  # Grid line thickness
        
        # Initialize info parameters
        self.info_font = cv2.FONT_HERSHEY_SIMPLEX
        self.info_font_scale = 0.6
        self.info_color = (0, 255, 0)  # Green
        self.info_thickness = 1
        
        # Initialize detection visualization parameters
        self.detection_thickness = 2
        self.detection_font_scale = 0.5
        
        # Initialize custom parameters
        self.custom_params = {}
    
    def set_view_mode(self, mode):
        """
        Set view mode
        
        Args:
            mode: View mode (NORMAL, ENHANCED, DEBUG, CUSTOM)
        """
        if isinstance(mode, int):
            try:
                self.view_mode = ViewMode(mode)
            except ValueError:
                rospy.logwarn(f"Invalid view mode: {mode}, using NORMAL")
                self.view_mode = ViewMode.NORMAL
        elif isinstance(mode, ViewMode):
            self.view_mode = mode
        else:
            rospy.logwarn(f"Invalid view mode type: {type(mode)}, using NORMAL")
            self.view_mode = ViewMode.NORMAL
        
        # Update overlay parameters based on view mode
        if self.view_mode == ViewMode.NORMAL:
            self.show_grid = False
            self.show_info = True
            self.show_detections = True
        elif self.view_mode == ViewMode.ENHANCED:
            self.show_grid = True
            self.show_info = True
            self.show_detections = True
        elif self.view_mode == ViewMode.DEBUG:
            self.show_grid = True
            self.show_info = True
            self.show_detections = True
        elif self.view_mode == ViewMode.CUSTOM:
            # Use custom parameters
            pass
    
    def set_custom_params(self, params):
        """
        Set custom parameters
        
        Args:
            params: Dictionary of custom parameters
        """
        self.custom_params = params
        
        # Update overlay parameters from custom parameters
        if 'show_grid' in params:
            self.show_grid = params['show_grid']
        if 'show_info' in params:
            self.show_info = params['show_info']
        if 'show_detections' in params:
            self.show_detections = params['show_detections']
        if 'grid_size' in params:
            self.grid_size = params['grid_size']
        if 'grid_color' in params:
            self.grid_color = params['grid_color']
        if 'grid_thickness' in params:
            self.grid_thickness = params['grid_thickness']
    
    def draw_grid(self, image):
        """
        Draw grid on image
        
        Args:
            image: Input image
            
        Returns:
            image_with_grid: Image with grid
        """
        if not self.show_grid:
            return image
        
        # Create copy of image
        image_with_grid = image.copy()
        
        # Get image dimensions
        height, width = image.shape[:2]
        
        # Draw vertical grid lines
        for x in range(0, width, self.grid_size):
            cv2.line(image_with_grid, (x, 0), (x, height), self.grid_color, self.grid_thickness)
        
        # Draw horizontal grid lines
        for y in range(0, height, self.grid_size):
            cv2.line(image_with_grid, (0, y), (width, y), self.grid_color, self.grid_thickness)
        
        return image_with_grid
    
    def draw_info(self, image, info=None):
        """
        Draw information on image
        
        Args:
            image: Input image
            info: Dictionary of information to display
            
        Returns:
            image_with_info: Image with information
        """
        if not self.show_info:
            return image
        
        # Create copy of image
        image_with_info = image.copy()
        
        # Initialize info if not provided
        if info is None:
            info = {}
        
        # Add view mode to info
        info['Mode'] = self.view_mode.name
        
        # Get image dimensions
        height, width = image.shape[:2]
        
        # Draw info
        y_offset = 30
        for key, value in info.items():
            text = f"{key}: {value}"
            cv2.putText(
                image_with_info,
                text,
                (10, y_offset),
                self.info_font,
                self.info_font_scale,
                self.info_color,
                self.info_thickness
            )
            y_offset += 25
        
        return image_with_info
    
    def draw_detections(self, image, detections, class_names=None):
        """
        Draw detections on image
        
        Args:
            image: Input image
            detections: List of detections, each containing [x1, y1, x2, y2, conf, class_id]
            class_names: Dictionary mapping class IDs to names
            
        Returns:
            image_with_detections: Image with detections
        """
        if not self.show_detections or not detections:
            return image
        
        # Create copy of image
        image_with_detections = image.copy()
        
        # Initialize class names if not provided
        if class_names is None:
            class_names = {}
        
        # Draw each detection
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            
            # Ensure coordinates are integers
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # Get class name
            class_id = int(class_id)
            class_name = class_names.get(class_id, f"class_{class_id}")
            
            # Set color (based on class ID)
            color = (int(255 * (class_id % 3 / 3.0)), 
                     int(255 * ((class_id + 1) % 3 / 3.0)), 
                     int(255 * ((class_id + 2) % 3 / 3.0)))
            
            # Draw rectangle
            cv2.rectangle(image_with_detections, (x1, y1), (x2, y2), color, self.detection_thickness)
            
            # Draw label
            label = f"{class_name}: {conf:.2f}"
            
            # Get text size
            (text_width, text_height), baseline = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, self.detection_font_scale, self.detection_thickness)
            
            # Ensure text box is within image bounds
            y1 = max(y1, text_height + baseline)
            
            # Draw text box background
            cv2.rectangle(
                image_with_detections, 
                (x1, y1 - text_height - baseline),
                (x1 + text_width, y1),
                color, 
                -1
            )
            
            # Draw text
            cv2.putText(
                image_with_detections,
                label,
                (x1, y1 - baseline),
                cv2.FONT_HERSHEY_SIMPLEX,
                self.detection_font_scale,
                (255, 255, 255),
                self.detection_thickness
            )
        
        return image_with_detections
    
    def apply_effects(self, image, effects=None):
        """
        Apply effects to image
        
        Args:
            image: Input image
            effects: List of effects to apply
            
        Returns:
            processed_image: Image with effects applied
        """
        if effects is None:
            return image
        
        # Create copy of image
        processed_image = image.copy()
        
        # Apply effects
        for effect in effects:
            if effect == 'blur':
                processed_image = cv2.GaussianBlur(processed_image, (5, 5), 0)
            elif effect == 'sharpen':
                kernel = np.array([[-1, -1, -1], [-1, 9, -1], [-1, -1, -1]])
                processed_image = cv2.filter2D(processed_image, -1, kernel)
            elif effect == 'edge':
                processed_image = cv2.Canny(processed_image, 100, 200)
                # Convert back to BGR if needed
                if len(image.shape) == 3 and len(processed_image.shape) == 2:
                    processed_image = cv2.cvtColor(processed_image, cv2.COLOR_GRAY2BGR)
            elif effect == 'grayscale' and len(processed_image.shape) == 3:
                processed_image = cv2.cvtColor(processed_image, cv2.COLOR_BGR2GRAY)
                # Convert back to BGR if needed
                if len(image.shape) == 3:
                    processed_image = cv2.cvtColor(processed_image, cv2.COLOR_GRAY2BGR)
            elif effect == 'invert':
                processed_image = cv2.bitwise_not(processed_image)
        
        return processed_image
    
    def process_image(self, image, detections=None, info=None, effects=None):
        """
        Process image based on current view mode
        
        Args:
            image: Input image
            detections: List of detections
            info: Dictionary of information to display
            effects: List of effects to apply
            
        Returns:
            processed_image: Processed image
        """
        if image is None:
            return None
        
        # Apply effects
        if effects is not None:
            image = self.apply_effects(image, effects)
        
        # Process based on view mode
        if self.view_mode == ViewMode.NORMAL:
            # Draw detections
            if detections is not None and self.show_detections:
                image = self.draw_detections(image, detections)
            
            # Draw info
            if self.show_info:
                image = self.draw_info(image, info)
                
        elif self.view_mode == ViewMode.ENHANCED:
            # Draw grid
            if self.show_grid:
                image = self.draw_grid(image)
            
            # Draw detections
            if detections is not None and self.show_detections:
                image = self.draw_detections(image, detections)
            
            # Draw info
            if self.show_info:
                image = self.draw_info(image, info)
                
        elif self.view_mode == ViewMode.DEBUG:
            # Draw grid
            if self.show_grid:
                image = self.draw_grid(image)
            
            # Draw detections with more details
            if detections is not None and self.show_detections:
                image = self.draw_detections(image, detections)
            
            # Draw info with more details
            debug_info = info.copy() if info else {}
            debug_info['Grid'] = 'On' if self.show_grid else 'Off'
            debug_info['Detections'] = 'On' if self.show_detections else 'Off'
            debug_info['Effects'] = str(effects) if effects else 'None'
            
            image = self.draw_info(image, debug_info)
            
        elif self.view_mode == ViewMode.CUSTOM:
            # Use custom parameters
            
            # Draw grid
            if self.show_grid:
                image = self.draw_grid(image)
            
            # Draw detections
            if detections is not None and self.show_detections:
                image = self.draw_detections(image, detections)
            
            # Draw info
            if self.show_info:
                image = self.draw_info(image, info)
        
        return image 

# Add this at the end of the file to properly export the classes
__all__ = ['ViewControl', 'ViewMode'] 