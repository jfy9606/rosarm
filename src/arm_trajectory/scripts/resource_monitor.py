#!/usr/bin/env python3
import rospy
import psutil
import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import threading
import time

class ResourceMonitor:
    """Node for monitoring system resources and publishing metrics"""
    
    def __init__(self):
        rospy.init_node('resource_monitor', anonymous=True)
        
        # Parameters
        self.update_interval = rospy.get_param('~update_interval', 1.0)  # seconds
        
        # Publishers
        self.resource_pub = rospy.Publisher('/system/resources', Float32MultiArray, queue_size=1)
        
        # Initialize metrics
        self.cpu_percent = 0.0
        self.memory_percent = 0.0
        self.disk_percent = 0.0
        
        # Start monitoring thread
        self.stop_thread = False
        self.monitor_thread = threading.Thread(target=self.monitor_resources)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        rospy.loginfo("Resource Monitor initialized")
        
        # Register shutdown hook
        rospy.on_shutdown(self.shutdown)
    
    def monitor_resources(self):
        """Thread function to continuously monitor system resources"""
        while not self.stop_thread and not rospy.is_shutdown():
            try:
                # Get CPU usage
                self.cpu_percent = psutil.cpu_percent(interval=None)
                
                # Get memory usage
                memory = psutil.virtual_memory()
                self.memory_percent = memory.percent
                
                # Get disk usage
                disk = psutil.disk_usage('/')
                self.disk_percent = disk.percent
                
                # Publish metrics
                self.publish_metrics()
                
            except Exception as e:
                rospy.logerr(f"Error monitoring resources: {e}")
            
            # Sleep for the specified interval
            time.sleep(self.update_interval)
    
    def publish_metrics(self):
        """Publish resource metrics to ROS topic"""
        try:
            # Create message
            msg = Float32MultiArray()
            
            # Set up dimensions
            dim = MultiArrayDimension()
            dim.label = "resources"
            dim.size = 3
            dim.stride = 3
            msg.layout.dim.append(dim)
            msg.layout.data_offset = 0
            
            # Set data: [CPU%, Memory%, Disk%]
            msg.data = [float(self.cpu_percent), float(self.memory_percent), float(self.disk_percent)]
            
            # Publish
            self.resource_pub.publish(msg)
            
            # Log periodically (at lower frequency to avoid filling logs)
            if int(time.time()) % 60 == 0:
                rospy.loginfo(f"System Resources - CPU: {self.cpu_percent:.1f}%, "
                             f"Memory: {self.memory_percent:.1f}%, "
                             f"Disk: {self.disk_percent:.1f}%")
                
        except Exception as e:
            rospy.logerr(f"Error publishing metrics: {e}")
    
    def shutdown(self):
        """Clean up on shutdown"""
        rospy.loginfo("Shutting down resource monitor")
        self.stop_thread = True
        if self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1.0)

if __name__ == '__main__':
    try:
        monitor = ResourceMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 