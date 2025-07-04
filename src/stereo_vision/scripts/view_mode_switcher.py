#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
try:
    from stereo_vision.srv import SetViewMode, SetViewModeResponse
except ImportError:
    rospy.logwarn("Could not import SetViewMode service. Service will not be available.")
    SetViewMode = None
    SetViewModeResponse = None

class ViewModeSwitcher:
    def __init__(self):
        """Initialize the view mode switcher node"""
        rospy.init_node('view_mode_switcher')
        
        # Get initial view mode from parameter
        try:
            self.current_mode = int(rospy.get_param('~initial_view_mode', 0))
            if not (0 <= self.current_mode <= 2):
                rospy.logwarn(f"Invalid initial_view_mode {self.current_mode}, using default (0)")
                self.current_mode = 0
        except (ValueError, TypeError):
            rospy.logwarn("Invalid initial_view_mode parameter, using default (0)")
            self.current_mode = 0
        
        # Create publisher for view mode changes
        self.view_mode_pub = rospy.Publisher('/stereo_vision/view_mode', Header, queue_size=10)
        
        # Create service for switching view modes if service type is available
        if SetViewMode is not None:
            rospy.Service('/stereo_vision/switch_view', SetViewMode, self.handle_switch_view)
        
        rospy.loginfo(f"View mode switcher initialized with mode {self.current_mode} (0=left, 1=right, 2=depth)")
        self.publish_current_mode()
    
    def handle_switch_view(self, req):
        """Handle request to switch view mode"""
        if 0 <= req.view_mode <= 2:
            self.current_mode = req.view_mode
            self.publish_current_mode()
            response = SetViewModeResponse()
            response.success = True
            response.message = f"Switched to view mode {self.current_mode}"
            return response
        else:
            response = SetViewModeResponse()
            response.success = False
            response.message = "Invalid view mode. Use 0 (left), 1 (right), or 2 (depth)"
            return response
    
    def publish_current_mode(self):
        """Publish current view mode"""
        header = Header()
        header.stamp = rospy.Time.now()
        # Use frame_id to store the view mode instead of seq (which is deprecated)
        header.frame_id = str(self.current_mode)
        self.view_mode_pub.publish(header)
    
    def run(self):
        """Run the node"""
        rate = rospy.Rate(1)  # 1 Hz
        last_published_mode = self.current_mode
        rospy.loginfo(f"Starting view mode: {self.current_mode} (0=left, 1=right, 2=depth)")
        
        while not rospy.is_shutdown():
            # 只在模式变化时输出日志
            if self.current_mode != last_published_mode:
                rospy.loginfo(f"View mode changed to: {self.current_mode} (0=left, 1=right, 2=depth)")
                last_published_mode = self.current_mode
            
            # 周期性发布当前模式
            self.publish_current_mode()
            rate.sleep()

if __name__ == '__main__':
    try:
        switcher = ViewModeSwitcher()
        switcher.run()
    except rospy.ROSInterruptException:
        pass 