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
        
        # 获取图像话题名称
        self.left_image_topic = rospy.get_param('~left_image_topic', '/left_camera/image_raw')
        self.right_image_topic = rospy.get_param('~right_image_topic', '/right_camera/image_raw')
        self.depth_image_topic = rospy.get_param('~depth_image_topic', '/depth_image')
        
        # Create publisher for view mode changes
        self.view_mode_pub = rospy.Publisher('/stereo_vision/view_mode', Header, queue_size=10)
        
        # Create service for switching view modes if service type is available
        if SetViewMode is not None:
            # 创建两个服务，一个是原来的，一个是GUI使用的
            self.service1 = rospy.Service('/stereo_vision/switch_view', SetViewMode, self.handle_switch_view)
            self.service2 = rospy.Service('/set_view_mode', SetViewMode, self.handle_switch_view)
            rospy.loginfo("View mode switching services started at /stereo_vision/switch_view and /set_view_mode")
        
        # 初始化日志控制
        self.last_log_time = rospy.Time.now()
        self.log_interval = rospy.Duration(5.0)  # 5秒内不重复输出相同的日志
        
        rospy.loginfo(f"View mode switcher initialized with mode {self.current_mode} (0=left, 1=right, 2=depth)")
        
        # 立即发布当前模式几次，确保其他节点能收到
        for _ in range(3):
            self.publish_current_mode(force=True)
            rospy.sleep(0.5)
    
    def handle_switch_view(self, req):
        """Handle request to switch view mode"""
        if 0 <= req.view_mode <= 2:
            # 检查是否切换到了新的模式
            if self.current_mode != req.view_mode:
                old_mode = self.current_mode
                self.current_mode = req.view_mode
                rospy.loginfo(f"Switched from mode {old_mode} to mode {self.current_mode} (0=left, 1=right, 2=depth)")
                
                # 切换模式时多发布几次，确保接收方能收到
                for _ in range(5):
                    self.publish_current_mode(force=True)
                    rospy.sleep(0.1)
            else:
                rospy.loginfo(f"Already in mode {self.current_mode}, no change needed")
            
            response = SetViewModeResponse()
            response.success = True
            response.message = f"Switched to view mode {self.current_mode}"
            return response
        else:
            response = SetViewModeResponse()
            response.success = False
            response.message = "Invalid view mode. Use 0 (left), 1 (right), or 2 (depth)"
            return response
    
    def publish_current_mode(self, force=False):
        """Publish current view mode"""
        header = Header()
        header.stamp = rospy.Time.now()
        # Use frame_id to store the view mode instead of seq (which is deprecated)
        header.frame_id = str(self.current_mode)
        self.view_mode_pub.publish(header)
        
        # 控制日志输出频率，只有强制输出或达到时间间隔时才输出
        current_time = rospy.Time.now()
        if force or (current_time - self.last_log_time) > self.log_interval:
            mode_descriptions = {
                0: "左摄像头",
                1: "右摄像头",
                2: "深度视图"
            }
            rospy.logdebug(f"当前视图模式: {mode_descriptions.get(self.current_mode, '未知')}")
            self.last_log_time = current_time
    
    def run(self):
        """Run the node"""
        rate = rospy.Rate(2)  # 2 Hz, 增加频率以确保可靠切换
        last_published_mode = self.current_mode
        rospy.loginfo(f"Starting view mode switcher with mode {self.current_mode} (0=left, 1=right, 2=depth)")
        
        while not rospy.is_shutdown():
            # 只在模式变化时输出日志
            if self.current_mode != last_published_mode:
                rospy.loginfo(f"View mode changed to: {self.current_mode} (0=left, 1=right, 2=depth)")
                last_published_mode = self.current_mode
                # 模式变化时多发布几次，确保可靠切换
                for _ in range(3):
                    self.publish_current_mode(force=True)
                    rospy.sleep(0.1)
            else:
                # 正常运行时静默发布
                self.publish_current_mode(force=False)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        switcher = ViewModeSwitcher()
        switcher.run()
    except rospy.ROSInterruptException:
        pass 