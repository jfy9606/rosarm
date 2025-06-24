#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool

class YoloController:
    def __init__(self):
        rospy.init_node('yolo_controller', anonymous=True)
        
        # 参数
        self.enabled = rospy.get_param('~enabled', False)
        
        # 发布器：发布YOLO启用状态
        self.status_pub = rospy.Publisher('/yolo_detection/status', Bool, queue_size=1)
        
        # 服务：控制YOLO开关
        self.service = rospy.Service('/yolo_detection/set_enabled', SetBool, self.set_enabled_callback)
        
        # 定时发布状态
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        
        rospy.loginfo("YOLO控制器已启动，当前状态: %s", "启用" if self.enabled else "禁用")
    
    def set_enabled_callback(self, req):
        """处理服务调用，设置YOLO启用状态"""
        old_state = self.enabled
        self.enabled = req.data
        
        # 立即发布新状态
        self.publish_status(None)
        
        response = SetBoolResponse()
        response.success = True
        if old_state != self.enabled:
            response.message = f"YOLO检测已{'启用' if self.enabled else '禁用'}"
            rospy.loginfo(response.message)
        else:
            response.message = f"YOLO检测状态未改变，当前为{'启用' if self.enabled else '禁用'}"
        
        return response
    
    def publish_status(self, event=None):
        """发布当前YOLO状态"""
        status_msg = Bool()
        status_msg.data = self.enabled
        self.status_pub.publish(status_msg)

if __name__ == '__main__':
    try:
        controller = YoloController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 