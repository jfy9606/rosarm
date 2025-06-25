#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
try:
    from std_srvs.srv import SetBool, SetBoolResponse
except ImportError:
    rospy.logerr("无法导入std_srvs.srv，请确保已安装ROS std_srvs包")
    # 创建一个简单的替代类
    class SetBool:
        def __init__(self):
            self.data = False
    
    class SetBoolResponse:
        def __init__(self):
            self.success = False
            self.message = ""

class YoloController:
    """YOLO检测控制器，用于启用或禁用YOLO检测"""
    
    def __init__(self):
        """初始化YOLO控制器节点"""
        rospy.init_node('yolo_controller')
        
        # 获取初始状态参数
        self.enabled = rospy.get_param('~initial_state', False)
        
        # 创建发布器，用于发布YOLO检测状态
        self.status_pub = rospy.Publisher('/yolo/status', Bool, queue_size=10)
        
        # 创建控制服务
        self.control_service = rospy.Service('/yolo/control', SetBool, self.handle_control)
        
        # 定期发布状态
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_status)
        
        rospy.loginfo(f"YOLO控制器已初始化，初始状态: {'启用' if self.enabled else '禁用'}")
    
    def handle_control(self, req):
        """处理控制请求"""
        self.enabled = req.data
        
        # 立即发布新状态
        self.publish_status(None)
        
        status_str = "启用" if self.enabled else "禁用"
        rospy.loginfo(f"YOLO检测已{status_str}")
        
        # 返回服务响应
        response = SetBoolResponse()
        response.success = True
        response.message = f"YOLO检测已{status_str}"
        return response
    
    def publish_status(self, event):
        """发布YOLO检测状态"""
        self.status_pub.publish(Bool(data=self.enabled))

def main():
    """主函数"""
    controller = YoloController()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    
    rospy.loginfo("YOLO控制器已关闭")

if __name__ == '__main__':
    main() 