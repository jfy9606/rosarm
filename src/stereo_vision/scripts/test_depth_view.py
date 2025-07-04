#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import Header

def test_depth_view():
    """直接发布视图模式切换消息以测试深度视图"""
    rospy.init_node('depth_view_tester', anonymous=True)
    
    # 创建发布者
    view_pub = rospy.Publisher('/stereo_vision/view_mode', Header, queue_size=10)
    view_relay_pub = rospy.Publisher('/stereo_vision/view_mode_relay', Header, queue_size=10)
    
    rospy.loginfo("启动深度视图测试...")
    rospy.sleep(1.0)  # 等待发布者初始化
    
    # 准备深度视图消息
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "2"  # 2 = 深度视图
    
    # 多次发布消息，确保能被接收
    for i in range(10):
        view_pub.publish(header)
        view_relay_pub.publish(header)
        rospy.loginfo(f"发送深度视图切换消息 #{i+1}/10")
        time.sleep(0.5)
    
    rospy.loginfo("深度视图测试完成！")

if __name__ == '__main__':
    try:
        test_depth_view()
    except rospy.ROSInterruptException:
        pass 