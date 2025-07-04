#!/usr/bin/env python3
import rospy
import sys
from stereo_vision.srv import SetViewMode

def test_gui_service():
    """测试GUI使用的/set_view_mode服务"""
    rospy.init_node('gui_service_tester', anonymous=True)
    
    # 等待服务可用
    rospy.loginfo("等待GUI视图切换服务...")
    try:
        rospy.wait_for_service('/set_view_mode', timeout=5.0)
    except rospy.ROSException:
        rospy.logerr("服务/set_view_mode在5秒内没有启动，请确保view_mode_switcher节点正在运行")
        return False
    
    try:
        # 获取服务代理
        set_view_mode = rospy.ServiceProxy('/set_view_mode', SetViewMode)
        
        # 测试深度视图切换
        mode = 2  # 深度视图
        rospy.loginfo(f"向GUI服务发送深度视图切换请求 (mode={mode})")
        response = set_view_mode(mode)
        rospy.loginfo(f"响应: 成功={response.success}, 消息={response.message}")
        
        # 等待一会儿让视图刷新
        rospy.sleep(2.0)
        
        # 测试切换回左视图
        mode = 0  # 左视图
        rospy.loginfo(f"向GUI服务发送左视图切换请求 (mode={mode})")
        response = set_view_mode(mode)
        rospy.loginfo(f"响应: 成功={response.success}, 消息={response.message}")
        
        return True
    
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")
        return False

if __name__ == '__main__':
    try:
        if test_gui_service():
            rospy.loginfo("GUI服务测试成功完成")
        else:
            rospy.logerr("GUI服务测试失败")
    except Exception as e:
        rospy.logerr(f"测试过程中发生错误: {e}") 