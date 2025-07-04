#!/usr/bin/env python3
import rospy
import sys
from std_msgs.msg import Header

def switch_view(mode):
    """
    手动切换视图模式
    mode: 0=左, 1=右, 2=深度
    """
    rospy.init_node('manual_view_switcher', anonymous=True)
    
    try:
        mode = int(mode)
        if not (0 <= mode <= 2):
            print(f"错误: 无效的视图模式 {mode}，请使用 0(左), 1(右), 或 2(深度)")
            return False
    except ValueError:
        print("错误: 请提供一个有效的数字模式 0(左), 1(右), 或 2(深度)")
        return False
    
    # 直接发布到视图模式话题
    pub = rospy.Publisher('/stereo_vision/view_mode', Header, queue_size=10)
    
    # 等待发布者准备就绪
    rospy.sleep(1.0)
    
    # 创建并发布消息
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = str(mode)
    
    for _ in range(5):  # 多发几次以确保接收
        pub.publish(header)
        rospy.sleep(0.2)
    
    mode_names = {0: "左视图", 1: "右视图", 2: "深度视图"}
    print(f"已切换到 {mode_names.get(mode, '未知模式')}")
    return True

def main():
    if len(sys.argv) < 2:
        print("用法: switch_view.py <mode>")
        print("  mode: 0=左视图, 1=右视图, 2=深度视图")
        return
    
    mode = sys.argv[1]
    switch_view(mode)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass 