#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import psutil
import os
import time
from std_msgs.msg import Float32, String

class ResourceMonitor:
    """
    系统资源监控节点
    监控CPU、内存、磁盘使用情况并发布到ROS话题
    """
    
    def __init__(self):
        """初始化资源监控节点"""
        rospy.init_node('resource_monitor', anonymous=True)
        
        # 初始化参数
        self.update_rate = rospy.get_param('~update_rate', 1.0)  # 更新频率（Hz）
        
        # 创建发布器
        self.cpu_pub = rospy.Publisher('/system/cpu_usage', Float32, queue_size=10)
        self.memory_pub = rospy.Publisher('/system/memory_usage', Float32, queue_size=10)
        self.disk_pub = rospy.Publisher('/system/disk_usage', Float32, queue_size=10)
        self.status_pub = rospy.Publisher('/system/status', String, queue_size=10, latch=True)
        
        # 创建定时器
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate), self.timer_callback)
        
        rospy.loginfo("资源监控节点初始化完成，更新频率: %.1f Hz", self.update_rate)
    
    def timer_callback(self, event):
        """定时器回调函数，发布系统资源使用情况"""
        try:
            # 获取CPU使用率
            cpu_percent = psutil.cpu_percent()
            
            # 获取内存使用率
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            
            # 获取磁盘使用率
            disk = psutil.disk_usage('/')
            disk_percent = disk.percent
            
            # 发布资源使用情况
            self.cpu_pub.publish(Float32(cpu_percent))
            self.memory_pub.publish(Float32(memory_percent))
            self.disk_pub.publish(Float32(disk_percent))
            
            # 发布状态信息
            status_msg = f"CPU: {cpu_percent:.1f}%, 内存: {memory_percent:.1f}%, 磁盘: {disk_percent:.1f}%"
            self.status_pub.publish(String(status_msg))
            
            # 每10次更新打印一次日志
            if int(time.time() * self.update_rate) % 10 == 0:
                rospy.loginfo(status_msg)
                
        except Exception as e:
            rospy.logerr(f"获取系统资源信息时出错: {str(e)}")

def main():
    """主函数"""
    try:
        # 创建资源监控节点
        monitor = ResourceMonitor()
        
        # 处理回调
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main() 