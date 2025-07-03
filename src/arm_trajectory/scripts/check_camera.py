#!/usr/bin/env python3
"""
相机诊断工具 - 检查相机是否可访问以及相关权限
"""

import sys
import os
import cv2
import time
import subprocess

def check_video_devices():
    """检查系统中的视频设备"""
    print("\n=== 检查系统视频设备 ===")
    
    # 检查/dev/video*设备文件
    video_devices = []
    try:
        for dev in os.listdir('/dev'):
            if dev.startswith('video'):
                video_path = f"/dev/{dev}"
                video_devices.append(video_path)
                
                # 获取设备权限信息
                try:
                    result = subprocess.run(['ls', '-la', video_path], 
                                           stdout=subprocess.PIPE, 
                                           stderr=subprocess.PIPE,
                                           text=True)
                    print(f"设备 {video_path}: {result.stdout.strip()}")
                except Exception as e:
                    print(f"无法获取 {video_path} 的权限信息: {e}")
        
        if not video_devices:
            print("没有找到视频设备。请检查相机是否已连接。")
            return False
            
        print(f"找到 {len(video_devices)} 个视频设备")
        return True
    except Exception as e:
        print(f"检查视频设备时出错: {e}")
        return False

def check_user_permissions():
    """检查当前用户是否有访问视频设备的权限"""
    print("\n=== 检查用户权限 ===")
    try:
        # 检查当前用户是否在video组中
        result = subprocess.run(['groups'], 
                              stdout=subprocess.PIPE, 
                              stderr=subprocess.PIPE,
                              text=True)
        
        groups = result.stdout.strip().split()
        if 'video' in groups:
            print("当前用户已在video组中，应该有相机访问权限")
            return True
        else:
            print("当前用户不在video组中，可能没有相机访问权限")
            print("建议执行: sudo usermod -a -G video $USER")
            print("然后注销并重新登录")
            return False
    except Exception as e:
        print(f"检查用户权限时出错: {e}")
        return False

def test_camera_access(device_indices=None):
    """尝试打开相机并读取帧"""
    print("\n=== 尝试访问相机 ===")
    
    if device_indices is None:
        # 尝试常见的设备索引和路径
        device_indices = [0, 1, 2, '/dev/video0', '/dev/video1', '/dev/video2']
    
    success = False
    
    for device in device_indices:
        print(f"尝试打开相机设备: {device}")
        
        try:
            # 尝试打开相机
            cap = cv2.VideoCapture(device)
            
            if not cap.isOpened():
                print(f"无法打开相机 {device}")
                continue
                
            print(f"成功打开相机 {device}")
            
            # 尝试读取一帧
            start_time = time.time()
            ret, frame = cap.read()
            elapsed = time.time() - start_time
            
            if ret and frame is not None and frame.size > 0:
                height, width = frame.shape[:2]
                print(f"成功读取帧: 尺寸={width}x{height}, 耗时={elapsed:.3f}秒")
                
                # 获取相机属性
                fps = cap.get(cv2.CAP_PROP_FPS)
                brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
                contrast = cap.get(cv2.CAP_PROP_CONTRAST)
                
                print(f"相机属性: FPS={fps}, 亮度={brightness}, 对比度={contrast}")
                success = True
            else:
                print(f"无法读取帧，耗时={elapsed:.3f}秒")
                
            # 释放相机资源
            cap.release()
            
        except Exception as e:
            print(f"访问相机 {device} 时出错: {e}")
    
    return success

def check_opencv_version():
    """检查OpenCV版本"""
    print("\n=== 检查OpenCV版本 ===")
    try:
        print(f"OpenCV版本: {cv2.__version__}")
        
        # 检查是否支持GStreamer
        gst_support = 'GStreamer' in cv2.getBuildInformation()
        print(f"GStreamer支持: {'是' if gst_support else '否'}")
        
        return True
    except Exception as e:
        print(f"检查OpenCV版本时出错: {e}")
        return False

def main():
    """主函数"""
    print("=== 相机诊断工具 ===")
    print(f"当前用户: {os.getenv('USER')}")
    print(f"Python版本: {sys.version}")
    
    # 执行检查
    devices_exist = check_video_devices()
    user_perm_ok = check_user_permissions()
    opencv_ok = check_opencv_version()
    camera_access_ok = test_camera_access()
    
    # 总结
    print("\n=== 诊断总结 ===")
    print(f"视频设备检测: {'通过' if devices_exist else '失败'}")
    print(f"用户权限检查: {'通过' if user_perm_ok else '失败'}")
    print(f"OpenCV检查: {'通过' if opencv_ok else '失败'}")
    print(f"相机访问测试: {'通过' if camera_access_ok else '失败'}")
    
    # 给出建议
    print("\n=== 建议 ===")
    if not devices_exist:
        print("- 请确认相机已正确连接到USB端口")
        print("- 尝试使用其他USB端口")
        print("- 检查相机驱动是否已安装")
    
    if not user_perm_ok:
        print("- 将当前用户添加到video组: sudo usermod -a -G video $USER")
        print("- 重新启动或注销并重新登录以应用权限更改")
    
    if not camera_access_ok:
        print("- 尝试使用模拟相机模式进行开发测试: roslaunch arm_trajectory mock_camera.launch")
        print("- 检查是否有其他应用程序正在使用相机")
        print("- 检查相机的固件是否支持Linux")

if __name__ == "__main__":
    main() 