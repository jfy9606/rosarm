#!/usr/bin/env python3
import cv2
import numpy as np
import argparse
import os
import time

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='测试USB摄像头并显示实时图像')
    parser.add_argument('--device', type=int, default=0, help='摄像头设备ID')
    parser.add_argument('--width', type=int, default=1280, help='期望的图像宽度')
    parser.add_argument('--height', type=int, default=480, help='期望的图像高度')
    parser.add_argument('--split', action='store_true', help='是否分割立体图像')
    parser.add_argument('--fps', type=int, default=30, help='期望的帧率')
    parser.add_argument('--save', action='store_true', help='保存测试图像')
    parser.add_argument('--list', action='store_true', help='列出所有支持的分辨率')
    args = parser.parse_args()
    
    print(f"正在打开摄像头设备 /dev/video{args.device}")
    
    try:
        # 打开摄像头
        cap = cv2.VideoCapture(args.device)
        if not cap.isOpened():
            print(f"错误: 无法打开摄像头设备 /dev/video{args.device}")
            return
            
        # 列出所有支持的分辨率
        if args.list:
            list_camera_properties(cap)
            cap.release()
            return
            
        # 设置分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        
        # 设置帧率
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        
        # 获取实际设置的参数
        actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        
        print(f"期望分辨率: {args.width}x{args.height}")
        print(f"实际分辨率: {actual_width}x{actual_height}")
        print(f"实际帧率: {actual_fps}")
        
        # 帧计数和FPS计算
        frame_count = 0
        start_time = time.time()
        fps_update_interval = 1.0  # 每隔1秒更新一次FPS
        last_fps_time = start_time
        current_fps = 0
        
        while True:
            # 读取一帧
            ret, frame = cap.read()
            if not ret:
                print("错误: 无法读取帧")
                break
                
            # 更新帧计数和计算FPS
            frame_count += 1
            current_time = time.time()
            elapsed = current_time - last_fps_time
            
            if elapsed > fps_update_interval:
                current_fps = frame_count / elapsed
                frame_count = 0
                last_fps_time = current_time
                
            # 添加FPS文本
            fps_text = f"FPS: {current_fps:.1f}"
            cv2.putText(frame, fps_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 添加尺寸文本
            size_text = f"尺寸: {frame.shape[1]}x{frame.shape[0]}"
            cv2.putText(frame, size_text, (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # 如果是分割图像
            if args.split and frame.shape[1] >= 2 * frame.shape[0]:
                # 计算分割点 (水平中点)
                mid_point = frame.shape[1] // 2
                
                # 分割左右图像
                left_image = frame[:, :mid_point].copy()
                right_image = frame[:, mid_point:].copy()
                
                # 调整大小以适应屏幕
                scale = min(1.0, 1920 / (left_image.shape[1] + right_image.shape[1]))
                
                # 显示左右图像
                cv2.imshow('左眼图像', left_image)
                cv2.imshow('右眼图像', right_image)
                
                # 保存分割图像
                if args.save and frame_count == 1:
                    save_dir = "/tmp"
                    cv2.imwrite(os.path.join(save_dir, "camera_raw.jpg"), frame)
                    cv2.imwrite(os.path.join(save_dir, "camera_left.jpg"), left_image)
                    cv2.imwrite(os.path.join(save_dir, "camera_right.jpg"), right_image)
                    print(f"已保存图像到 {save_dir}/camera_*.jpg")
            else:
                # 显示原始图像
                cv2.imshow('摄像头测试', frame)
                
                # 保存原始图像
                if args.save and frame_count == 1:
                    save_path = "/tmp/camera_test.jpg"
                    cv2.imwrite(save_path, frame)
                    print(f"已保存图像到 {save_path}")
            
            # 检查按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("用户按下q键，退出")
                break
            elif key == ord('s'):
                # 按s键保存当前帧
                save_path = f"/tmp/camera_capture_{time.time():.0f}.jpg"
                cv2.imwrite(save_path, frame)
                print(f"已保存当前帧到 {save_path}")
    
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 释放资源
        if 'cap' in locals() and cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        print("已释放所有资源")

def list_camera_properties(cap):
    """列出摄像头支持的所有属性"""
    print("\n摄像头属性:")
    
    # 常用的OpenCV属性ID和名称
    properties = {
        cv2.CAP_PROP_FRAME_WIDTH: "宽度",
        cv2.CAP_PROP_FRAME_HEIGHT: "高度",
        cv2.CAP_PROP_FPS: "帧率",
        cv2.CAP_PROP_BRIGHTNESS: "亮度",
        cv2.CAP_PROP_CONTRAST: "对比度",
        cv2.CAP_PROP_SATURATION: "饱和度",
        cv2.CAP_PROP_HUE: "色调",
        cv2.CAP_PROP_GAIN: "增益",
        cv2.CAP_PROP_EXPOSURE: "曝光",
        cv2.CAP_PROP_AUTO_EXPOSURE: "自动曝光",
        cv2.CAP_PROP_AUTOFOCUS: "自动对焦",
        cv2.CAP_PROP_FOURCC: "编码格式"
    }
    
    for prop_id, prop_name in properties.items():
        value = cap.get(prop_id)
        if prop_id == cv2.CAP_PROP_FOURCC:
            # 将FOURCC转换为可读格式
            value = decode_fourcc(value)
        print(f"{prop_name}: {value}")
    
    # 测试常用分辨率
    test_resolutions(cap)

def decode_fourcc(value):
    """将FOURCC代码转换为可读格式"""
    return "".join([chr((int(value) >> 8 * i) & 0xFF) for i in range(4)])

def test_resolutions(cap):
    """测试摄像头支持的常用分辨率"""
    print("\n测试摄像头支持的分辨率:")
    
    # 常见的分辨率和帧率组合
    resolutions = [
        (640, 480),    # VGA
        (1280, 720),   # 720p
        (1920, 1080),  # 1080p
        (2560, 1440),  # 2K
        (3840, 2160),  # 4K
        (1280, 480),   # 双目立体相机
        (1920, 1080),  # 高清双目
        (2560, 720),   # 宽屏双目
        (3840, 1080),  # 超宽双目
    ]
    
    original_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    original_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    original_fps = cap.get(cv2.CAP_PROP_FPS)
    
    supported = []
    print("分辨率\t\t支持状态\t实际分辨率\t实际帧率")
    
    for width, height in resolutions:
        # 尝试设置分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        # 获取实际设置
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        
        # 检查是否支持
        is_supported = (actual_width == width and actual_height == height)
        status = "✓" if is_supported else "✗"
        
        print(f"{width}x{height}\t{status}\t\t{actual_width}x{actual_height}\t{actual_fps:.1f}")
        
        if is_supported:
            supported.append((width, height, actual_fps))
            
            # 抓取一帧测试
            ret, frame = cap.read()
            if ret:
                # 保存测试图像
                test_path = f"/tmp/test_{width}x{height}.jpg"
                cv2.imwrite(test_path, frame)
    
    # 恢复原始设置
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, original_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, original_height)
    
    print("\n摄像头支持的分辨率:")
    for width, height, fps in supported:
        print(f"{width}x{height} @ {fps:.1f}fps")
        print(f"测试图像: /tmp/test_{width}x{height}.jpg")

if __name__ == "__main__":
    main() 