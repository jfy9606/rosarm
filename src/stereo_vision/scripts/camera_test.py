#!/usr/bin/env python3
import cv2
import numpy as np
import argparse
import os
import time

def test_camera(device_path, width=1280, height=480, fps=30, show_fps=True, save_image=False, save_dir=None):
    """测试摄像头，显示图像并计算帧率"""
    if not os.path.exists(device_path):
        print(f"错误: 设备 {device_path} 不存在")
        return False
    
    print(f"正在测试摄像头: {device_path}")
    print(f"请求分辨率: {width}x{height}, FPS: {fps}")
    
    # 打开摄像头
    cap = cv2.VideoCapture(device_path)
    if not cap.isOpened():
        print(f"错误: 无法打开摄像头 {device_path}")
        return False
    
    # 设置摄像头参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    
    # 尝试不同的像素格式
    formats = [
        (cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'), "MJPG"),
        (cv2.VideoWriter.fourcc('Y', 'U', 'Y', 'V'), "YUYV"),
        (cv2.VideoWriter.fourcc('B', 'G', 'R', '3'), "BGR3"),
        (cv2.VideoWriter.fourcc('Y', 'U', 'Y', '2'), "YUY2"),
    ]
    
    success = False
    active_format = "未知"
    
    for fmt_code, fmt_name in formats:
        print(f"尝试格式: {fmt_name}")
        cap.set(cv2.CAP_PROP_FOURCC, fmt_code)
        
        # 读取一帧测试
        ret, frame = cap.read()
        if ret:
            print(f"成功使用格式: {fmt_name}")
            active_format = fmt_name
            success = True
            break
    
    if not success:
        print("错误: 无法使用任何支持的格式读取图像")
        cap.release()
        return False
    
    # 获取实际分辨率和FPS
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"实际分辨率: {actual_width}x{actual_height}, 实际FPS: {actual_fps}")
    print(f"像素格式: {active_format}")
    
    # 是否为宽屏图像（宽高比大于1.9:1）
    wide_screen = (actual_width >= actual_height * 1.9)
    if wide_screen:
        print("检测到宽幅图像，将自动处理左半部分")
    
    # 计算帧率
    start_time = time.time()
    frame_count = 0
    fps_value = 0
    
    # 创建窗口
    window_name = f"摄像头测试 - {os.path.basename(device_path)}"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, actual_width, actual_height)
    
    # 如果是宽屏图像，创建左半部分窗口
    if wide_screen:
        processed_window = "处理后图像"
        cv2.namedWindow(processed_window, cv2.WINDOW_NORMAL)
        half_width = actual_width // 2
        cv2.resizeWindow(processed_window, half_width, actual_height)
    
    print("按 'q' 退出, 's' 保存当前帧")
    
    # 创建保存目录
    if save_image and save_dir is not None:
        os.makedirs(save_dir, exist_ok=True)
    
    # 用于检测黑帧的阈值
    black_threshold = 10
    consecutive_black_frames = 0
    max_black_frames = 5
    
    try:
        while True:
            # 读取一帧
            ret, frame = cap.read()
            if not ret:
                print("警告: 无法读取帧")
                time.sleep(0.1)
                continue
            
            # 检测黑帧
            mean_value = np.mean(frame)
            if mean_value < black_threshold:
                consecutive_black_frames += 1
                if consecutive_black_frames > max_black_frames:
                    print(f"警告: 检测到连续黑帧 ({mean_value:.2f})")
                    consecutive_black_frames = 0
            else:
                consecutive_black_frames = 0
            
            # 更新帧率
            frame_count += 1
            elapsed = time.time() - start_time
            if elapsed >= 1.0:
                fps_value = frame_count / elapsed
                frame_count = 0
                start_time = time.time()
            
            # 添加信息叠加层
            if show_fps:
                info_text = [
                    f"设备: {os.path.basename(device_path)}",
                    f"分辨率: {actual_width}x{actual_height}",
                    f"格式: {active_format}",
                    f"FPS: {fps_value:.1f}",
                    f"亮度: {mean_value:.1f}"
                ]
                
                # 在帧上显示信息
                y = 30
                for text in info_text:
                    cv2.putText(frame, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    y += 30
            
            # 显示原始图像
            cv2.imshow(window_name, frame)
            
            # 如果是宽屏图像，处理左半部分
            processed_img = None
            if wide_screen:
                half_width = frame.shape[1] // 2
                processed_img = frame[:, :half_width].copy()
                
                # 添加处理后图像的信息
                cv2.putText(processed_img, "处理后图像", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow(processed_window, processed_img)
            
            # 键盘操作
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s') and save_image:
                # 保存当前帧
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                
                if save_dir is not None:
                    filename = os.path.join(save_dir, f"camera_{os.path.basename(device_path)}_{timestamp}.jpg")
                else:
                    filename = f"camera_{os.path.basename(device_path)}_{timestamp}.jpg"
                
                cv2.imwrite(filename, frame)
                print(f"已保存图像: {filename}")
                
                # 如果是宽屏图像，也保存处理后的图像
                if wide_screen and processed_img is not None:
                    if save_dir is not None:
                        processed_filename = os.path.join(save_dir, f"processed_{os.path.basename(device_path)}_{timestamp}.jpg")
                    else:
                        processed_filename = f"processed_{os.path.basename(device_path)}_{timestamp}.jpg"
                    
                    cv2.imwrite(processed_filename, processed_img)
                    print(f"已保存处理后图像: {processed_filename}")
    
    except KeyboardInterrupt:
        print("用户中断，退出")
    finally:
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()
    
    return True

def list_video_devices():
    """列出所有可用的视频设备"""
    devices = []
    for i in range(10):  # 通常不会超过10个摄像头
        device_path = f"/dev/video{i}"
        if os.path.exists(device_path):
            devices.append(device_path)
    
    if not devices:
        print("未找到任何视频设备")
    else:
        print(f"找到 {len(devices)} 个视频设备:")
        for dev in devices:
            print(f"  - {dev}")
    
    return devices

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="摄像头测试工具")
    parser.add_argument("--list", action="store_true", help="列出所有可用的视频设备")
    parser.add_argument("--device", "-d", type=str, help="要测试的摄像头设备路径")
    parser.add_argument("--width", "-W", type=int, default=1280, help="分辨率宽度")
    parser.add_argument("--height", "-H", type=int, default=480, help="分辨率高度")
    parser.add_argument("--fps", "-f", type=int, default=30, help="帧率")
    parser.add_argument("--save", "-s", action="store_true", help="启用保存图像功能")
    parser.add_argument("--save-dir", type=str, help="保存图像的目录")
    parser.add_argument("--no-info", action="store_true", help="隐藏信息叠加层")
    
    args = parser.parse_args()
    
    # 列出设备
    if args.list:
        list_video_devices()
        return
    
    # 测试单个摄像头
    if args.device:
        test_camera(args.device, args.width, args.height, args.fps, not args.no_info, args.save, args.save_dir)
        return
    
    # 默认行为：列出设备并测试第一个可用设备
    devices = list_video_devices()
    if devices:
        print(f"自动测试第一个设备: {devices[0]}")
        test_camera(devices[0], args.width, args.height, args.fps, not args.no_info, args.save, args.save_dir)
    
if __name__ == "__main__":
    main() 