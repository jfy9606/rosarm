#!/usr/bin/env python3
import cv2
import numpy as np
import argparse
import os
import time

def capture_camera(device_id=0, width=1280, height=480, fps=30, save_path=None):
    """直接捕获摄像头图像"""
    print(f"尝试打开摄像头 {device_id} 分辨率 {width}x{height}")
    
    # 打开摄像头
    cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        print(f"无法打开摄像头 {device_id}")
        return
    
    # 设置分辨率和帧率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    
    # 尝试设置MJPG格式，提高帧率
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    
    # 获取实际设置的参数
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"摄像头实际分辨率：{actual_width}x{actual_height}，帧率：{actual_fps}")
    
    # 创建窗口
    cv2.namedWindow("摄像头图像", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("摄像头图像", actual_width, actual_height)
    
    # 如果宽高比大于1.9:1，说明可能是宽屏摄像头
    wide_screen = (actual_width >= actual_height * 1.9)
    left_image = None  # 用于保存左半部分图像
    
    if wide_screen:
        print("检测到宽幅图像，将自动处理左半部分")
        half_width = actual_width // 2
        cv2.namedWindow("处理后图像", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("处理后图像", half_width, actual_height)
    
    # 用于计算帧率
    frame_count = 0
    start_time = time.time()
    fps_display = 0
    
    print("按 'q' 退出，按 's' 保存当前帧")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("无法读取帧")
                time.sleep(0.1)
                continue
            
            # 计算亮度，检测黑帧
            mean_brightness = np.mean(frame)
            
            # 计算帧率
            frame_count += 1
            elapsed_time = time.time() - start_time
            if elapsed_time > 1.0:
                fps_display = frame_count / elapsed_time
                frame_count = 0
                start_time = time.time()
                print(f"帧率: {fps_display:.1f} FPS，亮度: {mean_brightness:.1f}")
            
            # 显示原始图像
            original_frame = frame.copy()
            cv2.putText(original_frame, f"FPS: {fps_display:.1f} 亮度: {mean_brightness:.1f}", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("摄像头图像", original_frame)
            
            # 如果是宽幅图像，处理左半部分
            if wide_screen:
                half_width = frame.shape[1] // 2
                left_image = frame[:, :half_width].copy()
                
                # 添加参考信息
                cv2.putText(left_image, f"处理后图像 {left_image.shape[1]}x{left_image.shape[0]}", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow("处理后图像", left_image)
            else:
                left_image = frame  # 如果不是宽幅，使用完整图像
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s') and save_path is not None:
                # 确保保存目录存在
                os.makedirs(save_path, exist_ok=True)
                
                # 使用时间戳作为文件名
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                
                # 保存原始图像
                full_path = os.path.join(save_path, f"camera_{timestamp}.jpg")
                cv2.imwrite(full_path, frame)
                print(f"已保存原始图像到 {full_path}")
                
                # 如果有处理后的图像，也保存它
                if left_image is not None and wide_screen:
                    processed_path = os.path.join(save_path, f"processed_{timestamp}.jpg")
                    cv2.imwrite(processed_path, left_image)
                    print(f"已保存处理后图像到 {processed_path}")
    
    except KeyboardInterrupt:
        print("用户中断，退出")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("摄像头已关闭")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="摄像头直接测试工具")
    parser.add_argument("--device", "-d", type=int, default=0, help="摄像头设备ID（整数）")
    parser.add_argument("--width", "-W", type=int, default=1280, help="分辨率宽度")
    parser.add_argument("--height", "-H", type=int, default=480, help="分辨率高度")
    parser.add_argument("--fps", "-f", type=int, default=30, help="帧率")
    parser.add_argument("--save-path", "-s", type=str, default="./camera_images", help="图像保存路径")
    
    args = parser.parse_args()
    capture_camera(args.device, args.width, args.height, args.fps, args.save_path) 