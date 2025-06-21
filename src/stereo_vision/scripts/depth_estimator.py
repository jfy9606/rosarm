#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import torch
import os

class DepthEstimator:
    def __init__(self):
        rospy.init_node('depth_estimator', anonymous=True)
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 获取参数
        self.input_topic = rospy.get_param('~input_topic', '/stereo_camera/image_processed')
        self.output_topic = rospy.get_param('~output_topic', '/stereo_camera/depth')
        self.point_cloud_topic = rospy.get_param('~point_cloud_topic', '/stereo_camera/points')
        
        # 相机内参
        self.camera_info = None
        self.camera_info_sub = rospy.Subscriber('/usb_cam/camera_info', CameraInfo, self.camera_info_callback)
        
        # 发布器
        self.depth_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.point_cloud_pub = rospy.Publisher(self.point_cloud_topic, PointCloud2, queue_size=1)
        
        # 初始化model_loaded标志
        self.model_loaded = False
        self.midas = None
        self.transform = None
        
        # 尝试加载MiDaS深度估计模型
        try:
            # 使用MiDaS小型模型进行深度估计
            self.midas = torch.hub.load("intel-isl/MiDaS", "MiDaS_small")
            if torch.cuda.is_available():
                self.midas.to('cuda')
            else:
                self.midas.to('cpu')
            self.midas.eval()
            
            # 输入变换
            self.midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
            self.transform = self.midas_transforms.small_transform
            
            rospy.loginfo("成功加载MiDaS深度估计模型")
            self.model_loaded = True
        except Exception as e:
            rospy.logerr(f"加载深度估计模型失败: {e}")
            self.model_loaded = False
            
            # 尝试使用备用方法
            rospy.loginfo("尝试使用OpenCV进行深度估计...")
            try:
                # 使用OpenCV的立体匹配算法作为备用
                self.stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
                rospy.loginfo("已初始化OpenCV立体匹配算法")
            except Exception as e2:
                rospy.logerr(f"无法初始化OpenCV立体匹配: {e2}")
        
        # 订阅图像
        self.image_sub = rospy.Subscriber(self.input_topic, Image, self.image_callback)
        
        rospy.loginfo("深度估计器已初始化")
    
    def camera_info_callback(self, data):
        """接收相机内参"""
        self.camera_info = data
        rospy.loginfo("接收到相机内参")
    
    def image_callback(self, data):
        if not self.model_loaded and not hasattr(self, 'stereo'):
            rospy.logerr_once("深度估计模型未加载，无法生成深度图")
            return
            
        if self.camera_info is None:
            rospy.logwarn_once("未接收到相机内参，使用默认值")
            
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 深度图生成
            if self.model_loaded:
                # 使用MiDaS进行深度估计
                depth = self.estimate_depth_with_midas(cv_image)
            elif hasattr(self, 'stereo'):
                # 使用OpenCV进行深度估计
                depth = self.estimate_depth_with_opencv(cv_image)
            else:
                rospy.logerr_once("没有可用的深度估计方法")
                return
            
            # 创建伪彩色深度图
            depth_colormap = cv2.applyColorMap(depth, cv2.COLORMAP_JET)
            
            # 发布深度图
            depth_msg = self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8")
            depth_msg.header = data.header
            self.depth_pub.publish(depth_msg)
            
            # 生成点云
            if self.camera_info is not None:
                point_cloud = self.generate_point_cloud(depth, cv_image, self.camera_info)
                self.point_cloud_pub.publish(point_cloud)
            
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge错误: {e}")
        except Exception as e:
            rospy.logerr(f"处理图像时出错: {e}")
    
    def estimate_depth_with_midas(self, image):
        """使用MiDaS模型估计深度"""
        try:
            # 准备输入
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            input_batch = self.transform(image).to(device)
            
            with torch.no_grad():
                prediction = self.midas(input_batch)
                
                # 调整大小以匹配输入
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=image.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
                
            # 转换为numpy数组
            depth = prediction.cpu().numpy()
            
            # 归一化深度图到0-255范围
            depth_min = depth.min()
            depth_max = depth.max()
            if depth_max > depth_min:
                depth = 255 * (depth - depth_min) / (depth_max - depth_min)
            depth = depth.astype(np.uint8)
            
            return depth
        except Exception as e:
            rospy.logerr(f"MiDaS深度估计失败: {e}")
            return np.zeros(image.shape[:2], dtype=np.uint8)
    
    def estimate_depth_with_opencv(self, image):
        """使用OpenCV立体匹配算法估计深度"""
        try:
            # 将图像转换为灰度
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 如果是立体图像，尝试分割
            height, width = gray.shape
            if width > 2 * height:
                # 可能是并排的立体图像
                mid = width // 2
                left = gray[:, :mid]
                right = gray[:, mid:]
                
                # 计算视差图
                disparity = self.stereo.compute(left, right)
                
                # 归一化到0-255
                disparity_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                return disparity_normalized
            else:
                # 单目图像，无法进行立体匹配
                rospy.logwarn_once("单目图像无法使用立体匹配算法")
                # 返回灰度图作为伪深度图
                return gray
        except Exception as e:
            rospy.logerr(f"OpenCV深度估计失败: {e}")
            return np.zeros(image.shape[:2], dtype=np.uint8)
    
    def generate_point_cloud(self, depth_image, color_image, camera_info):
        """从深度图和彩色图生成点云"""
        # 获取相机内参
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]
        
        # 创建点云
        height, width = depth_image.shape
        points = []
        
        # 深度缩放因子 (将深度值转换为实际距离，单位：米)
        # 这里假设深度图已经归一化到0-255，需要将其转换回实际深度
        # 假设最大深度为10米
        depth_scale = 10.0 / 255.0
        
        # 生成点云
        for v in range(0, height, 2):  # 隔行采样以减少点云大小
            for u in range(0, width, 2):  # 隔列采样
                z = depth_image[v, u] * depth_scale  # 深度值
                if z > 0:
                    # 计算实际3D坐标
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    
                    # 获取颜色
                    b, g, r = color_image[v, u]
                    
                    # RGB颜色打包为一个整数
                    rgb = (r << 16) | (g << 8) | b
                    
                    # 添加点
                    points.append([x, y, z, rgb])
        
        # 创建PointCloud2消息
        header = camera_info.header
        fields = [
            pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
            pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
            pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
            pc2.PointField('rgb', 12, pc2.PointField.UINT32, 1)
        ]
        
        pc = pc2.create_cloud(header, fields, points)
        return pc

if __name__ == '__main__':
    try:
        estimator = DepthEstimator()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("用户中断，退出")
    except Exception as e:
        rospy.logerr(f"未知错误: {e}") 