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
        self.input_topic = rospy.get_param('~input_topic', '/stereo_camera/image_merged')
        self.output_topic = rospy.get_param('~output_topic', '/stereo_camera/depth')
        self.point_cloud_topic = rospy.get_param('~point_cloud_topic', '/stereo_camera/points')
        self.use_midas = rospy.get_param('~use_midas', False)  # 默认使用OpenCV立体匹配
        
        # 相机内参
        self.camera_info = None
        self.camera_info_sub = rospy.Subscriber('/stereo_camera/camera_info', CameraInfo, self.camera_info_callback)
        
        # 发布器
        self.depth_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.point_cloud_pub = rospy.Publisher(self.point_cloud_topic, PointCloud2, queue_size=1)
        
        # 初始化model_loaded标志
        self.model_loaded = False
        self.midas = None
        self.transform = None
        
        # 初始化OpenCV立体匹配器
        self.init_stereo_matcher()
        
        # 尝试初始化WLS滤波器来改善深度图质量
        self.init_wls_filter()
        
        # 如果需要，尝试加载MiDaS深度估计模型
        if self.use_midas:
            self.load_midas_model()
        
        # 订阅图像
        self.image_sub = rospy.Subscriber(self.input_topic, Image, self.image_callback)
        
        rospy.loginfo("深度估计器已初始化")
    
    def init_stereo_matcher(self):
        """初始化OpenCV立体匹配算法"""
        try:
            # 基于示例代码优化的SGBM参数
            # 参照examples/cam/#final.py中的设置
            blockSize = 5  # 减小block size以获得更精确的边缘
            img_channels = 3
            self.stereo = cv2.StereoSGBM_create(
                minDisparity=1,
                numDisparities=64,  # 增加视差范围以检测更远的物体
                blockSize=blockSize,
                P1=8 * img_channels * blockSize * blockSize,
                P2=32 * img_channels * blockSize * blockSize,
                disp12MaxDiff=1,  # 设置为较小的正值，改善准确度
                preFilterCap=31,  # 减小预过滤器容量以减少噪声
                uniquenessRatio=10,  # 增加唯一性比率以减少误匹配
                speckleWindowSize=100,
                speckleRange=32,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY  # 使用3-way模式获得更好的质量
            )
            
            rospy.loginfo("已初始化优化的OpenCV立体匹配算法(SGBM)")
            
        except Exception as e:
            rospy.logerr(f"无法初始化OpenCV立体匹配: {e}")
            # 尝试使用更简单的StereoBM
            try:
                self.stereo = cv2.StereoBM_create(numDisparities=64, blockSize=9)
                rospy.loginfo("已初始化OpenCV立体匹配算法(BM)")
            except Exception as e2:
                rospy.logerr(f"无法初始化任何立体匹配算法: {e2}")
    
    def init_wls_filter(self):
        """初始化WLS滤波器来改善深度图"""
        self.use_wls_filter = False
        
        try:
            # 检查cv2.ximgproc模块是否可用
            if hasattr(cv2, 'ximgproc') and hasattr(cv2.ximgproc, 'createDisparityWLSFilter'):
                # 创建右侧匹配器（用于WLS滤波）
                self.right_matcher = cv2.ximgproc.createRightMatcher(self.stereo)
                
                # 创建WLS滤波器
                self.wls_filter = cv2.ximgproc.createDisparityWLSFilter(self.stereo)
                self.wls_filter.setLambda(8000.0)
                self.wls_filter.setSigmaColor(1.5)
                
                self.use_wls_filter = True
                rospy.loginfo("已初始化WLS滤波器，将用于改善深度图质量")
            else:
                rospy.logwarn("cv2.ximgproc模块不可用，不使用WLS滤波器")
        except Exception as e:
            rospy.logwarn(f"初始化WLS滤波器时出错: {e}，将不使用此功能")
    
    def load_midas_model(self):
        """加载MiDaS深度估计模型"""
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
    
    def camera_info_callback(self, data):
        """接收相机内参"""
        self.camera_info = data
        rospy.loginfo("接收到相机内参")
    
    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # 分割左右图像
            height, width = cv_image.shape[:2]
            if width > height * 1.5:  # 判断是否为合并的双目图像（OV4689，1280x480）
                # 精确划分左右图像
                mid = width // 2
                left_img = cv_image[:, :mid]
                right_img = cv_image[:, mid:]
                
                # 转换为灰度图像用于立体匹配
                left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
                right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
                
                # 预处理：直方图均衡化以提高匹配质量
                left_gray = cv2.equalizeHist(left_gray)
                right_gray = cv2.equalizeHist(right_gray)
                
                # 应用双边滤波进行降噪同时保留边缘细节
                left_gray = cv2.bilateralFilter(left_gray, 5, 75, 75)
                right_gray = cv2.bilateralFilter(right_gray, 5, 75, 75)
                
                # 使用改进的立体匹配算法计算视差
                try:
                    # 计算左右视差图
                    left_disp = self.stereo.compute(left_gray, right_gray)
                    
                    # 应用WLS滤波器进行后处理，如果可用
                    if self.use_wls_filter:
                        try:
                            # 计算右视差图
                            right_disp = self.right_matcher.compute(right_gray, left_gray)
                            
                            # 应用WLS滤波器
                            filtered_disp = self.wls_filter.filter(
                                disparity_map_left=left_disp, 
                                left_view=left_gray,
                                right_view=right_gray,
                                disparity_map_right=right_disp
                            )
                            
                            # 使用过滤后的视差
                            disparity = filtered_disp
                            rospy.loginfo_once("使用WLS滤波器处理视差图")
                        except Exception as wls_error:
                            rospy.logwarn_once(f"WLS滤波处理失败，回退到原始视差: {wls_error}")
                            disparity = left_disp
                    else:
                        disparity = left_disp
                    
                    # 应用中值滤波去除噪点
                    disparity_filtered = cv2.medianBlur(disparity, 5)
                    
                    # 归一化视差图到0-255
                    disparity_normalized = cv2.normalize(disparity_filtered, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    
                    # 创建伪彩色深度图
                    depth_colormap = cv2.applyColorMap(disparity_normalized, cv2.COLORMAP_JET)
                    
                    # 在深度图上覆盖原图轮廓，增强可视化效果
                    overlay = left_img.copy()
                    edges = cv2.Canny(left_img, 100, 200)
                    edges_colored = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
                    # 将边缘叠加到深度图上
                    alpha = 0.7
                    depth_with_edges = cv2.addWeighted(depth_colormap, alpha, edges_colored, 0.3, 0)
                    
                    # 计算实际深度值
                    # 假设我们有相机基线baseline和焦距focal_length
                    # 如果相机信息可用，使用实际的相机参数
                    baseline = 0.075  # 假设的相机基线，单位：米
                    if self.camera_info is not None:
                        focal_length = self.camera_info.K[0]  # 假设fx是焦距
                    else:
                        focal_length = 1000.0  # 假设的焦点，单位：像素
                    
                    # 视差到深度的转换
                    # 深度 = baseline * focal_length / disparity
                    # 避免除以零，给disparity设置一个最小值
                    min_disparity = 1.0
                    
                    # 创建深度图
                    depth_map = np.zeros_like(disparity_filtered, dtype=np.float32)
                    valid_disparities = disparity_filtered > min_disparity
                    depth_map[valid_disparities] = baseline * focal_length / disparity_filtered[valid_disparities]
                    
                    # 限制深度范围，滤除不合理的深度值
                    max_depth = 10.0  # 最大深度10米
                    depth_map[depth_map > max_depth] = max_depth
                    
                    # 发布深度图
                    depth_msg = self.bridge.cv2_to_imgmsg(depth_with_edges, "bgr8")
                    depth_msg.header = data.header
                    self.depth_pub.publish(depth_msg)
                    
                    # 生成点云
                    if self.camera_info is not None:
                        point_cloud = self.generate_point_cloud(depth_map, left_img, self.camera_info)
                        self.point_cloud_pub.publish(point_cloud)
                except Exception as e:
                    rospy.logerr(f"立体匹配错误: {e}")
                    # 发布空深度图或错误指示
                    error_img = np.zeros((height, mid, 3), dtype=np.uint8)
                    cv2.putText(error_img, "深度估计失败", (30, height//2), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    depth_msg = self.bridge.cv2_to_imgmsg(error_img, "bgr8")
                    depth_msg.header = data.header
                    self.depth_pub.publish(depth_msg)
                
            else:
                # 如果不是双目图像，使用MiDaS单目深度估计（如果可用）
                if self.use_midas and self.model_loaded:
                    depth = self.estimate_depth_with_midas(cv_image)
                    
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
                else:
                    rospy.logwarn_once("非双目图像且MiDaS不可用，无法估计深度")
            
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
    
    def generate_point_cloud(self, depth_image, color_image, camera_info):
        """从深度图和彩色图生成点云"""
        # 获取相机内参
        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]
        
        # 创建点云
        height, width = depth_image.shape[:2]
        points = []
        
        # 深度采样因子，减少点云密度
        sample_factor = 4
        
        # 生成点云
        for v in range(0, height, sample_factor):  # 隔行采样以减少点云大小
            for u in range(0, width, sample_factor):  # 隔列采样
                z = depth_image[v, u]  # 深度值（米）
                if z > 0.1 and z < 10.0:  # 忽略太近或太远的点
                    # 计算实际3D坐标
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    
                    # 获取颜色
                    if len(color_image.shape) == 3:  # 彩色图像
                        b, g, r = color_image[v, u]
                    else:  # 灰度图像
                        b = g = r = color_image[v, u]
                    
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