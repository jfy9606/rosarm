#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

# 相机参数（这些参数应该通过相机标定获取）
# 左相机内参矩阵 [fx, 0, cx; 0, fy, cy; 0, 0, 1]
left_camera_matrix = np.array([[708.021578084636, 0.0, 317.045973456068],
                               [0., 708.407895199159, 246.255369290019],
                               [0., 0., 1.]])

# 左相机畸变系数:[k1, k2, p1, p2, k3] (径向畸变和切向畸变)
left_distortion = np.array([[-0.423530558996000, 0.197897793816823, -0.000431572543977505, -0.00159243587269625, 0.0673104450517612]])

# 右相机内参矩阵
right_camera_matrix = np.array([[696.214795245276, 0.0, 332.575401506334],
                                [0., 697.690484333038, 254.476510158679],
                                [0., 0., 1.]])
# 右相机畸变系数:[k1, k2, p1, p2, k3]                                          
right_distortion = np.array([[-0.446997760685779, 0.334596216137961, -0.00156518872343852, 0.000556305187002041, -0.245497405770906]])

# 旋转矩阵 (描述右相机相对于左相机的旋转)
R = np.array([[0.999997797146582, -0.000841951628713670, 0.00192271148090808],
              [0.000845960639244048, 0.999997468098366, -0.00208522120015922],
              [-0.00192095095740603, 0.00208684314495605, 0.999995977508464]])

# 平移向量 (描述右相机相对于左相机的位置偏移，单位mm)
T = np.array([[-60.1855543356205], [0.0302533537233422], [-0.307971817904289]])

# 图像尺寸 (宽, 高)
size = (640, 480)

class StereoCameraNode:
    """双目摄像头节点，用于处理双目图像并计算深度"""
    
    def __init__(self):
        """初始化双目摄像头节点"""
        rospy.init_node('stereo_camera_node')
        
        # 获取参数
        self.device = rospy.get_param('~device', '/dev/video0')
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 30)
        
        # 创建图像发布器
        self.left_pub = rospy.Publisher('/stereo/left/image_raw', Image, queue_size=10)
        self.right_pub = rospy.Publisher('/stereo/right/image_raw', Image, queue_size=10)
        self.depth_pub = rospy.Publisher('/stereo/depth', Image, queue_size=10)
        self.merged_pub = rospy.Publisher('/stereo_camera/image_merged', Image, queue_size=10)
        
        # 创建OpenCV桥接器
        self.bridge = CvBridge()
        
        # 初始化摄像头
        self.init_camera()
        
        # 初始化立体匹配器
        self.init_stereo_matcher()
        
        # 计算校正映射
        self.compute_rectification_maps()
        
        # 设置定时器
        self.timer = rospy.Timer(rospy.Duration(1.0/self.fps), self.timer_callback)
        
        rospy.loginfo(f"双目摄像头节点已初始化，设备: {self.device}")
    
    def init_camera(self):
        """初始化摄像头设备"""
        try:
            self.cap = cv2.VideoCapture(self.device)
            
            if not self.cap.isOpened():
                rospy.logerr(f"无法打开摄像头设备: {self.device}")
                self.use_test_image = True
                return
            
            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width * 2)  # 双目图像宽度是单目的两倍
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            
            self.use_test_image = False
            rospy.loginfo("双目摄像头已成功初始化")
            
        except Exception as e:
            rospy.logerr(f"初始化摄像头时出错: {str(e)}")
            self.use_test_image = True
    
    def init_stereo_matcher(self):
        """初始化立体匹配器"""
        # 使用SGBM算法进行立体匹配
        block_size = 8
        img_channels = 3
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=1,
            numDisparities=64,
            blockSize=block_size,
            P1=8 * img_channels * block_size * block_size,
            P2=32 * img_channels * block_size * block_size,
            disp12MaxDiff=-1,
            preFilterCap=140,
            uniquenessRatio=1,
            speckleWindowSize=100,
            speckleRange=100,
            mode=cv2.STEREO_SGBM_MODE_HH
        )
        
        rospy.loginfo("立体匹配器已初始化")
    
    def compute_rectification_maps(self):
        """计算立体校正映射"""
        # 计算立体校正参数
        R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
            left_camera_matrix, left_distortion,
            right_camera_matrix, right_distortion,
            size, R, T
        )
        
        # 计算校正映射
        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
            left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2
        )
        
        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
            right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2
        )
        
        # 保存重投影矩阵
        self.Q = Q
        
        rospy.loginfo("立体校正映射已计算")
    
    def create_test_stereo_image(self):
        """创建测试双目图像"""
        # 创建左右测试图像
        left_img = np.zeros((self.height, self.width, 3), np.uint8)
        right_img = np.zeros((self.height, self.width, 3), np.uint8)
        
        # 左图像添加一些形状
        cv2.circle(left_img, (int(self.width/4), int(self.height/2)), 50, (0, 0, 255), -1)
        cv2.rectangle(left_img, (int(self.width/2), int(self.height/4)), 
                     (int(3*self.width/4), int(3*self.height/4)), (0, 255, 0), -1)
        
        # 右图像添加相似但稍微偏移的形状
        cv2.circle(right_img, (int(self.width/4)-10, int(self.height/2)), 50, (0, 0, 255), -1)
        cv2.rectangle(right_img, (int(self.width/2)-10, int(self.height/4)), 
                     (int(3*self.width/4)-10, int(3*self.height/4)), (0, 255, 0), -1)
        
        # 添加文本
        cv2.putText(left_img, "Left Test", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(right_img, "Right Test", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # 合并左右图像
        merged_img = np.hstack((left_img, right_img))
        
        return left_img, right_img, merged_img
    
    def timer_callback(self, event):
        """定时器回调函数，用于获取图像并发布"""
        if self.use_test_image:
            # 使用测试图像
            left_img, right_img, merged_img = self.create_test_stereo_image()
        else:
            # 从摄像头获取图像
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logwarn("无法获取图像，使用测试图像代替")
                left_img, right_img, merged_img = self.create_test_stereo_image()
            else:
                # 分割左右图像
                left_img = frame[:, :self.width]
                right_img = frame[:, self.width:2*self.width]
                merged_img = frame
        
        try:
            # 校正图像
            left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
            
            left_rectified = cv2.remap(left_gray, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
            right_rectified = cv2.remap(right_gray, self.right_map1, self.right_map2, cv2.INTER_LINEAR)
            
            # 计算视差图
            disparity = self.stereo.compute(left_rectified, right_rectified)
            
            # 归一化视差图以便可视化
            disp_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # 应用彩色映射
            disp_color = cv2.applyColorMap(disp_normalized, cv2.COLORMAP_JET)
            
            # 转换为ROS图像消息并发布
            left_msg = self.bridge.cv2_to_imgmsg(left_img, "bgr8")
            right_msg = self.bridge.cv2_to_imgmsg(right_img, "bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(disp_color, "bgr8")
            merged_msg = self.bridge.cv2_to_imgmsg(merged_img, "bgr8")
            
            # 设置时间戳和帧ID
            now = rospy.Time.now()
            left_msg.header.stamp = now
            left_msg.header.frame_id = "left_camera"
            right_msg.header.stamp = now
            right_msg.header.frame_id = "right_camera"
            depth_msg.header.stamp = now
            depth_msg.header.frame_id = "stereo_camera"
            merged_msg.header.stamp = now
            merged_msg.header.frame_id = "stereo_camera"
            
            # 发布消息
            self.left_pub.publish(left_msg)
            self.right_pub.publish(right_msg)
            self.depth_pub.publish(depth_msg)
            self.merged_pub.publish(merged_msg)
            
        except Exception as e:
            rospy.logerr(f"处理双目图像时出错: {str(e)}")
    
    def shutdown(self):
        """关闭节点"""
        if not self.use_test_image:
            self.cap.release()
        
        rospy.loginfo("双目摄像头节点已关闭")

def main():
    """主函数"""
    stereo_camera = StereoCameraNode()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    
    stereo_camera.shutdown()

if __name__ == '__main__':
    main() 