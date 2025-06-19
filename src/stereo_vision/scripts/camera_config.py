import numpy as np
import cv2

# 左相机内参矩阵
left_camera_matrix = np.array([
    [718.856, 0, 607.192],
    [0, 718.856, 185.215],
    [0, 0, 1]
], dtype=np.float32)

# 左相机畸变系数
left_distortion = np.array([0.1115, -0.1089, 0, 0, 0], dtype=np.float32)

# 右相机内参矩阵
right_camera_matrix = np.array([
    [718.856, 0, 607.192],
    [0, 718.856, 185.215],
    [0, 0, 1]
], dtype=np.float32)

# 右相机畸变系数
right_distortion = np.array([0.1115, -0.1089, 0, 0, 0], dtype=np.float32)

# 旋转矩阵
R = np.array([
    [0.9999, 0.0106, -0.0049],
    [-0.0107, 0.9999, -0.0088],
    [0.0048, 0.0089, 0.9999]
], dtype=np.float32)

# 平移向量
T = np.array([-120.3, 0.0, -0.3], dtype=np.float32)

# 图像尺寸
size = (640, 480)

# 立体校正
R1, R2, P1, P2, Q, validROI1, validROI2 = cv2.stereoRectify(
    left_camera_matrix, left_distortion,
    right_camera_matrix, right_distortion,
    size, R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
)

# 计算映射矩阵
left_map1, left_map2 = cv2.initUndistortRectifyMap(
    left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2
)
right_map1, right_map2 = cv2.initUndistortRectifyMap(
    right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2
) 