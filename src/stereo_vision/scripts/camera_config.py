import numpy as np
import cv2

# 左相机内参矩阵 [fx, 0, cx; 0, fy, cy; 0, 0, 1]
left_camera_matrix = np.array([[708.021578084636, 0.0, 317.045973456068],
                                         [0., 708.407895199159, 246.255369290019],
                                         [0., 0., 1.]])
 
# 左相机畸变系数:[k1, k2, p1, p2, k3] (径向畸变和切向畸变)
left_distortion = np.array([[-0.423530558996000, 0.197897793816823,-0.000431572543977505, -0.00159243587269625, 0.0673104450517612]])
 
# 右相机内参矩阵
right_camera_matrix = np.array([[696.214795245276, 0.0, 332.575401506334],
                                          [0., 697.690484333038, 254.476510158679],
                                            [0., 0., 1.]])
# 右相机畸变系数:[k1, k2, p1, p2, k3]                                          
right_distortion = np.array([[-0.446997760685779, 0.334596216137961, -0.00156518872343852, 0.000556305187002041, -0.245497405770906]])
 
#  旋转矩阵 (描述右相机相对于左相机的旋转)
R = np.array([[0.999997797146582, -0.000841951628713670, 0.00192271148090808],
                           [ 0.000845960639244048, 0.999997468098366, -0.00208522120015922],
                           [ -0.00192095095740603, 0.00208684314495605, 0.999995977508464]])
 
# 平平移向量 (描述右相机相对于左相机的位置偏移，单位mm)
T = np.array([[-60.1855543356205], [0.0302533537233422], [-0.307971817904289]])

# 图像尺寸 (宽, 高)
size = (640, 480)

# 立体校正：stereoRectify计算校正参数，使左右图像行对齐
# 输入：左右相机内参、畸变系数、图像尺寸、旋转矩阵、平移向量
# 输出：校正后的旋转矩阵R1/R2、投影矩阵P1/P2、重投影矩阵Q、有效像素区域
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R,
                                                                  T)

# 计算校正映射表：initUndistortRectifyMap生成校正映射表，后续用于快速图像校正
# 用于将原始图像转换为校正后的图像
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2) 