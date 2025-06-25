#!/usr/bin/env python3
import numpy as np
import rospy
import tf.transformations
from math import pi, cos, sin, atan2, sqrt
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from arm_trajectory.srv import PlanTrajectory, PlanTrajectoryRequest, PlanTrajectoryResponse
from arm_trajectory.msg import TrajectoryPath, TrajectoryPoint

# 导入鲸鱼优化器
try:
    # 直接导入本地文件
    import sys
    import os
    # 获取当前脚本路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # 将当前目录添加到sys.path
    if current_dir not in sys.path:
        sys.path.append(current_dir)
    # 直接导入本地文件
    from whales_optimizer import WhaleOptimizer, forward_kinematics_dh
except ImportError as e:
    rospy.logerr(f"导入WhaleOptimizer失败: {e}")
    sys.exit(1)

class CoordinateSystem:
    """机械臂工作空间坐标系统"""
    
    def __init__(self):
        # 基础坐标系参数
        self.base_frame = "base_link"
        self.world_frame = "world"
        
        # 工作空间边界（单位：厘米）
        self.x_min = -50.0
        self.x_max = 50.0
        self.y_min = -50.0
        self.y_max = 50.0
        self.z_min = 0.0
        self.z_max = 50.0
        
        # 工作区域划分（网格大小，单位：厘米）
        self.grid_size = 5.0
        
        # 放置区坐标
        self.placement_areas = {
            "area_1": {"center": [30, 30, 5], "size": [15, 15, 0.1]},
            "area_2": {"center": [-30, 30, 5], "size": [15, 15, 0.1]},
            "area_3": {"center": [0, -30, 5], "size": [15, 15, 0.1]}
        }
        
        # 障碍物列表
        self.obstacles = []
        
        # 初始化可视化发布器
        self.marker_pub = rospy.Publisher('/workspace_visualization', MarkerArray, queue_size=1)
        
        # 初始化坐标系变换
        self.tf_camera_to_base = np.eye(4)  # 相机到机械臂基座的变换
        self.setup_transforms()
        
        rospy.loginfo("初始化工作空间坐标系")
    
    def setup_transforms(self):
        """设置相机到机械臂基座的坐标变换"""
        # 这里的变换参数需要通过相机标定获得
        # 假设相机放置在机械臂前方约50cm处，高度30cm
        camera_translation = np.array([50, 0, 30])  # 单位：厘米
        
        # 相机朝向机械臂，所以旋转180度
        roll, pitch, yaw = 0, 0, pi
        rotation_matrix = tf.transformations.euler_matrix(roll, pitch, yaw)
        
        # 构建变换矩阵
        self.tf_camera_to_base = np.eye(4)
        self.tf_camera_to_base[:3, :3] = rotation_matrix[:3, :3]
        self.tf_camera_to_base[:3, 3] = camera_translation
        
        rospy.loginfo("已设置相机到机械臂基座的变换")
    
    def camera_to_base_coords(self, camera_coords):
        """将相机坐标系中的点转换到机械臂基座坐标系"""
        # 输入点的齐次坐标
        point_cam = np.array([camera_coords[0], camera_coords[1], camera_coords[2], 1.0])
        
        # 应用变换
        point_base = self.tf_camera_to_base @ point_cam
        
        return point_base[:3]  # 返回非齐次坐标
    
    def base_to_camera_coords(self, base_coords):
        """将机械臂基座坐标系中的点转换到相机坐标系"""
        # 输入点的齐次坐标
        point_base = np.array([base_coords[0], base_coords[1], base_coords[2], 1.0])
        
        # 计算逆变换
        tf_base_to_camera = np.linalg.inv(self.tf_camera_to_base)
        
        # 应用变换
        point_cam = tf_base_to_camera @ point_base
        
        return point_cam[:3]  # 返回非齐次坐标
    
    def visualize_workspace(self):
        """可视化工作空间和放置区域"""
        marker_array = MarkerArray()
        
        # 添加工作空间边界
        workspace_marker = Marker()
        workspace_marker.header.frame_id = self.base_frame
        workspace_marker.header.stamp = rospy.Time.now()
        workspace_marker.ns = "workspace"
        workspace_marker.id = 0
        workspace_marker.type = Marker.CUBE
        workspace_marker.action = Marker.ADD
        
        # 设置工作空间大小和位置
        workspace_marker.pose.position.x = (self.x_min + self.x_max) / 2 / 100.0  # 转换为米
        workspace_marker.pose.position.y = (self.y_min + self.y_max) / 2 / 100.0  
        workspace_marker.pose.position.z = (self.z_min + self.z_max) / 2 / 100.0
        workspace_marker.pose.orientation.w = 1.0
        
        workspace_marker.scale.x = (self.x_max - self.x_min) / 100.0  # 转换为米
        workspace_marker.scale.y = (self.y_max - self.y_min) / 100.0
        workspace_marker.scale.z = (self.z_max - self.z_min) / 100.0
        
        # 设置半透明的蓝色
        workspace_marker.color.r = 0.0
        workspace_marker.color.g = 0.5
        workspace_marker.color.b = 1.0
        workspace_marker.color.a = 0.2
        
        marker_array.markers.append(workspace_marker)
        
        # 添加放置区
        for i, (area_id, area) in enumerate(self.placement_areas.items()):
            area_marker = Marker()
            area_marker.header.frame_id = self.base_frame
            area_marker.header.stamp = rospy.Time.now()
            area_marker.ns = "placement_areas"
            area_marker.id = i + 1
            area_marker.type = Marker.CUBE
            area_marker.action = Marker.ADD
            
            # 设置放置区位置和大小
            area_marker.pose.position.x = area["center"][0] / 100.0  # 转换为米
            area_marker.pose.position.y = area["center"][1] / 100.0
            area_marker.pose.position.z = area["center"][2] / 100.0
            area_marker.pose.orientation.w = 1.0
            
            area_marker.scale.x = area["size"][0] / 100.0  # 转换为米
            area_marker.scale.y = area["size"][1] / 100.0
            area_marker.scale.z = area["size"][2] / 100.0
            
            # 设置放置区颜色
            area_marker.color.r = 0.2
            area_marker.color.g = 0.8
            area_marker.color.b = 0.2
            area_marker.color.a = 0.7
            
            marker_array.markers.append(area_marker)
        
        # 添加障碍物
        for i, obstacle in enumerate(self.obstacles):
            obs_marker = Marker()
            obs_marker.header.frame_id = self.base_frame
            obs_marker.header.stamp = rospy.Time.now()
            obs_marker.ns = "obstacles"
            obs_marker.id = i + 100  # 避免ID重复
            obs_marker.type = Marker.CUBE
            obs_marker.action = Marker.ADD
            
            # 设置障碍物位置和大小
            obs_marker.pose.position.x = obstacle["center"][0] / 100.0  # 转换为米
            obs_marker.pose.position.y = obstacle["center"][1] / 100.0
            obs_marker.pose.position.z = obstacle["center"][2] / 100.0
            obs_marker.pose.orientation.w = 1.0
            
            obs_marker.scale.x = obstacle["size"][0] / 100.0  # 转换为米
            obs_marker.scale.y = obstacle["size"][1] / 100.0
            obs_marker.scale.z = obstacle["size"][2] / 100.0
            
            # 设置障碍物颜色
            obs_marker.color.r = 0.8
            obs_marker.color.g = 0.2
            obs_marker.color.b = 0.2
            obs_marker.color.a = 0.7
            
            marker_array.markers.append(obs_marker)
        
        # 发布标记数组
        self.marker_pub.publish(marker_array)
        rospy.loginfo("已发布工作空间可视化标记")

class PathPlanner:
    """机械臂路径规划器"""
    
    def __init__(self, coordinate_system, arm_config):
        """
        初始化路径规划器
        
        Args:
            coordinate_system: 坐标系统对象
            arm_config: 机械臂配置参数
        """
        self.coord_system = coordinate_system
        self.arm_config = arm_config
        
        # 初始化鲸鱼优化器，使用正确的参数
        self.optimizer = WhaleOptimizer(
            num_whales=arm_config["optimizer"]["population_size"],
            max_iter=arm_config["optimizer"]["max_iterations"],
            forward_kinematics_func=lambda joints: forward_kinematics_dh(joints, arm_config["dh_params"]),
            joint_limits=arm_config["joint_limits"]
        )
        
        # 初始化路径可视化发布器
        self.path_marker_pub = rospy.Publisher('/path_visualization', MarkerArray, queue_size=1)
        
        # 订阅可视化命令
        rospy.Subscriber('/path_planner/visualize_workspace', Bool, self.handle_visualize_workspace)
        
        # 吸盘末端执行器参数（单位：厘米）
        self.gripper_length = 5.0
        self.approach_distance = 10.0  # 接近距离
        
        rospy.loginfo("路径规划器初始化完成")
    
    def inverse_kinematics(self, target_pose, initial_joints=None):
        """求解逆运动学"""
        if initial_joints is None:
            initial_joints = self.arm_config['default_pose']
        
        # 使用鲸鱼优化算法求解
        best_joints, fitness, _ = self.optimizer.optimize(target_pose, initial_joints)
        return best_joints, fitness
    
    def plan_path(self, start_joints, target_pose, steps=10, avoid_obstacles=True):
        """
        规划从起始关节角度到目标位姿的路径
        
        Args:
            start_joints: 起始关节角度
            target_pose: 目标位姿 (4x4变换矩阵)
            steps: 路径点数量
            avoid_obstacles: 是否避障
            
        Returns:
            path: 路径点列表，每个点是一组关节角度
        """
        # 求解目标关节角度
        target_joints, fitness = self.inverse_kinematics(target_pose)
        
        if fitness > 0.1:  # 阈值可调整
            rospy.logwarn(f"无法找到合适的逆运动学解决方案，误差: {fitness}")
            return None
        
        # 创建路径
        path = []
        for i in range(steps + 1):
            alpha = i / steps
            # 线性插值
            joints = (1 - alpha) * np.array(start_joints) + alpha * np.array(target_joints)
            
            # 如果需要避障，检查是否与障碍物碰撞
            if avoid_obstacles and self.check_collision(joints):
                rospy.logwarn(f"路径点 {i} 与障碍物碰撞，尝试寻找替代路径")
                # 简单的避障策略：稍微提高路径点高度
                joints[2] += 5.0  # 假设关节2控制高度
                
                # 再次检查碰撞
                if self.check_collision(joints):
                    rospy.logerr("无法找到无碰撞路径")
                    return None
            
            path.append(joints.tolist())
        
        # 可视化路径
        self.visualize_path(path)
        
        return path
    
    def check_collision(self, joints):
        """检查给定关节角度是否与障碍物碰撞"""
        # 简化的碰撞检测，实际应用中应该使用完整的碰撞检测算法
        # 这里仅作为示例
        
        # 使用正向运动学计算机械臂位姿
        arm_pose = forward_kinematics_dh(joints, self.arm_config['dh_params'])
        arm_position = arm_pose[:3, 3]
        
        # 检查是否与任何障碍物碰撞
        for obstacle in self.coord_system.obstacles:
            obs_center = np.array(obstacle["center"])
            obs_size = np.array(obstacle["size"]) / 2  # 半尺寸
            
            # 简单的AABB碰撞检测
            if (abs(arm_position[0] - obs_center[0]) < obs_size[0] and
                abs(arm_position[1] - obs_center[1]) < obs_size[1] and
                abs(arm_position[2] - obs_center[2]) < obs_size[2]):
                return True  # 碰撞
        
        return False  # 无碰撞
    
    def visualize_path(self, path):
        """可视化规划路径"""
        marker_array = MarkerArray()
        
        # 添加路径点
        for i, joints in enumerate(path):
            # 计算路径点的笛卡尔坐标
            pose = forward_kinematics_dh(joints, self.arm_config['dh_params'])
            position = pose[:3, 3]
            
            # 创建球体标记
            point_marker = Marker()
            point_marker.header.frame_id = self.coord_system.base_frame
            point_marker.header.stamp = rospy.Time.now()
            point_marker.ns = "path_points"
            point_marker.id = i
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            
            # 设置位置
            point_marker.pose.position.x = position[0] / 100.0  # 转换为米
            point_marker.pose.position.y = position[1] / 100.0
            point_marker.pose.position.z = position[2] / 100.0
            point_marker.pose.orientation.w = 1.0
            
            # 设置大小
            point_marker.scale.x = 0.02  # 2cm直径
            point_marker.scale.y = 0.02
            point_marker.scale.z = 0.02
            
            # 设置颜色
            point_marker.color.r = 1.0
            point_marker.color.g = 0.5
            point_marker.color.b = 0.0
            point_marker.color.a = 0.8
            
            marker_array.markers.append(point_marker)
            
            # 添加路径线段（除第一个点外）
            if i > 0:
                prev_pose = forward_kinematics_dh(path[i-1], self.arm_config['dh_params'])
                prev_position = prev_pose[:3, 3]
                
                # 创建线段标记
                line_marker = Marker()
                line_marker.header.frame_id = self.coord_system.base_frame
                line_marker.header.stamp = rospy.Time.now()
                line_marker.ns = "path_lines"
                line_marker.id = i
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                
                # 添加线段的起点和终点
                p1 = Point()
                p1.x = prev_position[0] / 100.0  # 转换为米
                p1.y = prev_position[1] / 100.0
                p1.z = prev_position[2] / 100.0
                
                p2 = Point()
                p2.x = position[0] / 100.0  # 转换为米
                p2.y = position[1] / 100.0
                p2.z = position[2] / 100.0
                
                line_marker.points.append(p1)
                line_marker.points.append(p2)
                
                # 设置线宽
                line_marker.scale.x = 0.01  # 1cm宽
                
                # 设置颜色
                line_marker.color.r = 0.0
                line_marker.color.g = 0.8
                line_marker.color.b = 0.8
                line_marker.color.a = 0.8
                
                marker_array.markers.append(line_marker)
        
        # 发布标记数组
        self.path_marker_pub.publish(marker_array)
        rospy.loginfo(f"已发布包含 {len(path)} 个点的路径可视化")

    def handle_visualize_workspace(self, msg):
        """处理可视化工作空间命令"""
        if msg.data:
            rospy.loginfo("接收到可视化工作空间命令")
            self.coord_system.visualize_workspace()

class TaskPlanner:
    """任务规划器，用于生成抓取和放置任务"""
    
    def __init__(self, coord_system, path_planner):
        """
        初始化任务规划器
        
        Args:
            coord_system: 坐标系统对象
            path_planner: 路径规划器对象
        """
        self.coord_sys = coord_system
        self.path_planner = path_planner
        
        # 初始化任务可视化发布器
        self.task_pub = rospy.Publisher('/task_visualization', MarkerArray, queue_size=1)
        
        rospy.loginfo("初始化任务规划器")
    
    def plan_pick_task(self, object_pose, current_joints):
        """
        规划抓取任务
        
        Args:
            object_pose: 物体位姿 (4x4变换矩阵)
            current_joints: 当前关节角度
            
        Returns:
            task: 任务字典，包含接近、抓取和撤离阶段的路径
        """
        rospy.loginfo("规划抓取任务")
        
        # 提取物体位置
        object_position = object_pose[:3, 3]
        
        # 1. 计算接近位姿（在物体上方）
        approach_pose = object_pose.copy()
        approach_pose[2, 3] += self.path_planner.approach_distance  # 抬高Z轴
        
        # 2. 计算抓取位姿（添加吸盘偏移）
        grasp_pose = object_pose.copy()
        
        # 3. 计算撤离位姿（与接近位姿相同）
        retreat_pose = approach_pose.copy()
        
        # 规划接近阶段路径
        rospy.loginfo("规划接近阶段路径")
        approach_path = self.path_planner.plan_path(current_joints, approach_pose)
        if approach_path is None:
            rospy.logerr("无法规划接近阶段路径")
            return None
        
        # 规划抓取阶段路径
        rospy.loginfo("规划抓取阶段路径")
        grasp_path = self.path_planner.plan_path(approach_path[-1], grasp_pose)
        if grasp_path is None:
            rospy.logerr("无法规划抓取阶段路径")
            return None
        
        # 规划撤离阶段路径
        rospy.loginfo("规划撤离阶段路径")
        retreat_path = self.path_planner.plan_path(grasp_path[-1], retreat_pose)
        if retreat_path is None:
            rospy.logerr("无法规划撤离阶段路径")
            return None
        
        # 构建完整任务
        task = {
            "type": "pick",
            "phases": {
                "approach": approach_path,
                "grasp": grasp_path,
                "retreat": retreat_path
            },
            "object_pose": object_pose
        }
        
        # 可视化任务
        self.visualize_task(task)
        
        return task
    
    def plan_place_task(self, place_position, current_joints, object_held=True):
        """
        规划放置任务
        
        Args:
            place_position: 放置位置 [x, y, z]
            current_joints: 当前关节角度
            object_held: 是否已持有物体
            
        Returns:
            task: 任务字典，包含接近、放置和撤离阶段的路径
        """
        if not object_held:
            rospy.logwarn("未持有物体，无法执行放置任务")
            return None
        
        rospy.loginfo("规划放置任务")
        
        # 创建放置位姿
        place_pose = np.eye(4)
        place_pose[:3, 3] = place_position
        
        # 接近位姿（在放置位置上方）
        approach_pose = place_pose.copy()
        approach_pose[2, 3] += self.path_planner.approach_distance  # 抬高Z轴
        
        # 撤离位姿（与接近位姿相同）
        retreat_pose = approach_pose.copy()
        
        # 规划接近阶段路径
        rospy.loginfo("规划接近阶段路径")
        approach_path = self.path_planner.plan_path(current_joints, approach_pose)
        if approach_path is None:
            rospy.logerr("无法规划接近阶段路径")
            return None
        
        # 规划放置阶段路径
        rospy.loginfo("规划放置阶段路径")
        place_path = self.path_planner.plan_path(approach_path[-1], place_pose)
        if place_path is None:
            rospy.logerr("无法规划放置阶段路径")
            return None
        
        # 规划撤离阶段路径
        rospy.loginfo("规划撤离阶段路径")
        retreat_path = self.path_planner.plan_path(place_path[-1], retreat_pose)
        if retreat_path is None:
            rospy.logerr("无法规划撤离阶段路径")
            return None
        
        # 构建完整任务
        task = {
            "type": "place",
            "phases": {
                "approach": approach_path,
                "place": place_path,
                "retreat": retreat_path
            },
            "place_position": place_position
        }
        
        # 可视化任务
        self.visualize_task(task)
        
        return task
    
    def visualize_task(self, task):
        """可视化任务规划"""
        marker_array = MarkerArray()
        
        # 添加任务类型文本标记
        text_marker = Marker()
        text_marker.header.frame_id = self.coord_sys.base_frame
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "task_type"
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # 设置文本位置
        if task["type"] == "pick":
            position = task["object_pose"][:3, 3]
        else:  # place
            position = task["place_position"]
        
        text_marker.pose.position.x = position[0] / 100.0  # 转换为米
        text_marker.pose.position.y = position[1] / 100.0
        text_marker.pose.position.z = (position[2] / 100.0) + 0.1  # 稍微抬高
        text_marker.pose.orientation.w = 1.0
        
        # 设置文本内容和样式
        text_marker.text = f"Task: {task['type'].upper()}"
        text_marker.scale.z = 0.05  # 文本大小
        
        # 设置文本颜色
        if task["type"] == "pick":
            text_marker.color.r = 1.0
            text_marker.color.g = 0.5
            text_marker.color.b = 0.0
        else:  # place
            text_marker.color.r = 0.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.5
        text_marker.color.a = 1.0
        
        marker_array.markers.append(text_marker)
        
        # 发布标记数组
        self.task_pub.publish(marker_array)
        rospy.loginfo(f"已发布任务可视化: {task['type']}")

def plan_trajectory_service(req, path_planner, task_planner):
    """处理轨迹规划服务请求"""
    rospy.loginfo(f"收到轨迹规划请求: {req.arm_id}")
    
    # 创建响应对象
    response = PlanTrajectoryResponse()
    
    try:
        # 获取当前关节状态
        # 这里应该从关节状态话题获取，但为了简单起见，我们使用默认值
        current_joints = [0, 0, 0, 0, pi/2, 10]  # 默认关节角度
        
        # 创建目标位姿
        target_pose = np.eye(4)
        target_pose[:3, 3] = [
            req.target_pose.position.x,
            req.target_pose.position.y,
            req.target_pose.position.z
        ]
        
        # 从四元数获取旋转矩阵
        q = [
            req.target_pose.orientation.x,
            req.target_pose.orientation.y,
            req.target_pose.orientation.z,
            req.target_pose.orientation.w
        ]
        
        if all(v == 0 for v in q[:3]) and q[3] == 0:
            # 如果四元数全为0，设置默认朝下的姿态
            q = [0, 0, 0, 1]
        
        rot_matrix = tf.transformations.quaternion_matrix(q)
        target_pose[:3, :3] = rot_matrix[:3, :3]
        
        # 规划路径
        path = path_planner.plan_path(
            current_joints,
            target_pose,
            steps=20,
            avoid_obstacles=req.avoid_obstacles
        )
        
        if path:
            # 创建轨迹路径消息
            trajectory_path = TrajectoryPath()
            trajectory_path.header.stamp = rospy.Time.now()
            trajectory_path.header.frame_id = "base_link"
            trajectory_path.arm_id = req.arm_id
            trajectory_path.execution_time = req.execution_time
            
            # 添加路径点
            for i, joints in enumerate(path):
                point = TrajectoryPoint()
                point.joint_positions = joints
                
                # 计算位姿
                pose_matrix = forward_kinematics_dh(joints, path_planner.arm_config["dh_params"])
                
                point.pose.position.x = pose_matrix[0, 3]
                point.pose.position.y = pose_matrix[1, 3]
                point.pose.position.z = pose_matrix[2, 3]
                
                # 从旋转矩阵计算四元数
                q = tf.transformations.quaternion_from_matrix(pose_matrix)
                point.pose.orientation.x = q[0]
                point.pose.orientation.y = q[1]
                point.pose.orientation.z = q[2]
                point.pose.orientation.w = q[3]
                
                # 设置时间戳
                point.time_from_start = rospy.Duration(req.execution_time * i / len(path))
                
                trajectory_path.points.append(point)
            
            # 设置响应
            response.success = True
            response.message = f"成功规划包含 {len(path)} 个点的轨迹"
            response.trajectory_path = trajectory_path
            
            # 可视化路径
            path_planner.visualize_path(path)
        else:
            response.success = False
            response.message = "无法找到有效路径"
    
    except Exception as e:
        response.success = False
        response.message = f"轨迹规划失败: {str(e)}"
    
    return response

def main():
    """主函数"""
    rospy.init_node('path_planner_node')
    
    # 加载参数
    try:
        # 尝试从参数服务器获取参数
        arm_config_file = rospy.get_param('~arm_config', 'arm_params.yaml')
    except:
        # 如果失败，使用默认值
        arm_config_file = 'arm_params.yaml'
        rospy.logwarn(f"无法从参数服务器获取配置，使用默认配置: {arm_config_file}")
    
    # 创建坐标系统
    coord_system = CoordinateSystem()
    
    # 加载机械臂配置
    arm_config = {
        "dh_params": [
            # a, alpha, d, theta
            [0, pi/2, 0, 0],  # 关节1 - 底座旋转
            [30, 0, 0, 0],    # 关节2 - 伸缩关节
            [0, pi/2, 0, 0],  # 关节3 - 肩部关节
            [30, 0, 0, 0],    # 关节4 - 肘部关节
            [0, pi/2, 0, 0],  # 关节5 - 腕部旋转
            [0, 0, 10, 0]     # 关节6 - 末端伸缩
        ],
        "joint_limits": [
            # min, max (弧度或厘米)
            [-pi, pi],      # 关节1 - 底座旋转 (-180° ~ 180°)
            [0, 43],        # 关节2 - 伸缩关节 (0 ~ 43cm)
            [-pi/2, pi/2],  # 关节3 - 肩部关节 (-90° ~ 90°)
            [0, pi],        # 关节4 - 肘部关节 (0° ~ 180°)
            [-pi, pi],      # 关节5 - 腕部旋转 (-180° ~ 180°)
            [5, 15]         # 关节6 - 末端伸缩 (5 ~ 15cm)
        ],
        "optimizer": {
            "max_iterations": 50,
            "population_size": 30,
            "spiral_constant": 1.0
        }
    }
    
    # 创建路径规划器
    path_planner = PathPlanner(coord_system, arm_config)
    
    # 创建任务规划器
    task_planner = TaskPlanner(coord_system, path_planner)
    
    # 可视化工作空间
    coord_system.visualize_workspace()
    
    # 创建路径规划服务
    service = rospy.Service('/plan_trajectory', PlanTrajectory, 
                           lambda req: plan_trajectory_service(req, path_planner, task_planner))
    
    rospy.loginfo("路径规划服务已启动")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("路径规划器已关闭")

if __name__ == "__main__":
    main() 