#!/usr/bin/env python3
import rospy
import numpy as np
import tf.transformations
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from arm_trajectory.srv import PlanTrajectory, PlanTrajectoryRequest, PlanTrajectoryResponse
from arm_trajectory.msg import TrajectoryPath, TrajectoryPoint

class VisionArmBridge:
    """桥接视觉系统和机械臂控制的节点"""
    
    def __init__(self):
        """初始化桥接节点"""
        rospy.init_node('vision_arm_bridge')
        
        # 相机到机械臂基座的变换矩阵
        self.tf_camera_to_base = np.eye(4)
        
        # 检测到的物体
        self.detected_objects = {}
        
        # 当前规划的路径
        self.current_path = None
        
        # 当前任务状态
        self.current_task = None
        self.task_in_progress = False
        
        # 设置相机到机械臂基座的变换
        self.setup_transforms()
        
        # 订阅检测到的物体位姿
        rospy.Subscriber('/detections/poses', PoseArray, self.handle_detection_poses)
        
        # 订阅命令
        rospy.Subscriber('/arm/command', String, self.handle_command)
        
        # 发布可视化标记
        self.marker_pub = rospy.Publisher('/detection_markers', MarkerArray, queue_size=10)
        
        # 发布路径可视化
        self.path_marker_pub = rospy.Publisher('/path_visualization', MarkerArray, queue_size=10)
        
        # 发布任务状态
        self.task_status_pub = rospy.Publisher('/arm/task_status', String, queue_size=10)
        
        # 创建轨迹规划服务客户端
        rospy.wait_for_service('/plan_trajectory')
        self.plan_trajectory = rospy.ServiceProxy('/plan_trajectory', PlanTrajectory)
        
        # 创建路径执行发布器
        self.path_pub = rospy.Publisher('/arm/trajectory_path', TrajectoryPath, queue_size=10)
        
        rospy.loginfo("视觉-机械臂桥接节点初始化完成")
    
    def setup_transforms(self):
        """设置坐标变换"""
        # 这里需要根据实际安装的位置进行标定
        # 假设相机位于机械臂基座前方50厘米，高度30厘米
        camera_translation = np.array([50.0, 0.0, 30.0])  # 单位：厘米
        
        # 相机朝向机械臂基座，所以需要绕Y轴旋转180度
        roll, pitch, yaw = 0.0, np.pi, 0.0
        rotation_matrix = tf.transformations.euler_matrix(roll, pitch, yaw)
        
        # 构建变换矩阵
        self.tf_camera_to_base = np.eye(4)
        self.tf_camera_to_base[:3, :3] = rotation_matrix[:3, :3]
        self.tf_camera_to_base[:3, 3] = camera_translation
        
        rospy.loginfo("已设置相机到机械臂基座的变换矩阵")
    
    def camera_to_base_coords(self, camera_coords):
        """将相机坐标系中的点转换到机械臂基座坐标系"""
        # 输入点的齐次坐标
        point_cam = np.array([camera_coords[0], camera_coords[1], camera_coords[2], 1.0])
        
        # 应用变换
        point_base = self.tf_camera_to_base @ point_cam
        
        return point_base[:3]  # 返回非齐次坐标
    
    def handle_detection_poses(self, msg):
        """处理检测到的物体位姿"""
        # 清空之前的检测
        self.detected_objects = {}
        
        # 转换检测到的物体位姿到机械臂基座坐标系
        for i, pose in enumerate(msg.poses):
            # 提取相机坐标系中的位置
            camera_position = np.array([pose.position.x, pose.position.y, pose.position.z])
            
            # 转换到机械臂基座坐标系
            base_position = self.camera_to_base_coords(camera_position)
            
            # 创建新的位姿
            base_pose = Pose()
            base_pose.position.x = base_position[0]
            base_pose.position.y = base_position[1]
            base_pose.position.z = base_position[2]
            
            # 保持原始姿态（四元数）
            base_pose.orientation = pose.orientation
            
            # 存储转换后的位姿
            object_id = f"object_{i}"
            self.detected_objects[object_id] = base_pose
            
            rospy.loginfo(f"检测到物体 {object_id}，相机坐标: [{camera_position[0]:.2f}, {camera_position[1]:.2f}, {camera_position[2]:.2f}]，"
                         f"机械臂坐标: [{base_position[0]:.2f}, {base_position[1]:.2f}, {base_position[2]:.2f}]")
        
        # 可视化检测到的物体
        self.visualize_detected_objects()
    
    def visualize_detected_objects(self):
        """可视化检测到的物体"""
        marker_array = MarkerArray()
        
        for i, (object_id, pose) in enumerate(self.detected_objects.items()):
            # 创建物体标记
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "detected_objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # 设置位置和姿态
            marker.pose = pose
            
            # 设置大小（假设是10x10x10厘米的立方体）
            marker.scale.x = 0.1  # 10厘米转换为米
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # 设置颜色（红色，半透明）
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            
            marker_array.markers.append(marker)
            
            # 创建文本标记
            text_marker = Marker()
            text_marker.header.frame_id = "base_link"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "object_labels"
            text_marker.id = i + 100  # 避免ID冲突
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            # 设置位置（在物体上方）
            text_marker.pose = pose
            text_marker.pose.position.z += 0.1  # 在物体上方10厘米
            
            # 设置文本内容和大小
            text_marker.text = object_id
            text_marker.scale.z = 0.05  # 文本大小
            
            # 设置颜色（白色）
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            marker_array.markers.append(text_marker)
        
        # 发布标记
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
    
    def handle_command(self, msg):
        """处理命令"""
        cmd = msg.data.strip().lower()
        parts = cmd.split()
        
        if len(parts) < 1:
            return
        
        if parts[0] == "pick" and len(parts) >= 2:
            # 格式: "pick object_id"
            object_id = parts[1]
            self.execute_pick_command(object_id)
            
        elif parts[0] == "place" and len(parts) >= 4:
            # 格式: "place x y z"
            try:
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                self.execute_place_command(x, y, z)
            except ValueError:
                rospy.logerr("无效的放置坐标")
                
        elif parts[0] == "scan":
            # 扫描场景中的物体
            self.execute_scan_command()
            
        elif parts[0] == "plan" and len(parts) >= 3:
            # 格式: "plan object_id placement_area"
            object_id = parts[1]
            placement_area = parts[2]
            self.execute_plan_command(object_id, placement_area)
            
        elif parts[0] == "execute":
            # 执行当前规划的路径
            self.execute_path_command()
            
        elif parts[0] == "visualize_workspace":
            # 可视化工作空间
            self.execute_visualize_workspace_command()
            
        else:
            rospy.logwarn(f"未知命令: {cmd}")
    
    def execute_pick_command(self, object_id):
        """执行抓取命令"""
        if object_id not in self.detected_objects:
            rospy.logerr(f"未检测到物体: {object_id}")
            return
        
        rospy.loginfo(f"执行抓取命令: {object_id}")
        
        # 获取物体位姿
        target_pose = self.detected_objects[object_id]
        
        # 创建轨迹规划请求
        req = PlanTrajectoryRequest()
        req.arm_id = "arm1"  # 机械臂ID
        req.target_pose = target_pose
        req.execution_time = 5.0  # 执行时间（秒）
        req.avoid_obstacles = True
        
        try:
            # 调用轨迹规划服务
            response = self.plan_trajectory(req)
            
            if response.success:
                rospy.loginfo(f"成功规划抓取轨迹: {response.message}")
                # 这里可以添加实际的轨迹执行代码
            else:
                rospy.logerr(f"轨迹规划失败: {response.message}")
                
        except rospy.ServiceException as e:
            rospy.logerr(f"轨迹规划服务调用失败: {e}")
    
    def execute_place_command(self, x, y, z):
        """执行放置命令"""
        rospy.loginfo(f"执行放置命令: 位置=[{x}, {y}, {z}]")
        
        # 创建目标位姿
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0  # 默认朝下的姿态
        
        # 创建轨迹规划请求
        req = PlanTrajectoryRequest()
        req.arm_id = "arm1"  # 机械臂ID
        req.target_pose = target_pose
        req.execution_time = 5.0  # 执行时间（秒）
        req.avoid_obstacles = True
        
        try:
            # 调用轨迹规划服务
            response = self.plan_trajectory(req)
            
            if response.success:
                rospy.loginfo(f"成功规划放置轨迹: {response.message}")
                # 这里可以添加实际的轨迹执行代码
            else:
                rospy.logerr(f"轨迹规划失败: {response.message}")
                
        except rospy.ServiceException as e:
            rospy.logerr(f"轨迹规划服务调用失败: {e}")
    
    def execute_scan_command(self):
        """执行扫描命令"""
        rospy.loginfo("执行扫描命令")
        
        # 发布任务状态
        self.task_status_pub.publish(String(data="scanning"))
        
        # 这里可以添加实际的扫描逻辑
        # 例如：移动机械臂到扫描位置，启用视觉检测等
        
        # 如果需要，可以请求YOLO检测器启动
        try:
            rospy.wait_for_service('/yolo/control', timeout=2.0)
            yolo_control = rospy.ServiceProxy('/yolo/control', SetBool)
            yolo_control(True)  # 启用YOLO检测
            rospy.loginfo("已启用YOLO检测")
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logwarn(f"无法启用YOLO检测: {e}")
        
        # 发布任务状态
        self.task_status_pub.publish(String(data="scan_complete"))
    
    def execute_plan_command(self, object_id, placement_area):
        """执行路径规划命令"""
        rospy.loginfo(f"执行路径规划命令: 物体={object_id}, 放置区={placement_area}")
        
        # 检查物体是否存在
        if object_id not in self.detected_objects:
            rospy.logerr(f"未检测到物体: {object_id}")
            return
        
        # 发布任务状态
        self.task_status_pub.publish(String(data="planning"))
        
        # 获取物体位姿
        pick_pose = self.detected_objects[object_id]
        
        # 获取放置区位置
        # 这里应该从配置或参数服务器获取放置区位置
        placement_positions = {
            "area_1": [30, 30, 5],
            "area_2": [-30, 30, 5],
            "area_3": [0, -30, 5]
        }
        
        if placement_area not in placement_positions:
            rospy.logerr(f"未知的放置区: {placement_area}")
            return
        
        place_position = placement_positions[placement_area]
        
        # 创建放置位姿
        place_pose = Pose()
        place_pose.position.x = place_position[0]
        place_pose.position.y = place_position[1]
        place_pose.position.z = place_position[2]
        place_pose.orientation.w = 1.0  # 默认朝下的姿态
        
        # 创建轨迹规划请求 - 抓取路径
        pick_req = PlanTrajectoryRequest()
        pick_req.arm_id = "arm1"
        pick_req.target_pose = pick_pose
        pick_req.execution_time = 5.0
        pick_req.avoid_obstacles = True
        
        # 创建轨迹规划请求 - 放置路径
        place_req = PlanTrajectoryRequest()
        place_req.arm_id = "arm1"
        place_req.target_pose = place_pose
        place_req.execution_time = 5.0
        place_req.avoid_obstacles = True
        
        try:
            # 规划抓取路径
            pick_response = self.plan_trajectory(pick_req)
            
            if not pick_response.success:
                rospy.logerr(f"抓取路径规划失败: {pick_response.message}")
                return
            
            # 规划放置路径
            place_response = self.plan_trajectory(place_req)
            
            if not place_response.success:
                rospy.logerr(f"放置路径规划失败: {place_response.message}")
                return
            
            # 组合路径
            self.current_path = {
                "pick": pick_response.trajectory_path,
                "place": place_response.trajectory_path,
                "object_id": object_id,
                "placement_area": placement_area
            }
            
            rospy.loginfo("路径规划成功")
            
            # 可视化路径
            self.visualize_path(self.current_path)
            
            # 发布任务状态
            self.task_status_pub.publish(String(data="plan_complete"))
            
        except rospy.ServiceException as e:
            rospy.logerr(f"轨迹规划服务调用失败: {e}")
    
    def execute_path_command(self):
        """执行当前规划的路径"""
        if not self.current_path:
            rospy.logerr("没有可执行的路径")
            return
        
        if self.task_in_progress:
            rospy.logwarn("任务正在执行中")
            return
        
        rospy.loginfo("开始执行路径")
        
        # 发布任务状态
        self.task_status_pub.publish(String(data="executing"))
        self.task_in_progress = True
        
        # 执行抓取路径
        rospy.loginfo("执行抓取路径")
        self.path_pub.publish(self.current_path["pick"])
        
        # 等待路径执行完成
        rospy.sleep(5.0)  # 这里应该使用服务或动作来等待实际完成
        
        # 执行抓取动作
        rospy.loginfo(f"抓取物体: {self.current_path['object_id']}")
        # 这里应该添加实际的抓取代码
        
        # 执行放置路径
        rospy.loginfo("执行放置路径")
        self.path_pub.publish(self.current_path["place"])
        
        # 等待路径执行完成
        rospy.sleep(5.0)  # 这里应该使用服务或动作来等待实际完成
        
        # 执行放置动作
        rospy.loginfo(f"放置物体到: {self.current_path['placement_area']}")
        # 这里应该添加实际的放置代码
        
        # 发布任务状态
        self.task_status_pub.publish(String(data="execute_complete"))
        self.task_in_progress = False
        
        rospy.loginfo("路径执行完成")
    
    def execute_visualize_workspace_command(self):
        """执行可视化工作空间命令"""
        rospy.loginfo("执行可视化工作空间命令")
        
        # 这里可以发布一个特殊的消息，让路径规划器可视化工作空间
        # 或者直接在这里实现可视化逻辑
        
        # 示例：发布一个特殊命令
        visualize_pub = rospy.Publisher('/path_planner/visualize_workspace', Bool, queue_size=1)
        visualize_pub.publish(Bool(data=True))
    
    def visualize_path(self, path_data):
        """可视化规划的路径"""
        if not path_data:
            return
        
        marker_array = MarkerArray()
        
        # 可视化抓取路径
        pick_path = path_data["pick"]
        self.add_path_markers(marker_array, pick_path, "pick_path", 0, [0, 0, 1, 0.8])  # 蓝色
        
        # 可视化放置路径
        place_path = path_data["place"]
        self.add_path_markers(marker_array, place_path, "place_path", 100, [0, 1, 0, 0.8])  # 绿色
        
        # 发布标记
        if marker_array.markers:
            self.path_marker_pub.publish(marker_array)
    
    def add_path_markers(self, marker_array, path, ns, id_offset, color):
        """添加路径标记到标记数组"""
        # 添加路径线
        line_marker = Marker()
        line_marker.header.frame_id = "base_link"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = ns
        line_marker.id = id_offset
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        
        # 设置线条属性
        line_marker.scale.x = 0.01  # 线宽1厘米
        line_marker.color.r = color[0]
        line_marker.color.g = color[1]
        line_marker.color.b = color[2]
        line_marker.color.a = color[3]
        
        # 添加路径点
        for point in path.points:
            p = Point()
            p.x = point.pose.position.x
            p.y = point.pose.position.y
            p.z = point.pose.position.z
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # 添加路径点标记
        for i, point in enumerate(path.points):
            point_marker = Marker()
            point_marker.header.frame_id = "base_link"
            point_marker.header.stamp = rospy.Time.now()
            point_marker.ns = ns + "_points"
            point_marker.id = id_offset + i + 1
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            
            # 设置点的位置
            point_marker.pose = point.pose
            
            # 设置点的大小
            point_marker.scale.x = 0.02  # 2厘米直径
            point_marker.scale.y = 0.02
            point_marker.scale.z = 0.02
            
            # 设置点的颜色
            point_marker.color.r = color[0]
            point_marker.color.g = color[1]
            point_marker.color.b = color[2]
            point_marker.color.a = color[3]
            
            marker_array.markers.append(point_marker)

def main():
    """主函数"""
    bridge = VisionArmBridge()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    
    rospy.loginfo("视觉-机械臂桥接节点已关闭")

if __name__ == '__main__':
    main() 