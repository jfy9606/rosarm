#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from std_msgs.msg import String
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from arm_trajectory.msg import TrajectoryPath, TrajectoryPoint
from arm_trajectory.srv import PlanTrajectory, PlanTrajectoryRequest, PlanTrajectoryResponse
import sys
import os
import math
import random

# 通过环境变量获取Python路径
# 获取PYTHONPATH环境变量并将其添加到sys.path
if 'PYTHONPATH' in os.environ:
    env_paths = os.environ['PYTHONPATH'].split(':')
    for path in env_paths:
        if path and path not in sys.path and os.path.exists(path):
            sys.path.append(path)
            rospy.loginfo(f"从PYTHONPATH添加路径: {path}")

# 获取脚本的目录并添加到sys.path
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)
    rospy.loginfo(f"添加脚本目录到路径: {script_dir}")

# 添加当前工作目录
cwd = os.getcwd()
if cwd not in sys.path:
    sys.path.append(cwd)
    rospy.loginfo(f"添加当前工作目录到路径: {cwd}")

# 尝试导入WhaleOptimizer
try:
    # 直接从当前目录导入
    import sys
    import os
    
    # 获取当前脚本目录路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 将当前目录添加到Python路径
    if current_dir not in sys.path:
        sys.path.append(current_dir)
    
    # 直接从文件导入类
    from whales_optimizer import WhaleOptimizer, forward_kinematics_dh
    rospy.loginfo("成功导入WhaleOptimizer")
except ImportError as e:
    rospy.logerr(f"导入WhaleOptimizer失败: {e}")
    rospy.loginfo("将使用备选优化方法")
    
    # 定义简单的替代优化器
    class SimpleOptimizer:
        """
        当无法导入WhaleOptimizer时使用
        """
        def __init__(self, num_whales=30, dim=6, max_iter=100, lb=None, ub=None, robot=None, target_T=None):
            self.dim = dim
            self.max_iter = max_iter
            self.lb = np.array(lb) if lb is not None else np.array([-math.pi, 0, -math.pi/2, 0, math.pi/2, 5])
            self.ub = np.array(ub) if ub is not None else np.array([math.pi, 43, math.pi/2, math.pi, math.pi/2, 15])
            self.robot = robot
            self.target_T = target_T
            self.best_position = None
            self.best_fitness = float('inf')
        
        def calculate_fitness(self, position):
            """计算适应度"""
            if self.robot is not None:
                T = self.robot.fkine(position)
            else:
                T = forward_kinematics_dh(position)
            
            position_error = np.linalg.norm(T[:3, 3] - self.target_T[:3, 3])
            rotation_error = np.linalg.norm(T[:3, :3] - self.target_T[:3, :3], 'fro')
            return 0.7 * position_error + 0.3 * rotation_error
        
        def run(self):
            """运行优化"""
            # 简单随机搜索
            best_position = np.zeros(self.dim)
            best_fitness = float('inf')
            
            for _ in range(self.max_iter * 10):  # 增加迭代次数以提高成功率
                # 随机生成一个位置
                position = np.array([random.uniform(self.lb[i], self.ub[i]) for i in range(self.dim)])
                
                # 计算适应度
                fitness = self.calculate_fitness(position)
                
                # 更新最佳位置
                if fitness < best_fitness:
                    best_fitness = fitness
                    best_position = position.copy()
            
            self.best_position = best_position
            self.best_fitness = best_fitness
            return best_position, best_fitness
        
        def getBestPosition(self):
            """获取最佳位置"""
            return self.best_position.tolist() if self.best_position is not None else None
        
        def getBestFitness(self):
            """获取最佳适应度"""
            return self.best_fitness
    
    # 使用替代优化器
    WhaleOptimizer = SimpleOptimizer

from math import pi
import tf.transformations

# 备选的简单前向运动学函数
def simple_forward_kinematics(joint_values, dh_params=None):
    """
    简化版的前向运动学计算
    当无法导入WhaleOptimizer时使用
    """
    # DH变换矩阵计算
    def compute_dh_transform(theta, d, a, alpha):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    if dh_params is None:
        # 默认DH参数
        a1, a2, a3 = 13, 13, 25  # 杆长
        dh_params = [
            # type, d, theta, a, alpha
            ('revolute', 0, joint_values[0], a1, pi/2),  # 第一关节 (旋转)
            ('prismatic', joint_values[1], pi/4, 0, 0),   # 第二关节 (伸缩)
            ('revolute', a2, joint_values[2], 0, pi/2),  # 第三关节 (旋转)
            ('revolute', 0, joint_values[3], a3, pi/2),  # 第四关节 (旋转)
            ('revolute', 0, joint_values[4], 0, pi/2),   # 第五关节 (固定)
            ('prismatic', joint_values[5], pi/2, 0, 0)    # 第六关节 (伸缩)
        ]
    
    T = np.eye(4)
    
    for joint_type, d, theta, a, alpha in dh_params:
        # 计算此关节的变换矩阵
        transform = compute_dh_transform(theta, d, a, alpha)
        # 累积变换
        T = T @ transform
    
    return T

class ArmTrajectoryBridge:
    """Bridge node for connecting stereo vision with trajectory planning"""
    
    def __init__(self):
        rospy.init_node('arm_trajectory_bridge', anonymous=True)
        
        # Initialize robot arm parameters
        self.setup_robot_params()
        
        # Initialize TF buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Setup services
        self.plan_srv = rospy.Service('plan_arm_trajectory', PlanTrajectory, self.handle_plan_trajectory)
        
        # Publishers
        self.trajectory_pub = {}
        for arm_id in self.arm_configs:
            self.trajectory_pub[arm_id] = rospy.Publisher(
                f'/{arm_id}/trajectory_command', JointTrajectory, queue_size=1)
        
        # Subscribe to detected objects
        self.detected_poses_sub = rospy.Subscriber(
            '/stereo_vision/detected_poses', PoseArray, self.handle_detected_poses)
        
        # Subscriber for commands
        self.command_sub = rospy.Subscriber('/arm_command', String, self.handle_command)
        
        # Store detected objects and target positions
        self.detected_objects = {}
        self.target_positions = {}
        
        # Initialize the optimizer for each arm
        self.optimizers = {}
        for arm_id, config in self.arm_configs.items():
            if use_whale_optimizer:
                # 使用鲸鱼优化算法，但不传递不兼容的参数
                self.optimizers[arm_id] = WhaleOptimizer(
                    num_whales=30,
                    max_iter=100,
                    lb=[limit[0] for limit in config['joint_limits']],
                    ub=[limit[1] for limit in config['joint_limits']]
                    # 注意：WhaleOptimizer不接受forward_kinematics_func参数
                    # 它使用自己的forward_kinematics_dh函数
                )
            else:
                # 使用备选优化方法
                self.optimizers[arm_id] = SimpleOptimizer(
                    num_particles=30,
                    max_iter=100, 
                    forward_kinematics_func=lambda joints, arm=arm_id: self.forward_kinematics(joints, arm),
                    joint_limits=config['joint_limits']
                )
        
        rospy.loginfo("Trajectory Bridge Node initialized")
    
    def setup_robot_params(self):
        """Set up parameters for each robot arm in the system"""
        # This would typically be loaded from URDF or parameter server
        # For now, we'll hardcode configurations for multiple arms
        
        self.arm_configs = {
            'arm1': {
                'dh_params': [
                    ('revolute', 0, 0, 13, pi/2),        # Joint 1: (type, d, θ, a, α)
                    ('prismatic', 0, pi/4, 0, 0),        # Joint 2
                    ('revolute', 13, 0, 0, pi/2),        # Joint 3
                    ('revolute', 0, 0, 25, pi/2),        # Joint 4
                    ('revolute', 0, pi/2, 0, pi/2),      # Joint 5 (fixed)
                    ('prismatic', 0, pi/2, 0, 0)         # Joint 6
                ],
                'joint_limits': [
                    [-pi, pi],      # theta1 (rad)
                    [0, 43],        # d2 (cm)
                    [-pi/2, pi/2],  # theta3 (rad)
                    [0, pi],        # theta4 (rad)
                    [pi/2, pi/2],   # theta5 (rad, fixed)
                    [5, 15]         # d6 (cm)
                ],
                'default_pose': [0, 0, 0, 0, pi/2, 10]  # Default joint configuration
            },
            'arm2': {
                'dh_params': [
                    ('revolute', 0, 0, 13, pi/2),
                    ('prismatic', 0, pi/4, 0, 0),
                    ('revolute', 13, 0, 0, pi/2),
                    ('revolute', 0, 0, 25, pi/2),
                    ('revolute', 0, pi/2, 0, pi/2),
                    ('prismatic', 0, pi/2, 0, 0)
                ],
                'joint_limits': [
                    [-pi, pi],
                    [0, 43],
                    [-pi/2, pi/2],
                    [0, pi],
                    [pi/2, pi/2],
                    [5, 15]
                ],
                'default_pose': [0, 0, 0, 0, pi/2, 10]
            }
        }
    
    def forward_kinematics(self, joint_values, arm_id):
        """Calculate forward kinematics for the specified arm"""
        if arm_id not in self.arm_configs:
            rospy.logerr(f"Unknown arm ID: {arm_id}")
            return np.eye(4)  # Return identity matrix on error
        
        return forward_kinematics_dh(joint_values, self.arm_configs[arm_id]['dh_params'])
    
    def inverse_kinematics(self, target_pose, arm_id, initial_joints=None):
        """Use WOA to solve inverse kinematics for the specified arm"""
        if arm_id not in self.optimizers:
            rospy.logerr(f"No optimizer for arm ID: {arm_id}")
            return None, float('inf')
        
        # If no initial guess, use the default pose
        if initial_joints is None:
            initial_joints = self.arm_configs[arm_id]['default_pose']
            
        if use_whale_optimizer:
            # WhaleOptimizer需要在每次调用前设置target_T
            optimizer = self.optimizers[arm_id]
            optimizer.target_T = target_pose
            
            # 运行优化器
            best_position, best_fitness = optimizer.run()
            fitness_history = optimizer.fitness_history
        else:
            # 使用SimpleOptimizer
            best_position, best_fitness, fitness_history = self.optimizers[arm_id].optimize(target_pose, initial_joints)
        
        return best_position, best_fitness
    
    def generate_trajectory(self, start_joints, end_joints, execution_time, arm_id, num_points=50):
        """
        Generate a smooth trajectory from start to end joint positions
        using cubic interpolation
        """
        trajectory = TrajectoryPath()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.arm_id = arm_id
        trajectory.execution_time = execution_time
        
        start_joints = np.array(start_joints)
        end_joints = np.array(end_joints)
        
        # Create time points
        times = np.linspace(0, execution_time, num_points)
        dt = execution_time / (num_points - 1)
        
        # Calculate velocities at endpoints (zero for start and end)
        start_vel = np.zeros_like(start_joints)
        end_vel = np.zeros_like(end_joints)
        
        # Calculate cubic polynomial coefficients
        a0 = start_joints
        a1 = start_vel
        a2 = 3 * (end_joints - start_joints) / execution_time**2 - 2 * start_vel / execution_time - end_vel / execution_time
        a3 = 2 * (start_joints - end_joints) / execution_time**3 + (start_vel + end_vel) / execution_time**2
        
        # Generate trajectory points
        for t in times:
            # Calculate joint positions using cubic polynomial
            q = a0 + a1 * t + a2 * t**2 + a3 * t**3
            
            # Calculate joint velocities
            qd = a1 + 2 * a2 * t + 3 * a3 * t**2
            
            # Calculate joint accelerations
            qdd = 2 * a2 + 6 * a3 * t
            
            # Create a trajectory point
            point = TrajectoryPoint()
            point.header.stamp = rospy.Time.now() + rospy.Duration(t)
            point.joint_positions = q.tolist()
            point.joint_velocities = qd.tolist()
            point.joint_accelerations = qdd.tolist()
            
            # Calculate the end effector pose for this joint configuration
            T = self.forward_kinematics(q, arm_id)
            
            # Convert T to a pose
            pose = Pose()
            pose.position.x = T[0, 3]
            pose.position.y = T[1, 3]
            pose.position.z = T[2, 3]
            
            # Simple conversion of rotation matrix to quaternion
            quat = tf.transformations.quaternion_from_matrix(T)
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            
            point.pose = pose
            trajectory.points.append(point)
        
        return trajectory
    
    def trajectory_to_joint_trajectory(self, trajectory):
        """Convert our TrajectoryPath to ROS JointTrajectory message"""
        joint_traj = JointTrajectory()
        joint_traj.header = trajectory.header
        
        # Set joint names based on the arm type
        if trajectory.arm_id in self.arm_configs:
            joint_traj.joint_names = [f"{trajectory.arm_id}_joint{i+1}" for i in range(len(self.arm_configs[trajectory.arm_id]['joint_limits']))]
        
        # Convert points
        for point in trajectory.points:
            jtp = JointTrajectoryPoint()
            jtp.positions = point.joint_positions
            jtp.velocities = point.joint_velocities
            jtp.accelerations = point.joint_accelerations
            jtp.time_from_start = point.header.stamp - trajectory.header.stamp
            joint_traj.points.append(jtp)
        
        return joint_traj
    
    def handle_plan_trajectory(self, req):
        """Service handler to plan a trajectory"""
        response = PlanTrajectoryResponse()
        
        # Check if requested arm exists
        if req.arm_id not in self.arm_configs:
            response.success = False
            response.message = f"Unknown arm ID: {req.arm_id}"
            return response
        
        try:
            # Get current joint state (would normally come from joint state publisher)
            current_joints = self.arm_configs[req.arm_id]['default_pose']
            
            # Create target transformation matrix from provided pose
            target_pose = np.eye(4)
            
            # Set the position part
            target_pose[0:3, 3] = [
                req.target_pose.position.x,
                req.target_pose.position.y,
                req.target_pose.position.z
            ]
            
            # Set the orientation part (quaternion to rotation matrix)
            q = [
                req.target_pose.orientation.x,
                req.target_pose.orientation.y,
                req.target_pose.orientation.z,
                req.target_pose.orientation.w
            ]
            target_pose[0:3, 0:3] = tf.transformations.quaternion_matrix(q)[0:3, 0:3]
            
            # Compute inverse kinematics
            target_joints, fitness = self.inverse_kinematics(target_pose, req.arm_id, current_joints)
            
            if fitness > 0.1:  # Threshold for acceptable IK solution
                response.success = False
                response.message = f"Could not find a valid IK solution. Fitness: {fitness}"
                return response
            
            # Generate trajectory
            trajectory = self.generate_trajectory(
                current_joints, target_joints, req.execution_time, req.arm_id)
            
            response.success = True
            response.message = "Trajectory planning successful"
            response.trajectory = trajectory
            
            return response
            
        except Exception as e:
            response.success = False
            response.message = f"Error planning trajectory: {str(e)}"
            return response
    
    def handle_detected_poses(self, msg):
        """Handle detected object poses from stereo vision"""
        # In a real system, we'd need to identify which detection corresponds to which object
        # For now, just store all the poses and assume they're identified properly elsewhere
        self.detected_objects = {}
        
        # Store detected objects with unique IDs (just using indices for now)
        for i, pose in enumerate(msg.poses):
            object_id = f"object_{i}"
            self.detected_objects[object_id] = pose
            
            # Log detection
            rospy.loginfo(f"Detected object {object_id} at position: [{pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}]")
    
    def handle_command(self, msg):
        """Handle text commands for controlling the arms"""
        cmd = msg.data.strip().lower()
        parts = cmd.split()
        
        if len(parts) < 1:
            return
        
        if parts[0] == "pick" and len(parts) >= 3:
            # Format: "pick [arm_id] [object_id]"
            arm_id = parts[1]
            object_id = parts[2]
            self.execute_pick_command(arm_id, object_id)
            
        elif parts[0] == "place" and len(parts) >= 5:
            # Format: "place [arm_id] [x] [y] [z]"
            arm_id = parts[1]
            try:
                x = float(parts[2])
                y = float(parts[3])
                z = float(parts[4])
                self.execute_place_command(arm_id, x, y, z)
            except ValueError:
                rospy.logerr("Invalid coordinates in place command")
        
        elif parts[0] == "move_to" and len(parts) >= 4:
            # Format: "move_to [x] [y] [z]" - 直接移动到笛卡尔坐标
            try:
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                # 默认使用arm1
                arm_id = "arm1"
                self.execute_move_to_command(arm_id, x, y, z)
            except ValueError:
                rospy.logerr("Invalid coordinates in move_to command")
            
        elif parts[0] == "home" and len(parts) >= 2:
            # Format: "home [arm_id]"
            arm_id = parts[1]
            self.move_to_home(arm_id)
            
        else:
            rospy.logwarn(f"Unknown command: {cmd}")
    
    def execute_pick_command(self, arm_id, object_id):
        """Execute a pick command for the specified arm and object"""
        if arm_id not in self.arm_configs:
            rospy.logerr(f"Unknown arm ID: {arm_id}")
            return
        
        if object_id not in self.detected_objects:
            rospy.logerr(f"Unknown object ID: {object_id}")
            return
        
        # Get target pose (the detected object's position)
        target_pose = self.detected_objects[object_id]
        
        # Create service request
        req = PlanTrajectoryRequest()
        req.arm_id = arm_id
        req.target_pose = target_pose
        req.execution_time = 5.0  # 5 seconds execution time
        req.avoid_obstacles = True
        
        # Call our own service handler
        response = self.handle_plan_trajectory(req)
        
        if response.success:
            # Convert to joint trajectory and publish
            joint_traj = self.trajectory_to_joint_trajectory(response.trajectory)
            self.trajectory_pub[arm_id].publish(joint_traj)
            rospy.loginfo(f"Executing pick command for arm {arm_id} to object {object_id}")
        else:
            rospy.logerr(f"Failed to plan trajectory: {response.message}")
    
    def execute_place_command(self, arm_id, x, y, z):
        """Execute a place command for the specified arm to the given position"""
        if arm_id not in self.arm_configs:
            rospy.logerr(f"Unknown arm ID: {arm_id}")
            return
        
        # Create target pose
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0  # Default orientation (no rotation)
        
        # Create service request
        req = PlanTrajectoryRequest()
        req.arm_id = arm_id
        req.target_pose = target_pose
        req.execution_time = 5.0  # 5 seconds execution time
        req.avoid_obstacles = True
        
        # Call our own service handler
        response = self.handle_plan_trajectory(req)
        
        if response.success:
            # Convert to joint trajectory and publish
            joint_traj = self.trajectory_to_joint_trajectory(response.trajectory)
            self.trajectory_pub[arm_id].publish(joint_traj)
            rospy.loginfo(f"Executing place command for arm {arm_id} to position [{x}, {y}, {z}]")
        else:
            rospy.logerr(f"Failed to plan trajectory: {response.message}")
    
    def move_to_home(self, arm_id):
        """Move the specified arm to its home position"""
        if arm_id not in self.arm_configs:
            rospy.logerr(f"Unknown arm ID: {arm_id}")
            return
        
        # Get home joint configuration
        home_joints = self.arm_configs[arm_id]['default_pose']
        
        # Get current joint state (in a real system this would come from joint state publisher)
        current_joints = home_joints  # Placeholder, would be replaced with actual current state
        
        # Generate trajectory to home position
        trajectory = self.generate_trajectory(current_joints, home_joints, 3.0, arm_id)
        
        # Convert to joint trajectory and publish
        joint_traj = self.trajectory_to_joint_trajectory(trajectory)
        self.trajectory_pub[arm_id].publish(joint_traj)
        rospy.loginfo(f"Moving arm {arm_id} to home position")

    def execute_move_to_command(self, arm_id, x, y, z):
        """Execute a move-to command to a specific cartesian position"""
        if arm_id not in self.arm_configs:
            rospy.logerr(f"Unknown arm ID: {arm_id}")
            return
        
        # 创建目标姿态
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y 
        target_pose.position.z = z
        
        # 设置固定朝向（默认向下）
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 1.0
        
        # 创建服务请求
        req = PlanTrajectoryRequest()
        req.arm_id = arm_id
        req.target_pose = target_pose
        req.execution_time = 3.0  # 3秒执行时间
        req.avoid_obstacles = False  # 简单直线移动，不避障
        
        # 调用自己的服务处理程序
        response = self.handle_plan_trajectory(req)
        
        if response.success:
            # 转换为关节轨迹并发布
            joint_traj = self.trajectory_to_joint_trajectory(response.trajectory)
            self.trajectory_pub[arm_id].publish(joint_traj)
            rospy.loginfo(f"执行移动命令: arm={arm_id}, 位置=[{x:.3f}, {y:.3f}, {z:.3f}]")
        else:
            rospy.logerr(f"规划轨迹失败: {response.message}")

if __name__ == '__main__':
    try:
        node = ArmTrajectoryBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 