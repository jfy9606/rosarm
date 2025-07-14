from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """启动轨迹和路径规划节点"""
    return LaunchDescription([
        # 运动学服务节点
        Node(
            package='trajectory',
            executable='kinematics_node_main',
            name='kinematics_node',
            output='screen',
        ),
        
        # 轨迹规划节点
        Node(
            package='trajectory',
            executable='trajectory_node.py',
            name='trajectory_node',
            output='screen',
        ),
        
        # 路径规划节点
        Node(
            package='trajectory',
            executable='path_planner_node.py',
            name='path_planner_node',
            output='screen',
        ),
    ]) 