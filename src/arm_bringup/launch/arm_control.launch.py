#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 声明启动参数
    use_gui = LaunchConfiguration('use_gui', default='true')
    use_vision = LaunchConfiguration('use_vision', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加启动参数
    ld.add_action(DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='是否启动GUI'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'use_vision',
        default_value='true',
        description='是否启动视觉系统'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否启动RViz'
    ))
    
    # 添加电机控制节点
    ld.add_action(Node(
        package='motor',
        executable='motor_node_main',
        name='motor_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'baudrate': 115200
        }]
    ))
    
    # 添加腕部控制节点
    ld.add_action(Node(
        package='servo',
        executable='wrist_node',
        name='wrist_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB1',
            'baudrate': 1000000,
            'timeout': 100
        }]
    ))
    
    # 添加真空吸盘控制节点
    ld.add_action(Node(
        package='servo',
        executable='vacuum_node',
        name='vacuum_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB1',
            'baudrate': 1000000,
            'timeout': 100,
            'vacuum_id': 10
        }]
    ))
    
    # 添加轨迹规划节点
    ld.add_action(Node(
        package='trajectory',
        executable='trajectory_planner_node_main',
        name='trajectory_planner_node',
        output='screen'
    ))
    
    # 添加路径规划节点
    ld.add_action(Node(
        package='trajectory',
        executable='path_planner_node.py',
        name='path_planner_node',
        output='screen'
    ))
    
    # 如果启用视觉，添加视觉处理节点
    # 注意：这里使用条件启动，但需要在运行时评估
    # 在实际应用中，您可能需要使用launch.conditions.IfCondition
    ld.add_action(Node(
        package='vision',
        executable='vision_node.py',
        name='vision_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_vision'))
    ))
    
    return ld