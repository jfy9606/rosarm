#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Launch arguments
    use_gui = LaunchConfiguration('use_gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_vision = LaunchConfiguration('use_vision', default='true')
    hardware_config = LaunchConfiguration('hardware_config', default='config/hardware_config.yaml')
    
    # Declare launch arguments
    declare_use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value=use_gui,
        description='Whether to launch the GUI')
        
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value=use_rviz,
        description='Whether to launch RViz')
        
    declare_use_vision_arg = DeclareLaunchArgument(
        'use_vision',
        default_value=use_vision,
        description='Whether to launch vision nodes')
    
    declare_hardware_config_arg = DeclareLaunchArgument(
        'hardware_config',
        default_value=hardware_config,
        description='Path to the hardware configuration file')
    
    # Get hardware config path
    config_file = PathJoinSubstitution([
        FindPackageShare('arm_bringup'),
        hardware_config
    ])
    
    # Include motor launch
    motor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('motor'),
                'launch',
                'motor_control.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': config_file
        }.items()
    )
    
    # Include servo launch
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('servo'),
                'launch',
                'servo_nodes.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': config_file
        }.items()
    )
    
    # Include trajectory launch
    trajectory_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('trajectory'),
                'launch',
                'trajectory.launch.py'
            ])
        ])
    )
    
    # Include vision launch (conditional)
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vision'),
                'launch',
                'vision.launch.py'
            ])
        ]),
        condition=LaunchConfiguration('use_vision')
    )
    
    # Include GUI launch (conditional)
    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gui'),
                'launch',
                'gui.launch.py'
            ])
        ]),
        condition=LaunchConfiguration('use_gui')
    )
    
    # RViz configuration
    rviz_config = PathJoinSubstitution([
        FindPackageShare('arm_bringup'),
        'config',
        'arm.rviz'
    ])
    
    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=LaunchConfiguration('use_rviz')
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_gui_arg,
        declare_use_rviz_arg,
        declare_use_vision_arg,
        declare_hardware_config_arg,
        
        # Launch files
        motor_launch,
        servo_launch,
        trajectory_launch,
        vision_launch,
        gui_launch,
        
        # Nodes
        rviz_node
    ]) 