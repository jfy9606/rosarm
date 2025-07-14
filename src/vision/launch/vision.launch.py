#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    camera_topic = LaunchConfiguration('camera_topic', default='/camera/image_raw')
    camera_info_topic = LaunchConfiguration('camera_info_topic', default='/camera/camera_info')
    detection_topic = LaunchConfiguration('detection_topic', default='/vision/detections')
    object_topic = LaunchConfiguration('object_topic', default='/vision/objects')
    debug_image = LaunchConfiguration('debug_image', default='true')
    
    # Declare launch arguments
    declare_camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value=camera_topic,
        description='Camera image topic')
        
    declare_camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value=camera_info_topic,
        description='Camera info topic')
        
    declare_detection_topic_arg = DeclareLaunchArgument(
        'detection_topic',
        default_value=detection_topic,
        description='Detection output topic')
        
    declare_object_topic_arg = DeclareLaunchArgument(
        'object_topic',
        default_value=object_topic,
        description='Object detection output topic')
        
    declare_debug_image_arg = DeclareLaunchArgument(
        'debug_image',
        default_value=debug_image,
        description='Whether to publish debug images')
    
    # Vision node
    vision_node = Node(
        package='vision',
        executable='vision_node.py',
        name='vision_node',
        output='screen',
        parameters=[{
            'camera_topic': camera_topic,
            'camera_info_topic': camera_info_topic,
            'detection_topic': detection_topic,
            'object_topic': object_topic,
            'debug_image': debug_image
        }]
    )
    
    # Object detection node
    object_detection_node = Node(
        package='vision',
        executable='object_detection_node.py',
        name='object_detection_node',
        output='screen',
        parameters=[{
            'camera_topic': camera_topic,
            'detection_topic': detection_topic,
            'debug_image': debug_image
        }]
    )
    
    return LaunchDescription([
        declare_camera_topic_arg,
        declare_camera_info_topic_arg,
        declare_detection_topic_arg,
        declare_object_topic_arg,
        declare_debug_image_arg,
        vision_node,
        object_detection_node
    ]) 