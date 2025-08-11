from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    conf_thres = LaunchConfiguration('confidence_threshold')
    camera_topic = LaunchConfiguration('camera_topic')
    device = LaunchConfiguration('device')

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('confidence_threshold', default_value='0.5'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('device', default_value='auto'),

        Node(
            package='vision',
            executable='yolo_detection_node.py',
            name='yolo_detection_node',
            output='screen',
            parameters=[{
                'model_path': model_path,
                'confidence_threshold': conf_thres,
                'camera_topic': camera_topic,
                'device': device,
            }]
        )
    ])