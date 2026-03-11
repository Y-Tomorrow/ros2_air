from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='model/best_v8.pt',
            description='YOLOv8 模型路径（相对工作空间根目录或绝对路径）',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_raw',
            description='输入的图像话题',
        ),
        DeclareLaunchArgument(
            'conf_threshold',
            default_value='0.25',
            description='检测置信度阈值',
        ),
        DeclareLaunchArgument(
            'publish_annotated',
            default_value='true',
            description='是否发布带框的可视化图像',
        ),
        Node(
            package='drone_detector',
            executable='drone_detector_node',
            name='drone_detector_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'image_topic': LaunchConfiguration('image_topic'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'publish_annotated': LaunchConfiguration('publish_annotated'),
            }],
        ),
    ])
