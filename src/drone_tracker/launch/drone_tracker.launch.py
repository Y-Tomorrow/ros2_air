from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_id',
            default_value='2',
            description='要控制的无人机编号（PX4 MAV_SYS_ID / target_system）。1->/fmu/...，2->/px4_1/fmu/...，3->/px4_2/fmu/... 以此类推',
        ),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/drone_detections',
            description='检测结果话题（vision_msgs/Detection2DArray）',
        ),
        DeclareLaunchArgument(
            'target_class',
            default_value='',
            description='目标类别名（空表示不过滤）',
        ),
        DeclareLaunchArgument(
            'image_width',
            default_value='1920',
            description='图像宽度（像素）',
        ),
        DeclareLaunchArgument(
            'hfov_deg',
            default_value='90.0',
            description='水平视场角（度）',
        ),
        DeclareLaunchArgument(
            'kp',
            default_value='0.1',
            description='比例系数（检测像素偏移 -> yaw误差后再乘kp）',
        ),
        DeclareLaunchArgument(
            'sign',
            default_value='1.0',
            description='方向因子（若方向反了改为 1.0；默认按常见PX4/NED与像素方向相反设置）',
        ),
        DeclareLaunchArgument(
            'center_deadband_px',
            default_value='1',
            description='画面中心死区（像素），小于该偏差不再调整yaw以减小抖动',
        ),
        DeclareLaunchArgument(
            'error_lowpass_alpha',
            default_value='0.6',
            description='yaw误差低通滤波系数alpha(0~1)，越大越跟得紧但越抖',
        ),
        DeclareLaunchArgument(
            'center_weight',
            default_value='0.6',
            description='选目标时“靠近中心”的权重，越大越不容易跳目标',
        ),
        DeclareLaunchArgument(
            'min_score',
            default_value='0.25',
            description='最小置信度，低于该值的检测将忽略',
        ),
        DeclareLaunchArgument(
            'hover_x',
            default_value='3.0',
            description='无人机2悬停X（NED/本地坐标，单位m）',
        ),
        DeclareLaunchArgument(
            'hover_y',
            default_value='0.0',
            description='无人机2悬停Y（单位m）',
        ),
        DeclareLaunchArgument(
            'hover_z',
            default_value='-5.0',
            description='无人机2悬停Z（NED向下为负，单位m）',
        ),
        DeclareLaunchArgument(
            'scan_rate_rad_s',
            default_value='0.8',
            description='目标丢失时原地自转角速度（rad/s）',
        ),
        DeclareLaunchArgument(
            'lost_timeout_s',
            default_value='0.35',
            description='超过该时间未检测到目标则认为丢失并恢复自转（秒）',
        ),
        DeclareLaunchArgument(
            'max_yaw_step_rad',
            default_value='0.25',
            description='每个控制周期最大偏航步长（rad）',
        ),
        Node(
            package='drone_tracker',
            executable='drone_tracker_node',
            name='drone_tracker_node',
            output='screen',
            parameters=[{
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'detections_topic': LaunchConfiguration('detections_topic'),
                'target_class': LaunchConfiguration('target_class'),
                'image_width': LaunchConfiguration('image_width'),
                'hfov_deg': LaunchConfiguration('hfov_deg'),
                'kp': LaunchConfiguration('kp'),
                'sign': LaunchConfiguration('sign'),
                'center_deadband_px': LaunchConfiguration('center_deadband_px'),
                'error_lowpass_alpha': LaunchConfiguration('error_lowpass_alpha'),
                'center_weight': LaunchConfiguration('center_weight'),
                'min_score': LaunchConfiguration('min_score'),
                'hover_x': LaunchConfiguration('hover_x'),
                'hover_y': LaunchConfiguration('hover_y'),
                'hover_z': LaunchConfiguration('hover_z'),
                'scan_rate_rad_s': LaunchConfiguration('scan_rate_rad_s'),
                'lost_timeout_s': LaunchConfiguration('lost_timeout_s'),
                'max_yaw_step_rad': LaunchConfiguration('max_yaw_step_rad'),
            }],
        ),
    ])

