from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_id',
            default_value='1',
            description='无人机编号。1 -> /fmu/...，2 -> /px4_1/fmu/...',
        ),
        DeclareLaunchArgument(
            'hover_thrust',
            default_value='0.7292',
            description='悬停推力 [0,1]，body 系 -Z 方向',
        ),
        DeclareLaunchArgument(
            'roll_deg',
            default_value='0.0',
            description='基础 roll 角（度）',
        ),
        DeclareLaunchArgument(
            'pitch_deg',
            default_value='0.0',
            description='基础 pitch 角（度）',
        ),
        DeclareLaunchArgument(
            'yaw_deg',
            default_value='0.0',
            description='基础 yaw 角（度）',
        ),
        DeclareLaunchArgument(
            'control_period_ms',
            default_value='20',
            description='控制周期（ms），20 -> 50Hz',
        ),
        Node(
            package='offboard_control',
            executable='attitude_control',
            name='attitude_control',
            output='screen',
            parameters=[{
                'vehicle_id': LaunchConfiguration('vehicle_id'),
                'hover_thrust': PythonExpression(["float('", LaunchConfiguration('hover_thrust'), "')"]),
                'roll_deg': PythonExpression(["float('", LaunchConfiguration('roll_deg'), "')"]),
                'pitch_deg': PythonExpression(["float('", LaunchConfiguration('pitch_deg'), "')"]),
                'yaw_deg': PythonExpression(["float('", LaunchConfiguration('yaw_deg'), "')"]),
                'control_period_ms': LaunchConfiguration('control_period_ms'),
            }],
        ),
    ])
