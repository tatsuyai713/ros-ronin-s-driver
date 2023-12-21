from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('debug_print', default_value='false'),  # 直接パラメータを定義
        Node(
            package='ros_ronin_s_driver',
            executable='ronin_s_driver',
            name='ronin_s_driver',
            output='screen',
            parameters=[
                {'debug_print': LaunchConfiguration('debug_print')},  # パラメータ1の型を整数に変換
            ],
        )
    ])