from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='interate_prac',
            executable='D435i_YOLO',
            name='D435i_YOLO',
            output='screen',
        )
    ])