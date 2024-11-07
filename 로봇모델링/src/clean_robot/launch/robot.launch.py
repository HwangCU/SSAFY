from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # urdf_file = '/home/edurobot/prac_ws/src/clean_robot/urdf/clean_robot.urdf'
    urdf_file = '/home/edurobot/prac_ws/src/urdf_example/urdf/robot3.urdf'
    rviz_config_file = '/home/edurobot/prac_ws/src/clean_robot/rviz/clean_robot.rviz'

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]  # RViz 설정 파일 절대경로
        ),
    ])
