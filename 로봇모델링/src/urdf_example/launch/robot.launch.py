# File: robot_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your URDF file
    urdf_file = '/home/edurobot/prac_ws/src/urdf_example/urdf/robot3.urdf'
    rviz_config_file = '/home/edurobot/prac_ws/src/urdf_example/rviz/prac2.rviz'

    # Read the URDF file content
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Robot state publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # Joint state publisher GUI node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
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

