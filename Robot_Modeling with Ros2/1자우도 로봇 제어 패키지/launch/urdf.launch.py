from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    rviz_config_file = '/home/chichi/urdf_example/src/urdf_example/rviz/urdf_rviz.rviz'

    urdf_file = '/home/chichi/urdf_example/src/urdf_example/urdf/robot3.urdf'

    doc = xacro.process_file(urdf_file)
    robot_description = doc.toxml()

    return LaunchDescription([
        # robot_state_publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # joint_state_publisher_gui node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),

        # Rviz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )

    ])