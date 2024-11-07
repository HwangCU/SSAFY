from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    rviz_config_file = '/home/chichi/practice_ws/src/rs_d435/rviz/rs_rviz.rviz'


    return LaunchDescription([
        # robot_state_publisher node
        Node(
            package='rs_d435',
            executable='test',
            output='screen',
        ),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            output='screen',
            parameters=[{
                'topic: /camera/color/image_raw'
            }]
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