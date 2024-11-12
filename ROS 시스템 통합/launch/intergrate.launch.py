from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    dof_arg = DeclareLaunchArgument(
        'DOF', default_value='4', description= 'Degrees of freedom for the dobot magician'
    )

    tool_arg = DeclareLaunchArgument(
        'tool', default_value='suction_cup', description= 'Tool attached to dobot magician'
    )

    dobot_bringup_launch= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('/home/chichi/magician_ros2_control_system_ws/src/dobot_bringup/launch', 'dobot_magician_control_system.launch.py')
        )
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('/home/chichi/magician_ros2_control_system_ws/src/dobot_description/launch', 'display.launch.py')
        ),
        launch_arguments = {
            'DOF': LaunchConfiguration('DOF'),
            'tool': LaunchConfiguration('tool')
        }.items()
    )

    dobot_homing_service_call = TimerAction(
        period = 35.0,
        actions=[
            ExecuteProcess(
                cmd = ['ros2 ', 'service ', 'call ', '/dobot_homing_service', 'dobot_msgs/srv/ExecuteHomingProcedure'],
                shell=True,
                output='screen',
            )
        ]
    )

    ptp_move_node = TimerAction(
        period=45.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2 ','run ', 'control_dobot ', 'n1'],
                shell=True,
                output='screen',

            )
        ]
    )

    # realsense
    realsense2_launch = ExecuteProcess(
        cmd = [[
            'ros2 ', 'launch ', 'realsense2_camera ', 'rs_launch.py'
        ]],
        shell=True,
        output='screen',
    )

    rqt_image_view = ExecuteProcess(
        cmd = ['ros2 ','run ','rqt_image_view ','rqt_image_view'],
        shell = True,
        output='screen'
    )

    return LaunchDescription([
        dof_arg,
        tool_arg,
        dobot_bringup_launch,
        TimerAction(period=20.0, actions=[realsense2_launch]),
        TimerAction(period=25.0, actions=[display_launch]),
        TimerAction(period=30.0, actions=[rqt_image_view]),
        dobot_homing_service_call,
        ptp_move_node,
    ])