from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Locate the path to realsense2_camera's rs_launch.py
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return LaunchDescription([
        Node(
            package='state_machine',
            executable='state_machine',
            name='state_machine',
            output='screen'
        ),
        Node(
            package='motion_control',
            executable='motion_control',
            name='motion_control',
            output='screen'
        ),
        Node(
            package='motor',
            executable='motor_control',
            name='motor_control',
            output='screen'
        ),
        Node(
            package='gui',
            executable='gui',
            name='gui',
            output='screen'
        ),
        Node(
            package='motor_communication',
            executable='motor_servo_ctl',
            name='motor_servo_ctl',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path)
        )
    ])
