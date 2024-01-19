import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            parameters=[{
                'serial_port': '/dev/serial/by-path/...',
                'frame-id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        )
    ])
