#! /usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('car_gazebo'))

    # Base_link to laser_frame tranformation
    baseLink_laserFrame_tf = Node(
        package='car_gazebo',
        executable='base_link_to_laser_frame.py',
        output='screen'
    )
    # Odom_Frame to Base_Footprint transformation
    odom_baseLink_tf = Node(
        package='car_gazebo',
        executable='odom_to_base_link.py',
        output='screen'
    )

    map_baseLink_tf = Node(
        package='car_gazebo',
        executable='map_to_base_link.py',
        output='screen'
    )

    return LaunchDescription([
        baseLink_laserFrame_tf,
        odom_baseLink_tf,
        map_baseLink_tf
    ])
