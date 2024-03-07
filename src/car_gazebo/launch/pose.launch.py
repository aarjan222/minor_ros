import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('car_gazebo')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf_param.yaml'),
                    {'use_sim_time': use_sim_time}])

    return LaunchDescription([
        robot_localization_node,
    ])
