import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('car_body'))
    xacro_file = os.path.join(pkg_path, 'description', 'car.urdf.xacro')
    car_description_raw = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {'robot_description': car_description_raw.toxml()}
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher', 
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
