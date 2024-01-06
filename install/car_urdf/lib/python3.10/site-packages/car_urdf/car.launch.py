import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


# def generate_launch_description():
#     pkg_name = 'urdf'
#     file_subpath = 'description/car.urdf.xacro'

#     xacro_file = os.path.join(
#         get_package_share_directory(pkg_name), file_subpath)
#     car_description_raw = xacro.process_file(xacro_file).toxml

#     node_car_state_publisher = Node(
#         package='car_state_publisher',
#         executable='car_state_publisher',
#         output='screen',
#         name='carrrr',
#         parameters=[{'car_description': car_description_raw}]
#     )

#     return LaunchDescription([
#         node_car_state_publisher
# ])


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
    ])
