import os

from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import xacro


def generate_launch_description():
    car_desc_dir = FindPackageShare(
        package='car_body').find('car_body')
    car_gazebo_dir = FindPackageShare(
        package='car_gazebo').find('car_gazebo')
    default_controller_yaml_file = os.path.join(
        car_gazebo_dir, 'config/ackermann_controller.yaml')

    # Create launch configuration variables
    params_file = LaunchConfiguration('params_file')
    xacro_file = os.path.join(car_desc_dir,'description','vehicle.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file)

    # robot_description = Command(['xacro ', os.path.join(car_desc_dir, 'description/vehicle.urdf.xacro'),
    #                             ' car_controller_yaml_file:=', params_file])

    # Define launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file', default_value=default_controller_yaml_file,
        description='Absolute path to controller yaml config file')

    # # Define actions
    start_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        )

    params = {'robot_description': robot_description_raw.toxml()}
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
        )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen')

    load_ackermann_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'ackermann_drive_base_controller'],
        output='screen')

    # Declare event handlers
    load_ackermann_drive_base_ctrl_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_ackermann_drive_base_controller,
            on_exit=[load_ackermann_drive_base_controller]))
    
    load_joint_state_ctrl_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_joint_state_controller]))

    gamepad = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                'car_gazebo'), 'launch', 'joystick.launch.py'
        )]))

    base_laser = Node(
        package='car_gazebo', executable='base_link_to_laser_frame.py', output='screen'
    )

    fake_encoder_odom_topic = Node(
        package='car_gazebo', executable='fake_encoder_odom_topic.py', output='screen')

    map_base_link = Node(
        package='car_gazebo', executable='map_to_base_link.py', output='screen')

    odom_base_link = Node(
        package='car_gazebo', executable='odom_to_base_link.py', output='screen')

    imu = Node(package='car_gazebo',
               executable='imu_mobile_server.py', output='screen')
    
    car_control = Node(package='car_gazebo', executable='car_teleop.py', output='screen')
    pose_receiver= Node(package='car_gazebo', executable='pose_receiver.py', output='screen')

    return LaunchDescription([
        declare_params_file_cmd,
        start_joint_state_publisher,
        start_robot_state_publisher,
        gamepad,
        load_ackermann_drive_base_controller,
        load_joint_state_controller,
        base_laser,
        fake_encoder_odom_topic,
        odom_base_link,
        map_base_link,
        imu,
        car_control,
        pose_receiver,
    ])
