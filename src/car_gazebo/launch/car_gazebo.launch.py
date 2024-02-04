import os

import launch

from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    car_desc_dir = FindPackageShare(
        package='car_body').find('car_body')
    car_gazebo_dir = FindPackageShare(
        package='car_gazebo').find('car_gazebo')
    launch_dir = os.path.join(car_gazebo_dir, 'launch')
    default_controller_yaml_file = os.path.join(
        car_gazebo_dir, 'config/ackermann_controller.yaml')
    default_rviz_config_file = os.path.join(
        car_gazebo_dir, 'rviz/default_view.rviz')

    # Create launch configuration variables
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')

    robot_description = Command(['xacro ', os.path.join(car_desc_dir, 'description/vehicle.urdf.xacro'),
                                ' car_controller_yaml_file:=', params_file])
    default_world_file = Command(
        ['xacro ', os.path.join(car_gazebo_dir, 'worlds/empty.world')])

    # Define launch arguments
    headless = LaunchConfiguration('headless')
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to start gzclient')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Start with RViz if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file', default_value=default_controller_yaml_file,
        description='Absolute path to controller yaml config file')

    declare_world_file_cmd = DeclareLaunchArgument(
        name='world_file', default_value=default_world_file,
        description='Absolute path to world file to launch with Gazebo')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file', default_value=default_rviz_config_file,
        description='Absolute path to rviz config file')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    gazebo_params_file = os.path.join(
        car_gazebo_dir, 'config', 'gazebo_params.yaml')

    # # Define actions
    start_gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        cwd=[launch_dir], output='screen')

    start_gazebo_client = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')

    start_gazebo_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_car',
                   '-topic', 'robot_description'],
        output='screen',
        parameters=[
            {'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}],
    )

    start_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}])

    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}])
    start_rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

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
            target_action=start_gazebo_spawner,
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

    # # Create launch description
    ld = launch.LaunchDescription()

    # Declare launch options
    ld.add_action(gamepad)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_log_level_cmd)

    # Register event handlers
    ld.add_action(load_ackermann_drive_base_ctrl_event)
    ld.add_action(load_joint_state_ctrl_event)

    # Add actions
    ld.add_action(start_rviz)
    ld.add_action(start_gazebo_server)
    ld.add_action(start_gazebo_client)
    ld.add_action(fake_encoder_odom_topic)
    ld.add_action(start_gazebo_spawner)
    ld.add_action(map_base_link)
    ld.add_action(odom_base_link)
    ld.add_action(start_joint_state_publisher)
    ld.add_action(start_robot_state_publisher)
    # ld.add_action(imu)

    return ld
