# car_gazebo 
Car like robot is simulated in real life using ros2_control. This package contains urdf file for the car like robot.

## Installation
```shell
pip3 uninstall serial
pip3 install pyserial==3.5

sudo apt install ros-humble-xacro
sudo apt install ros-humble-joint-state-publisher-gui 
rosdep install --from-paths src --ignore-src -r -y

colcon build
source install/setup.bash
```

# Run this in dev machine
## Launch File for viewing car only
```shell
ros2 launch car_gazebo view_car.launch.py
```
set base_footprint as fixed_frame in rviz2

## Launch File for running the hardware real carlikerobot which runs rplidar node, imu node, twist mux node, teleop node(for car teleop) and map->odom transform, odom->base_link transform nodes.
```shell
ros2 launch car_gazebo bringup_car.launch.py remap_odometry_tf:=true
```
set odom as fixed_frame in rviz2 to view car
set map as fixed frame when running slam and localizer

## Running slam toolbox launch file
```shell
ros2 launch car_gazebo online_async_launch.py 
```

# In local machine
## Run rviz2 in local device to see the robot and map
```shell
rviz2
```

## For running the joystick
```shell
ros2 launch car_gazebo joystick.launch.py 
```

# On Dev machine

## Runnning twist mux
```shell
ros2 run twist_mux twist_mux --ros-args --params-file src/car_gazebo/config/twist_mux.yaml -r cmd_vel_out:=/cmd_vel_stamped
```

##  For giving 2D goal pose.
```shell
ros2 launch nav2_bringup navigation_launch.py params_file:=src/car_gazebo/config/nav2_test_params.yaml 
```

# On local machine

## runnin dwb local planner

```shell
ros2 launch nav2_bringup navigation_launch.py params_file:=src/navigation2/config/nav2_params.yaml 
```

## running mppi controller
```shell
 ros2 launch nav2_bringup navigation_launch.py params_file:=src/navigation2/config/nav2_params_test.yaml
```