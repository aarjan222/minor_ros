# car_gazebo 
Car like robot is simulated in real life using ros2_control. This package contains urdf file for the car like robot.

## Installation
```shell
pip3 install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y

colcon build
source install/setup.bash
```

## Launch File for viewing car only
```shell
ros2 launch car_gazebo view_car.launch.py
```
set base_footprint in rviz2

## Launch File for running the hardware real carlikerobot
```shell
ros2 launch car_gazebo bringup_car.launch.py remap_odometry_tf:=true
```
