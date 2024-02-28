# car_gazebo 
Car like robot is simulated in real life using ros2_control. This package contains urdf file for the car like robot.

## Installation
```shell
pip3 uninstall serial
pip3 install pyserial==3.5

rosdep install --from-paths src --ignore-src -r -y

colcon build
source install/setup.bash
```

## Launch File for viewing car only
```shell
ros2 launch car_gazebo view_car.launch.py
```
set base_footprint as fixed_frame in rviz2

## Launch File for running the hardware real carlikerobot
```shell
ros2 launch car_gazebo bringup_car.launch.py remap_odometry_tf:=true
```
set odom as fixed_frame in rviz2

## In another terminal run for sending feedback of stm to ros2_control using socket
```shell
ros2 run car_gazebo pose_receiver.py
```

## Check if the hardware interface loaded properly, by opening another terminal and executing

```shell
ros2 control list_hardware_interfaces
```

## You should get
```shell
command interfaces
   bicycle_steering_controller/angular/position [unavailable] [unclaimed]
   bicycle_steering_controller/linear/velocity [unavailable] [unclaimed]
   virtual_front_wheel_joint/position [available] [claimed]
   virtual_rear_wheel_joint/velocity [available] [claimed]
state interfaces
   virtual_front_wheel_joint/position
   virtual_rear_wheel_joint/position
   virtual_rear_wheel_joint/velocity
```

## Check if controllers are running
```shell
ros2 control list_controllers
```

You should get
```shell
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
bicycle_steering_controller[bicycle_steering_controller/BicycleSteeringController] active
```

## If everything is fine, now you can send a command to bicycle_steering_controller using ROS 2 CLI:

```shell
ros2 topic pub --rate 30 /bicycle_steering_controller/reference geometry_msgs/msg/TwistStamped "
  twist:
    linear:
      x: 1.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.1"
```

## You can also run your car using teleop_twist_keyboard also
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```