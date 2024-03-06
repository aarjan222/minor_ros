#ifndef ROBOT_H_
#define ROBOT_H_

#include "stm32f4xx.h"

#ifdef __cplusplus
#include "robotlib/actuators/motor.hpp"
#include "robotlib/sensors/encoder.hpp"
#include "rear_motor.hpp"
#include "robotlib/communication/uart.h"
#include "tim.h"
#include "usart.h"
#include "robotlib/actuators/servo.hpp"
#include "ackermann_kinematics.hpp"

#define wheel_base 0.54
#define rear_wheel_to_center 0.27
uint32_t loop_time = 100.0;
float wheel_radius = 0.115;

struct Odometry
{
  float x;
  float y;
  float theta;
  float rear_encoder_val;
  float rear_wheel_velocity;
  float front_servo_steering;

};

struct ROSBridgeTransmit
{
  Odometry odom;
};

struct RosData
{
  float linear_x;
  float angular_z;
};

float car_pose[3] = {0., 0., 0.};

struct CarState
{
  float car_speed, car_steer_angle;
  CarState(float _car_speed, float _car_steer_angle) : car_speed(_car_speed), car_steer_angle(_car_steer_angle) {}
  CarState() {}
};

class Robot
{
public:
  Robot(UART_HandleTypeDef *);
  Rear rear;
  Servo steering;
  UART rosbridge;
  RosData received_data;
  AckermannKinematics ackermannkinematics;
  CarState car_state;
  ROSBridgeTransmit ros_transmit;
  void init();
  void run();
  void set_state_from_uart_data();
  void update();
};

#endif // __cplusplus

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

  void init_robot();
  void operate_robot();

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // ROBOT_H_
