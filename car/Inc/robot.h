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

struct Odometry
{
  float x;
  float y;
  float theta;
};

struct ROSBridgeTransmit
{
  uint16_t acc[3];
  uint16_t gyro[3];
  Odometry odom;
};

struct RosData
{
  float linear_x;
  float angular_z;
};

class Robot
{
public:
  Robot(UART_HandleTypeDef *);
  Rear rear;
  Servo steering;
  UART rosbridge;
  RosData received_data;
  void init();
  void run();
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
