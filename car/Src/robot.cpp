#include "robot.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <algorithm>
#include <stdint.h>
#include "math.h"

#define pi 3.14
#define max_velocity 1.6
#define offset 30

float rad_to_deg(float x)
{

  x = std::min<float>(x, 0.5235);
  x = std::max<float>(x, -0.5235);
  return (x * 180 / pi) + 90 - offset;
}

float deg_to_rad(float x)
{
  return ((x - 90 + offset) * pi / 180);
}

float map(float x, float inp_min, float inp_max, float op_min, float op_max)
{
  return x * (op_max - op_min) / (inp_max - inp_min);
}

float count_to_radian(float x)
{
  return x * 2 * pi / 6200;
}

Robot car(&huart2);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
  __HAL_UART_FLUSH_DRREGISTER(huart);
  // printf("print here\n");
  if (huart->Instance == huart2.Instance)
  {
    // printf("then huart\n");
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    car.rosbridge.get_received_data((uint8_t *)(&car.received_data));
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == huart2.Instance)
  {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    // printf("ros transmitting\n");
  }
}

Robot::Robot(UART_HandleTypeDef *roshuart) : rosbridge(roshuart, 8, sizeof(ROSBridgeTransmit), BOTH),
                                             ackermannkinematics(wheel_base, rear_wheel_to_center) {}

void init_robot()
{
  printf("Init car\n");
  car.init();
}

void operate_robot()
{
  printf("RUN car\n");
  uint32_t init_time = HAL_GetTick();
  while (true)
  {
    if (HAL_GetTick() - init_time > 30)
    {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
      car.run();
      car.rosbridge.transmit((uint8_t *)&(car.ros_transmit));
      init_time = HAL_GetTick();
    }
  }
}

void Robot::init()
{
  steering = Servo(&htim10, TIM_CHANNEL_1);
  rosbridge.init();

  steering.init();
  rear.init();

  car_state.car_steer_angle = 90.0;
  steering.set_angle(car_state.car_steer_angle);
  steering.run();

  car_state.car_speed = 0.0;
  // rear.set_speed(car_state.car_speed, car_state.car_speed);
}

void Robot::run()
{
  set_state_from_uart_data();
}

void Robot::set_state_from_uart_data()
{
  // if (HAL_GetTick() - rosbridge.last_updated_tick > 400)
  // {
  //   printf("delayed or no data from pi\t");
  //   car_state = CarState(0.0, 90.0);
  //   printf("%f\t%f\n", car_state.car_speed, car_state.car_steer_angle);
  //   return;
  // }

  car_state.car_speed = car.received_data.linear_x;
  car_state.car_steer_angle = rad_to_deg(received_data.angular_z);
  // car_state.car_steer_angle = rad_to_deg(0);

  // car_state = CarState(car_state.car_speed, car_state.car_steer_angle);
  update();
}

void Robot::update()
{
  steering.set_angle(car_state.car_steer_angle);
  steering.run();

  rear.set_speed(car_state.car_speed, car_state.car_speed); // TODO
  rear.run();

  float omega1 = -rear.motor_encoder[0].get_omega(); // rad/s
  float omega2 = -rear.motor_encoder[1].get_omega();

  // rear.velocity1 = omega1 * 0.115; // m/s
  // rear.velocity2 = omega2 * 0.115;

  // printf("%f\t%f\t\n", car_state.car_speed, car_state.car_steer_angle);
  // printf("%d\t%d\t", rear.motor_encoder[0].get_count(), rear.motor_encoder[1].get_count());
  // printf("omegas %f\t%f\t", omega1, omega2);
  // printf("velocities %f\t%f\n", rear.velocity1, rear.velocity2);
  ackermannkinematics.get_pose(car_pose, rear.velocity2, 0); // TODO
  printf("car pose= ");
  for (int i = 0; i < 3; i++)
  {
    printf("%f\t", car_pose[i]); // x,y in m and theta in radian
  }
  printf("\t\t\t");

  // printf("received data lin_x=%.4f\tang_z=%.4f\t", received_data.linear_x, received_data.angular_z);

  ros_transmit.odom.rear_encoder_val = count_to_radian(car.rear.motor_encoder[1].get_count_aggregate());
  ros_transmit.odom.rear_wheel_velocity = omega2;
  ros_transmit.odom.front_servo_steering = deg_to_rad(car_state.car_steer_angle);
  printf("transmit data rear_vel=%.4f\tfront_steer=%.4f\n", ros_transmit.odom.rear_wheel_velocity, ros_transmit.odom.front_servo_steering);
  // printf("\n");
}