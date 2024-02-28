#include "robot.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>

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
    if (HAL_GetTick() - init_time > 100)
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
  rear.set_speed(car_state.car_speed, car_state.car_speed);
}

void Robot::run()
{
  set_state_from_uart_data();
}

void Robot::set_state_from_uart_data()
{
  // car_state.car_speed = 0.5;
  // car_state.car_steer_angle = 110.0;
  // if (HAL_GetTick() - rosbridge.last_updated_tick > 400)
  // {
  //   printf("delayed or no data from pi\t");
  //   car_state = CarState(0.0, 90.0);
  //   printf("%f\t%f\n", car_state.car_speed, car_state.car_steer_angle);
  //   return;
  // }

  car_state.car_speed = received_data.linear_x;
  car_state.car_steer_angle = received_data.angular_z;

  if (received_data.linear_x > 0)
  {
    if (received_data.angular_z > 0)
    {
      printf("left forward");
      car_state.car_steer_angle = 70.0;
    }
    else if (received_data.angular_z == 0)
    {
      printf("forward");
      car_state.car_steer_angle = 90.0;
    }
    else if (received_data.angular_z < 0)
    {
      printf("right forward");
      car_state.car_steer_angle = 110.0;
    }
  }
  else if (received_data.linear_x == 0)
  {
    if (received_data.angular_z > 0)
    {
      printf("left");
      car_state.car_steer_angle = 70.0;
    }
    else if (received_data.angular_z == 0)
    {
      printf("stop");
      car_state.car_steer_angle = car_state.car_steer_angle;
    }
    else if (received_data.angular_z < 0)
    {
      printf("right");
      car_state.car_steer_angle = 110.0;
    }
  }
  else if (received_data.linear_x < 0)
  {
    if (received_data.angular_z < 0)
    {
      printf("left reverse");
      car_state.car_steer_angle = 70.0;
    }
    else if (received_data.angular_z == 0)
    {
      printf("reverse");
      car_state.car_steer_angle = 90.0;
    }
    else if (received_data.angular_z > 0)
    {
      printf("right reverse");
      car_state.car_steer_angle = 110.0;
    }
  }

  car_state = CarState(car_state.car_speed, car_state.car_steer_angle);
  update();
}

void Robot::update()
{
  car.steering.set_angle(car_state.car_steer_angle);
  car.steering.run();

  rear.set_speed(car_state.car_speed * 0.2, car_state.car_speed * 0.2);
  rear.run();

  rear.set_speed(car_state.car_speed, car_state.car_speed);
  rear.run();

  float omega1 = -rear.motor_encoder[0].get_omega(); // rad/s
  float omega2 = -rear.motor_encoder[1].get_omega();

  rear.velocity1 = omega1 * 0.115; // m/s
  rear.velocity2 = omega2 * 0.115;

  printf("%f\t%f\n", car_state.car_speed, car_state.car_steer_angle);
  // printf("%d\t%d\t", rear.motor_encoder[0].get_count(), rear.motor_encoder[1].get_count());
  // printf("omegas %f\t%f\t", omega1, omega2);
  // printf("velocities %f\t%f\t", rear.velocity1, rear.velocity2);
  ackermannkinematics.get_pose(car_pose, rear.velocity1, car_state.car_steer_angle - 90.0);
  printf("car pose= ");
  for (int i = 0; i < 3; i++)
  {
    printf("%f\t", car_pose[i]); // x,y in m and theta in radian
  }
  car.ros_transmit.odom.x = car_pose[0];
  car.ros_transmit.odom.y = car_pose[1];
  car.ros_transmit.odom.theta = car_pose[2];
  car.ros_transmit.odom.rear_encoder_val = rear.motor_encoder[0].get_count();
  car.ros_transmit.odom.rear_wheel_velocity = omega1;
  car.ros_transmit.odom.front_servo_steering = car_state.car_steer_angle;
  printf("\n");
}