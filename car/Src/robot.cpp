#include "robot.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>

Robot car(&huart1);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
  __HAL_UART_FLUSH_DRREGISTER(huart);
  // printf("print here\n");
  if (huart->Instance == huart1.Instance)
  {
    // printf("then huart\n");
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    car.rosbridge.get_received_data((uint8_t *)(&car.received_data));
  }
}

Robot::Robot(UART_HandleTypeDef *roshuart) : rosbridge(roshuart, 8, RECEIVING) {}

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
      init_time = HAL_GetTick();
    }
  }
}

void Robot::init()
{
  steering = Servo(&htim10, TIM_CHANNEL_1);
  rosbridge.init();

  steering.init();
  steering.set_angle(90);
  steering.run();

  rear.init();
}

void Robot::run()
{

  printf("linear x and angular z %f %f\n", car.received_data.linear_x, car.received_data.angular_z);
  car.rear.set_speed(car.received_data.linear_x * 0.2, car.received_data.linear_x * 0.2);
  car.rear.run();

  car.steering.run();

  if (received_data.linear_x > 0)
  {
    if (received_data.angular_z > 0)
    {
      printf("left forward");
      car.steering.set_angle(70);
      car.steering.run();
    }
    else if (received_data.angular_z == 0)
    {
      printf("forward");
      car.steering.set_angle(90);
      car.steering.run();
    }
    else if (received_data.angular_z < 0)
    {
      printf("right forward");
      car.steering.set_angle(110);
      car.steering.run();
    }
  }
  else if (received_data.linear_x == 0)
  {
    if (received_data.angular_z > 0)
    {
      printf("left");
      car.steering.set_angle(60);
      car.steering.run();
    }
    else if (received_data.angular_z == 0)
    {
      printf("stop");
    }
    else if (received_data.angular_z < 0)
    {
      printf("right");
      car.steering.set_angle(110);
      car.steering.run();
    }
  }
  else if (received_data.linear_x < 0)
  {
    if (received_data.angular_z < 0)
    {
      printf("left reverse");
      car.steering.set_angle(70);
      car.steering.run();
    }
    else if (received_data.angular_z == 0)
    {
      printf("reverse");
      car.steering.set_angle(90);
      car.steering.run();
    }
    else if (received_data.angular_z > 0)
    {
      printf("right reverse");
      car.steering.set_angle(110);
      car.steering.run();
    }
  }
}