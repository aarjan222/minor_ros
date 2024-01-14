#include "rear_motor.hpp"
#include "tim.h"
#include "math.h"
#include <stdio.h>

void Rear::init()
{
    motor_encoder[0] = Encoder(&htim4, 800);
    motor_encoder[1] = Encoder(&htim3, 800);

    motor[0] = Motor(&htim9, rear_left_motor_dir_GPIO_Port,
                     TIM_CHANNEL_1, rear_left_motor_dir_Pin, 65535);

    motor[1] = Motor(&htim9, rear_right_motor_dir_GPIO_Port,
                     TIM_CHANNEL_2, rear_right_motor_dir_Pin, 65535);

    for (int i = 0; i < 2; i++)
    {
        motor[i].init();
        motor_encoder[i].init();
    }
}

void Rear::set_speed(float _speed1, float _speed2)
{
    speed1 = _speed1;
    speed2 = _speed2;
}

void Rear::run()
{
    motor[0].set_speed(speed1);
    motor[1].set_speed(speed2);
    printf("motor speed %.1f %.1f\n", speed1, speed2);
    // printf("motor count %d\t %d \n", motor_encoder[0].get_count(), motor_encoder[1].get_count());
    // printf("\t motor speed  %.1f\t %.1f\n", motor_encoder[0].get_omega(), motor_encoder[1].get_omega());
}