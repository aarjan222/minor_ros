#include "servo.hpp"
#include "tim.h"
#include <stdio.h>

void Servo::init()
{
    HAL_TIM_PWM_Start(htim, timer_pwm_channel);
}

void Servo::set_angle(float _angle)
{
    if (_angle < 60)
    {
        angle = 60;
    }
    if (_angle > 120)
    {
        angle = 120;
    }
    else
    {
        angle = _angle;
    }
}

void Servo::run()
{
    float time = min_time + (max_time - min_time) * angle / 180;
    uint32_t count = 500 * time;
    __HAL_TIM_SetCompare(htim, timer_pwm_channel, count);
}