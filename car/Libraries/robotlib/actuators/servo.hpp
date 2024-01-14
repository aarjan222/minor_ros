#ifndef _SERVO_H
#define _SERVO_H

#include "stm32f4xx.h"

class Servo
{
public:
    Servo() {}
    TIM_HandleTypeDef *htim;
    int timer_pwm_channel;
    bool servo_bldc = false;
    float angle;
    float max_time = 0, min_time = 0;

    Servo(TIM_HandleTypeDef *_htim,
                 int _timer_pwm_channel, float _max_time = 2.5,
                 float _min_time = 0.5) : htim(_htim), timer_pwm_channel(_timer_pwm_channel), max_time(_max_time), min_time(_min_time){}
    void init();
    void set_angle(float);
    void run();
};

#endif