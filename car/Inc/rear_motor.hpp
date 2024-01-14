#ifndef REAR_MOTOR_HPP
#define REAR_MOTOR_HPP

#include "stm32f4xx.h"

#ifdef __cplusplus
#include "robotlib/sensors/encoder.hpp"
#include "robotlib/actuators/motor.hpp"

class Rear
{
    float speed1 = 0, speed2 = 0;

public:
    Motor motor[2];
    Encoder motor_encoder[2];

    void init();
    void run();
    void set_speed(float, float);
};

#endif // __cplusplus
#endif // ROLLER_H