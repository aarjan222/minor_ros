#ifndef ACKERMANN_KINEMATICS_H_
#define ACKERMANN_KINEMATICS_H_

#define WB 0.46

#include "math.h"

struct Twist
{
    float vx, w, L;
    Twist(float _vx, float _w) : vx(_vx), w(_w) {}
    Twist() : Twist(0, 0) {}

    static Twist from_v_delta(const float v, const float delta)
    {
        return Twist(v, v * tan(delta) / WB);
    }
};

class AckermannKinematics
{
public:
    float wheel_base, rear_wheel_to_center, slip_angle;
    AckermannKinematics(const float _wheel_base, const float _rear_wheel_to_center) : wheel_base(_wheel_base), rear_wheel_to_center(_rear_wheel_to_center) {}

    void get_pose(float *pose, float velocity, float steer_angle)
    {
        // slip_angle = atan(rear_wheel_to_center * tan(steer_angle) / wheel_base);
        slip_angle = 0.0;
        float delta_time = 0.03;
        pose[0] += delta_time * velocity * cos(pose[2] + slip_angle);
        pose[1] += delta_time * velocity * sin(pose[2] + slip_angle);
        pose[2] += delta_time * velocity * cos(slip_angle) * tan(steer_angle) / wheel_base * 180 / 3.14;
    }
};

#endif