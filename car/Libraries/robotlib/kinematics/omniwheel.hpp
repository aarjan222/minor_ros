#ifndef OMNIWHEEL_H_
#define OMNIWHEEL_H_

#include "math.h"

// #define RABBIT
#define ELEPHANT

#ifdef ELEPHANT
const float motor_offsets[4] = {M_PI / 4.0, 3.0 * M_PI / 4.0, 5.0 * M_PI / 4.0,
                                7.0 * M_PI / 4.0};
#endif
#ifdef RABBIT
const float motor_offsets[4] = { M_PI * 0.28761111, M_PI * 0.7123889, M_PI * 1.282055, M_PI * 1.712388};
#endif

struct Twist
{
  float vx, vy, w;
  Twist(float _vx, float _vy, float _w) : vx(_vx), vy(_vy), w(_w) {}
  Twist() : Twist(0, 0, 0) {}

  static Twist from_v_theta_omega(const float v, const float theta,
                                  const float omega)
  {
    return Twist(v * cos(theta), v * sin(theta), omega);
  }
  float get_theta() const
  {
    return atan2(vy, vx);
  }
};

class OmniwheelKinematics
{

public:
  float inverseCouplingMatrix[4][3];
  float base_diameter, wheel_diameter;
  OmniwheelKinematics(const float _base_diameter, const float _wheel_diameter)
      : base_diameter(_base_diameter), wheel_diameter(_wheel_diameter)
  {
    for (int i = 0; i < 4; i++)
    {
      inverseCouplingMatrix[i][0] = sin(motor_offsets[i]);
      inverseCouplingMatrix[i][1] = -cos(motor_offsets[i]);
      inverseCouplingMatrix[i][2] = -base_diameter / 2.0;
    }
  }

  void get_motor_omega(const Twist twist, float *motor_omega) const
  {
    for (int i = 0; i < 4; i++)
    {
      motor_omega[i] = inverseCouplingMatrix[i][0] * twist.vx;
      motor_omega[i] += inverseCouplingMatrix[i][1] * twist.vy;
      motor_omega[i] += inverseCouplingMatrix[i][2] * twist.w;
      motor_omega[i] *= 2.0 / wheel_diameter;
    }
  }
};

#endif // OMNIWHEEL_H_
