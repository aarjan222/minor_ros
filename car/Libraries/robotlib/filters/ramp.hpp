#include <math.h>
#include "robotlib/maths/math.hpp"

template <typename T>
class Ramp
{
public:
    T setpoint = 0;
    T slope;
    T output;

    Ramp(const T _slope, const T initial_output = 0) : slope(_slope), output(initial_output) {}

    T operator()(const T _setpoint)
    {
        setpoint = _setpoint;
        if (output > setpoint)
        {
            slope = -fabs(slope);
        }
        else
        {
            slope = fabs(slope);
        }

        if (slope > 0)
            output = clamp(output + slope, output, setpoint);
        else
            output = clamp(output + slope, setpoint, output);
        return output;
    }
};