#ifndef __D_CONTROLLER_H_
#define __D_CONTROLLER_H_

#include <system.h>

__BEGIN_SYS

class D
{
public:

    D(float kd, float dt, float max, float min);

    ~D();

    /*
    * @param setpoint target
    * @param pv = process value read from a sensor
    * @return derivative result
    */
    float calculate(float setpoint, float pv);

private:
    // max/min output
    float _max;
    float _min;

    // ki -  Derivative gain
    float _kd;

    // dt -  loop interval time
    float _dt;

    // previous error
    float _prev_error;
};

__END_SYS

#endif
