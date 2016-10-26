#ifndef __I_CONTROLLER_H_
#define __I_CONTROLLER_H_

#include <system.h>

__BEGIN_SYS

class I
{
public:

    I(float ki, float dt, float max, float min);

    ~I();

    /*
     * @param setpoint target
     * @param pv = process value read from a sensor
     * @return proportional result
     */
    float calculate(float setpoint, float pv);

private:
    float _max;
    float _min;
    // ki -  Integral gain
    float _ki;

     // dt -  loop interval time
    float _dt;

    // integral value
    float _integral;
    // previous error
    float _prev_error;
};

__END_SYS

#endif
