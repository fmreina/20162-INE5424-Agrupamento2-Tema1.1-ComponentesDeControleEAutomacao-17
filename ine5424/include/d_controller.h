#ifndef __D_CONTROLLER_H_
#define __D_CONTROLLER_H_

#include <system.h>

__BEGIN_SYS

class D
{
public:

    D(double kd, double dt, double max, double min);

    ~D();

    /*
    * @param setpoint target
    * @param pv = process value read from a sensor
    * @return derivative result
    */
    double calculate(double setpoint, double pv);

private:
    // max/min output
    double _max;
    double _min;

    // ki -  Derivative gain
    double _kd;

    // dt -  loop interval time
    double _dt;

    // previous error
    double _prev_error;
};

__END_SYS

#endif
