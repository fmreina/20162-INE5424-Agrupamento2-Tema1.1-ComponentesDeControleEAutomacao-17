#ifndef __P_CONTROLLER_H_
#define __P_CONTROLLER_H_

#include <system.h>

__BEGIN_SYS

class P {
  public:

    P(float kp, float max, float min);

    ~P();

    /*
     * @param setpoint target
     * @param pv = process value read from a sensor
     * @return proportional result
     */
    float calculate(float setpoint, float pv);

  private:
    float _max;
    float _min;
    // kp = proportional gain
    float _kp;
    // previous error
    float _prev_error;
};

__END_SYS

#endif
