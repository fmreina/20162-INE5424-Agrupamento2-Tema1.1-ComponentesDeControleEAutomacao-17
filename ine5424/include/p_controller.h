#ifndef __P_CONTROLLER_H_
#define __P_CONTROLLER_H_

#include<controller2.h>
#include <cpu.h>
#include <machine.h>
#include <system.h>

__BEGIN_SYS

class P : Controller2
{

  public:

    P(float sensor, float actuating, float setpoint, float min, float max, float kp)  : Controller2()
    {
      db<P>(WRN) << "P::P()" << endl;
      _sensor = sensor;
      _actuating = actuating;
      _setpoint = setpoint;
      _max = max;
    	_min = min;
    	_kp = kp;
    	_prev_error = 0;
    }

    ~P() : Controller2()
    {
       db<P>(WRN) << "~P()" << endl;
    }

  protected:
    float _kp;

  public:

    virtual float calculate( double setpoint )
    {
      	db<P>(WRN) << "P::calculate()" << endl;

        // calculate error
        float error = _setpoint - _sensor;
        // proportional result
      	float output = _kp * error;

        output = checkLimits(output, _min, _max);

        _prev_error = error;
        // actuating->act();
        return output;
    }

  private:
    float checkLimits(float value, float min, float max){
      // Restrict to max/min
      if( value > max ){
          value = max;
      }
      else if( value < min ){
          value = min;
      }
      return value;
    }
};

__END_SYS

#endif /* __P_CONTROLLER_H_ */
