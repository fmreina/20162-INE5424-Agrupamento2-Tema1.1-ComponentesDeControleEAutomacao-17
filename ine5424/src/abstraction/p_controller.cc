#include <p_controller.h>
#include <machine.h>
#include <system.h>

__BEGIN_SYS

P::P(float sensor, float actuating, float setpoint, float min, float max, float kp)
{
  db<Controller>(WRN) << "P::P(" << kp << ")" <<endl;
  _sensor = sensor;
  _actuating = actuating;
  _setpoint = setpoint
  _max = max;
	_min = min;
	_kp = kp;
	_prev_error = 0;
}

P::~P()
{
   db<Controller>(WRN) << "~P()" << endl;
}

float P::calculate( double setpoint )
{
  	db<P>(WRN) << "P::calculate(" << setpoint << ")" <<endl;

    // calculate error
    float error = _setpoint - _sensor;
    // proportional result
  	float output = _kp * error;

    output = P::checkLimits(output, _min, _max);

    // FIXME: set _prev_error
    _prev_error = error;
    // P::setPrevError(error);
    // actuating->act();
    return output;
}

__END_SYS
