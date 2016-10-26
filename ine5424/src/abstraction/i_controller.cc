

#include <system.h>
#include <i_controller.h>

__BEGIN_SYS

I::I(float ki, float dt, float max, float min){
	_max = max;
	_min = min;
	_ki = ki;
	_dt = dt;
	_prev_error = 0;
	_integral = 0;
}

I::~I() { }

float I::calculate (float setpoint, float pv) {
	db<I>(TRC) << "I::calculate" << endl;

	// calculate error
	float error = setpoint - pv;

	// Integral result
    _integral += error * _dt;
    float iOut = _ki * _integral;

	// Restrict to max/min
	if (iOut > _max)
	    iOut = _max;
	else if (iOut < _min)
	    iOut = _min;

	_prev_error = error;

  return iOut;
}

__END_SYS
