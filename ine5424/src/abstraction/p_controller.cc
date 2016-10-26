

#include <system.h>
#include <p_controller.h>

__BEGIN_SYS

P::P(float kp, float max, float min){
	_max = max;
	_min = min;
	_kp = kp;
	_prev_error = 0;
}

P::~P() { }

float P::calculate (float setpoint, float pv) {
	db<P>(TRC) << "P::calculate" << endl;

	// calculate error
	float error = setpoint - pv;

	// proportional result
	float pOut = _kp * error;

	// Restrict to max/min
	if (pOut > _max)
	    pOut = _max;
	else if (pOut < _min)
	    pOut = _min;

	_prev_error = error;

  return pOut;
}

__END_SYS
