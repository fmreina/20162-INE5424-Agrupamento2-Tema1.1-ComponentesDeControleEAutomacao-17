

#include <system.h>
#include <d_controller.h>


__BEGIN_SYS

D::D(float kd, float dt, float max, float min){
	_max = max;
	_min = min;
	_kd = kd;
	_dt = dt;
	_prev_error = 0;
}

D::~D() { }

float D::calculate (float setpoint, float pv) {
	db<D>(TRC) << "D::calculate" << endl;

	// calculate error
	float error = setpoint - pv;

    // Derivative term
    float derivative = (error - _prev_error) / _dt;
    float dOut = _kd * derivative;

	// Restrict to max/min
	if (dOut > _max)
	    dOut = _max;
	else if (dOut < _min)
	    dOut = _min;

	_prev_error = error;

  return dOut;
}

__END_SYS
