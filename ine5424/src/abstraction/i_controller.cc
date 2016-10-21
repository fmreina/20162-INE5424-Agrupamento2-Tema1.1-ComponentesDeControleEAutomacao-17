

#include <system.h>
#include <i_controller.h>

__BEGIN_SYS

I::I(double ki, double dt, double max, double min){
	_max = max;
	_min = min;
	_ki = ki;
	_dt = dt;
	_prev_error = 0;
	_integral = 0;
}

I::~I() { }

double I::calculate (double setpoint, double pv) {
	db<I>(TRC) << "I::calculate" << endl;

	// calculate error
	double error = setpoint - pv;

	// Integral result
    _integral += error * _dt;
    double iOut = _ki * _integral;

	// Restrict to max/min
	if( iOut > _max ){
	    iOut = _max;
	}
	else if( iOut < _min ){
	    iOut = _min;
	}

	_prev_error = error;

  return iOut;
}

__END_SYS
