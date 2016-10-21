

#include <system.h>
#include <d_controller.h>


__BEGIN_SYS

D::D(double kd, double dt, double max, double min){
	_max = max;
	_min = min;
	_kd = kd;
	_dt = dt;
	_prev_error = 0;
}

D::~D() { }

double D::calculate (double setpoint, double pv) {
	db<D>(TRC) << "D::calculate" << endl;

	// calculate error
	double error = setpoint - pv;

    // Derivative term
    double derivative = (error - _prev_error) / _dt;
    double dOut = _kd * derivative;

	// Restrict to max/min
	if( dOut > _max ){
	    dOut = _max;
	}
	else if( dOut < _min ){
	    dOut = _min;
	}

	_prev_error = error;

  return dOut;
}

__END_SYS
