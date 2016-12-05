#ifndef __CONTROLLER2_H_
#define __CONTROLLER2_H_

// #include <periodic_thread.h>
#include <sensor.h>
#include <actuating.h>

__BEGIN_SYS

class Controller2
{

public:
	/*
	 * Runs from a periodic thread to calculate the specified control every 'dt' ms
	 */
	template<typename ... Args>
	void Run(float (* _control)(float, float, float, float, Args... args), Args... args) {
		db<Controller2>(TRC) << "Controller2::Run()" << endl;
		float _pv = _sensor->read();

		_error = _setpoint - _pv;
		_integral += _error * _dt;

		db<Controller2>(TRC) << "_integral="<< _integral << endl;

		_result = _control(_error, _prev_error, _dt, _integral, args...);

		if (_result > _max)
			_result = _max;
		else if (_result < _min)
			_result = _min;

		_prev_error = _error;

		_actuating->act(_result);
	}

protected:
	Sensor* _sensor;
	Actuating* _actuating;

	float _setpoint;
	float _error;
	float _integral;
	// max/min output
	float _max;
	float _min;
	// previous error
	float _prev_error;
	// dt -  loop interval time
	float _dt;
	float _result;

public:

	float get_result(){
		return _result;
	}

  void setSetpoint(float value){
    db<Controller2>(WRN) << "setSetpoint(value=" << value << ")" << endl;
    _setpoint = value;
  }

};

__END_SYS

#endif /* _CONTROLLER2_H_ */
