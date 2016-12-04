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


	// // @params _kp, _kd
	// float static PD(float error, float prev_error, float dt, float integral, float kp, float kd) {
	// 	db<Controller2>(TRC) << "Controller2::PD(error=" << error
	// 			<< ",prev_error=" << prev_error
	// 			<< ",dt=" << dt
	// 			<< ",integral=" << integral
	// 			<< ",kp=" << kp
	// 			<< ",kd=" << kd << ")" <<endl;
  //
	// 	float pOut = CalculateP(error, kp);
	// 	float dOut = CalculateD(error, dt, prev_error, kd);
  //
	// 	return pOut + dOut;
	// }
  //
	
  //
	// // @params _kp, _ki, _kd
	// float static PID(float error, float prev_error, float dt, float integral, float kp, float ki, float kd) {
	// 	db<Controller2>(TRC) << "Controller2::PID(error=" << error
	// 			<< ",prev_error=" << prev_error
	// 			<< ",dt=" << dt
	// 			<< ",integral=" << integral
	// 			<< ",kp=" << kp
	// 			<< ",ki=" << ki
	// 			<< ",kd=" << kd << ")" <<endl;
  //
	// 	float pOut = CalculateP(error, kp);
	// 	float iOut = CalculateI(error, dt, ki, integral);
	// 	float dOut = CalculateD(error, dt, prev_error, kd);
  //
	// 	return pOut + iOut + dOut;
	// }
  //
	float get_result(){
		return _result;
	}

  void setSetpoint(float value){
    db<Controller2>(WRN) << "setSetpoint(value=" << value << ")" << endl;
    _setpoint = value;
  }

// protected:
// //	@params: float error, float kp
// 	float static CalculateP(float error, float kp) {
// 		db<Controller2>(TRC) << "CalculateP(error=" << error << ",kp=" << kp << ")" << endl;
//
// 		// proportional result
// 		float pOut = kp * error;
//
// 		return pOut;
// 	}
// //@params: float error, float dt, float ki, float integral
// 	float static CalculateI(float error, float dt, float ki, float integral) {
// 		db<Controller2>(TRC) << "CalculateI(error=" << error << ",ki=" << ki << ",integral=" << integral << endl;
//
// 		float iOut = ki * integral;
//
// 		return iOut;
// 	}
// //@params: float error, float dt, float prev_error, float kd
// 	float static CalculateD(float error, float dt, float prev_error, float kd) {
// 		db<Controller2>(TRC) << "CalculateD(error=" << error << ",prev_error=" << prev_error << ",kd=" << kd << ")"<< endl;
//
// 		// Derivative term
// 		float derivative = (error - prev_error) / dt;
// 		float dOut = kd * derivative;
//
// 		return dOut;
// 	}
};

__END_SYS

#endif /* _CONTROLLER2_H_ */
