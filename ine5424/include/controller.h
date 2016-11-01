#ifndef __CONTROLLER_H_
#define __CONTROLLER_H_

// #include <periodic_thread.h>
#include <sensor.h>
#include <actuating.h>

__BEGIN_SYS

class Controller
{

  public:
	/*
	 * int sensor and int acutating is being used just for test purpose,
	 * the real implementation  must receive a real sensor and a real actuating
	 */
	template<typename ... Args>
    Controller(Sensor* sensor, Actuating* actuating, float max, float min, float setpoint, float dt, float (* control)(float, float, float, float, Args... args), Args... args):
    	_sensor(sensor),
    	_actuating(actuating),
    	_max(max),
    	_min(min),
    	_setpoint(setpoint),
    	_dt(dt)
	{
		// Periodic_Thread p_thread(RTConf(_dt * 1000, 1000), _control, args...);
		Run(control, args...);
	}

  ~Controller();

  /*
   * Runs from a periodic thread to calculate the specified control every 'dt' ms
   */
  template<typename ... Args>
  void Run(float (* _control)(float, float, float, float, Args... args), Args... args) {
 	  float _pv = _sensor->read();

  	_error = _setpoint - _pv;
  	_integral += _error * _dt;

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
    // @params _kp
    float static P(float error, float prev_error, float dt, float integral, float kp) {
    	db<Controller>(TRC) << "Controller::P(error=" << error
    									  << ",prev_error=" << prev_error
    									  << ",dt=" << dt
    									  << ",integral=" << integral
    									  << ",kp=" << kp << ")" <<endl;

    	return CalculateP(error, kp);
    }

    // @params _ki, _integral
    float static I(float error, float prev_error, float dt, float integral, float ki) {
    	db<Controller>(TRC) << "Controller::I(error=" << error
    	    									  << ",prev_error=" << prev_error
    	    									  << ",dt=" << dt
    	    									  << ",integral=" << integral
    	    									  << ",ki=" << ki << ")" <<endl;

    	return CalculateI(error, dt, ki, integral);
    }

    // @params _kd
    float static D(float error, float prev_error, float dt, float integral, float kd) {
    	db<Controller>(TRC) << "Controller::D(error=" << error
    	    									  << ",prev_error=" << prev_error
    	    									  << ",dt=" << dt
    	    									  << ",integral=" << integral
    	    									  << ",kd=" << kd << ")" <<endl;

    	return CalculateD(error, dt, prev_error, kd);
    }

    // @params _kp, _kd
    float static PD(float error, float prev_error, float dt, float integral, float kp, float kd) {
		db<Controller>(TRC) << "Controller::PD(error=" << error
    	    									  << ",prev_error=" << prev_error
    	    									  << ",dt=" << dt
    	    									  << ",integral=" << integral
    	    									  << ",kp=" << kp
    	    									  << ",kd=" << kd << ")" <<endl;

  		float pOut = CalculateP(error, kp);
  		float dOut = CalculateD(error, dt, prev_error, kd);

      return pOut + dOut;
	  }

    // @params _kp, _ki, integral
    float static PI(float error, float prev_error, float dt, float integral, float kp, float ki) {
    	db<Controller>(TRC) << "Controller::PI(error=" << error
    	    									  << ",prev_error=" << prev_error
    	    									  << ",dt=" << dt
    	    									  << ",integral=" << integral
    	    									  << ",kp=" << kp
    	    									  << ",ki=" << ki << ")" <<endl;

    	float pOut = CalculateP(error, kp);
    	float iOut = CalculateI(error, dt, ki, integral);

      return pOut + iOut;
    }

    // @params _kp, _ki, _kd, integral
    float static PID(float error, float prev_error, float dt, float integral, float kp, float ki, float kd) {
    	db<Controller>(TRC) << "Controller::PID(error=" << error
    	    									  << ",prev_error=" << prev_error
    	    									  << ",dt=" << dt
    	    									  << ",integral=" << integral
    	    									  << ",kp=" << kp
    	    									  << ",ki=" << ki
    	    									  << ",kd=" << kd << ")" <<endl;

    	float pOut = CalculateP(error, kp);
    	float iOut = CalculateI(error, dt, ki, integral);
    	float dOut = CalculateD(error, dt, prev_error, kd);

      return pOut + iOut + dOut;
    }

    float get_result(){
      return _result;
    }
  protected:
    float static CalculateP(float error, float kp) {
    	db<Controller>(TRC) << "CalculateP(error=" << error << ",kp=" << kp << ")" << endl;

    	// proportional result
    	float pOut = kp * error;

      return pOut;
    }

    float static CalculateI(float error, float dt, float ki, float integral) {
        db<Controller>(TRC) << "CalculateI(error=" << error << ",ki=" << ki << ",integral=" << integral << endl;

        float iOut = ki * integral;

        return iOut;
    }

    float static CalculateD(float error, float dt, float prev_error, float kd) {
    	db<Controller>(TRC) << "CalculateD(error=" << error << ",prev_error=" << prev_error << ",kd=" << kd << ")"<< endl;

      // Derivative term
      float derivative = (error - prev_error) / dt;
      float dOut = kd * derivative;

      return dOut;
    }
};

__END_SYS

#endif /* _CONTROLLER_H_ */
