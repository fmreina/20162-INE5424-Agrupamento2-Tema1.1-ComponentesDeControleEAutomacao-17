#ifndef __CONTROLLER_H_
#define __CONTROLLER_H_

#include <sensor_interface.h>
#include <actuating_interface.h>

__BEGIN_SYS

class Controller
{

  public:
	/*
	 * int sensor and int acutating is being used just for test purpose,
	 * the real implementation  must receive a real sensor and a real actuating
	 */
	template<typename ... Args>
    Controller(Sensor_Interface* sensor, Actuating_Interface* actuating, float dt, void (* control)(Args... args), Args... args):
    	_sensor(sensor),
    	_actuating(actuating),
    	_dt(dt) {
        Run(control, args...);
      }

    ~Controller();

    /*
     * Runs a periodic thread to calculate the specified control every 'dt' ms
     */
    template<typename ... Args>
    void Run(void (* _control)(Args... args), Args... args) {
    	_control(args...);
    }
  protected:
    Sensor_Interface* _sensor;
    Actuating_Interface*  _actuating;

    float _kp,
		  _ki,
		  _kd;

    // max/min output
    float _max;
    float _min;
    // previous error
    float _prev_error;
    // dt -  loop interval time
    float _dt;

  public:

    // @params _kp
    void static P(float _kp) {
    	db<Controller>(WRN) << "Controller::P(" << _kp << ")" <<endl;
      // Controller::_sensor->read();
    }
    // @params _ki, _integral
    void static I(float _ki, float _integral) {
    	db<Controller>(WRN) << "Controller::I(" << _ki << "," << _integral << ")" <<endl;
      // _sensor->read();
    }
    // @params _kd
    void static D(float _kd) {
    	db<Controller>(WRN) << "Controller::I("  << _kd << ")" <<endl;
      // _sensor->read();
    }
    // @params _kp, _kd
  	void static PD(float _kp, float _kd) {
  		db<Controller>(WRN) << "Controller::PD(" << _kp << "," << _kd << ")" <<endl;
      // _sensor->read();
  	}
    // @params _kp, _ki, integral
    void static PI(float _kp, float _ki, float _integral) {
    	db<Controller>(WRN) << "Controller::PI(" << _kp << ","  << _ki << "," << _integral << ")" <<endl;
      // _sensor->read();
    }
    // @params _kp, _ki, _kd, integral
    void static PID(float _kp, float _ki, float _kd, float _integral) {
    	db<Controller>(WRN) << "Controller::PID(" << _kp << "," << _ki << "," << _kd << "," << _integral << ")" <<endl;
      // _sensor->read();
    }
//  protected:
//    float static CalculateP(float kp, float setpoint, float pv) {
//    	db<Controller>(WRN) << "CalculateP()::params (setpoint = " << setpoint << ", pv = " << pv << endl;
//
//    	// calculate error
//    	float error = setpoint - pv;
//
//    	// proportional result
//    	float pOut = _kp * error;
//
//    	// Restrict to max/min
//    	if (pOut > _max)
//    	    pOut = _max;
//    	else if (pOut < _min)
//    	    pOut = _min;
//
//        return pOut;
//    }
//    float static CalculateI(float ki, float integral, float setpoint, float pv) {
//        db<Controller>(WRN) << "CalculateI()::params (setpoint = " << setpoint << ", pv = " << pv << endl;
//
//        // calculate error
//        float error = setpoint - pv;
//
//        // Integral result
//        _integral += error * _dt;
//        float iOut = _ki * _integral;
//
//        // Restrict to max/min
//        if (iOut > _max)
//            iOut = _max;
//        else if (iOut < _min)
//            iOut = _min;
//
//        return iOut;
//    }
//    float static CalculateD(float kd, float setpoint, float pv) {
//    	db<Controller>(WRN) << "CalculateD()::params (setpoint = " << setpoint << ", pv = " << pv << endl;
//
//    	// calculate error
//    	float error = setpoint - pv;
//
//        // Derivative term
//        float derivative = (error - _prev_error) / _dt;
//        float dOut = _kd * derivative;
//
//    	// Restrict to max/min
//    	if (dOut > _max)
//    	    dOut = _max;
//    	else if (dOut < _min)
//    	    dOut = _min;
//
//    	_prev_error = error;
//
//      return dOut;
//    }
};

__END_SYS

#endif /* _CONTROLLER_H_ */
