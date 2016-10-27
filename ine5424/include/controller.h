#ifndef __CONTROLLER_H_
#define __CONTROLLER_H_

__BEGIN_SYS

class Controller
{

  public:
	/*
	 * int sensor and int acutating is being used just for test purpose,
	 * the real implementation  must receive a real sensor and a real actuating
	 */
	template<typename ... Args>
    Controller(int sensor, int actuating, float dt, void (* control)(Args... args), Args... args):
    	_sensor(sensor),
    	_actuating(actuating),
    	_dt(dt) { Run(control, args...); }
    ~Controller();

    /*
     * Runs a periodic thread to calculate the specified control every 'dt' ms
     */
    template<typename ... Args>
    void Run(void (* _control)(Args... args), Args... args) {
    	_control(args...);
    }
  protected:
    int   _sensor,
		  _actuating;
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
    	db<Controller>(TRC) << "Controller::P(" << _kp << ")" <<endl;
    }
    // @params _ki, _integral
    void static I(float _ki, float _integral) {
    	db<Controller>(TRC) << "Controller::I(" << _ki << "," << _integral << ")" <<endl;
    }
    // @params _kd
    void static D(float _kd) {
    	db<Controller>(TRC) << "Controller::I("  << _kd << ")" <<endl;
    }
    // @params _kp, _kd
	void static PD(float _kp, float _kd) {
		db<Controller>(TRC) << "Controller::PD(" << _kp << "," << _kd << ")" <<endl;
	}
    // @params _kp, _ki, integral
    void static PI(float _kp, float _ki, float _integral) {
    	db<Controller>(TRC) << "Controller::PI(" << _kp << ","  << _ki << "," << _integral << ")" <<endl;
    }
    // @params _kp, _ki, _kd, integral
    void static PID(float _kp, float _ki, float _kd, float _integral) {
    	db<Controller>(TRC) << "Controller::PID(" << _kp << "," << _ki << "," << _kd << "," << _integral << ")" <<endl;
    }
//  protected:
//    float static CalculateP(float kp, float setpoint, float pv) {
//    	db<Controller>(TRC) << "CalculateP()::params (setpoint = " << setpoint << ", pv = " << pv << endl;
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
//        db<Controller>(TRC) << "CalculateI()::params (setpoint = " << setpoint << ", pv = " << pv << endl;
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
//    	db<Controller>(TRC) << "CalculateD()::params (setpoint = " << setpoint << ", pv = " << pv << endl;
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
