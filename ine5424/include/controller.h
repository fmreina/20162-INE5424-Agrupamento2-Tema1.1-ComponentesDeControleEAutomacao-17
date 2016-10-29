#ifndef __CONTROLLER_H_
#define __CONTROLLER_H_

#include <sensor_interface.h>
#include <actuating_interface.h>
#include <controller_interface.h>

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
      float sensor_value = _sensor->read();

      _control(args...);

      _actuating->act(15);
    }
  protected:
    Sensor_Interface* _sensor;
    Actuating_Interface*  _actuating;
    Controller_Interface* _controller;

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
      // _controller = new Controller_Interface()
    }
    // @params _ki, _integral
    void static I(float _ki, float _integral) {
    	db<Controller>(WRN) << "Controller::I(" << _ki << "," << _integral << ")" <<endl;
    }
    // @params _kd
    void static D(float _kd) {
    	db<Controller>(WRN) << "Controller::I("  << _kd << ")" <<endl;
    }
    // @params _kp, _kd
  	void static PD(float _kp, float _kd) {
  		db<Controller>(WRN) << "Controller::PD(" << _kp << "," << _kd << ")" <<endl;
  	}
    // @params _kp, _ki, integral
    void static PI(float _kp, float _ki, float _integral) {
    	db<Controller>(WRN) << "Controller::PI(" << _kp << ","  << _ki << "," << _integral << ")" <<endl;
    }
    // @params _kp, _ki, _kd, integral
    void static PID(float _kp, float _ki, float _kd, float _integral) {
    	db<Controller>(WRN) << "Controller::PID(" << _kp << "," << _ki << "," << _kd << "," << _integral << ")" <<endl;
    }
};

__END_SYS

#endif /* _CONTROLLER_H_ */
