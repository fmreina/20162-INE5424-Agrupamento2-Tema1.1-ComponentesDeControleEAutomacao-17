#ifndef __CONTROLLER_H_
#define __CONTROLLER_H_

// #include <sensor_interface.h>
// #include <actuating_interface.h>
// #include <controller_interface.h>

__BEGIN_SYS

class Controller
{

  public:
	/*
	 * int sensor and int acutating is being used just for test purpose,
	 * the real implementation  must receive a real sensor and a real actuating
	 */
	template<typename ... Args>
    Controller(float (* control)(Args... args), Args... args)
    {
      Run(control, args...);
    }

    ~Controller();

    /*
     * Runs a periodic thread to calculate the specified control every 'dt' ms
     */
    template<typename ... Args>
    void Run(float (* _control)(Args... args), Args... args) {
      // float sensor_value = _sensor->read();

      float _output = _control(args...);

      // _actuating->act(15);
    }
  protected:
    // float _sensor;
    // float  _actuating;

    float _kp, _ki, _kd;

    // max/min output
    float _max;
    float _min;
    // previous error
    static float _prev_error;
    // dt -  loop interval time
    float _dt;

  public:

    // @params sensor, actuating, setpoint, min, max, _kp
    float static P(float sensor, float actuating, float _setpoint, float _min, float _max, float _kp) {
    	db<Controller>(WRN) << "Controller::P(" << _kp << ")" <<endl;

      // calculate error
      float error = _setpoint - sensor;
      // proportional result
    	float output = _kp * error;
      // Restrict to max/min
    	if( output > _max ){
    	    output = _max;
      } else if( output < _min ){
    	    output = _min;
      }

      // _prev_error = error;
      // actuating->act();
      return output;
    }

    // @params sensor, actuating, setpoint, min, max, _ki, dt, _integral
    float static I(float sensor, float actuating, float _setpoint, float _min, float _max, float _ki, float _dt, float _integral) {
    	db<Controller>(WRN) << "Controller::I(" << _ki << "," << _integral << ")" <<endl;

      // calculate error
      float error = _setpoint - sensor;
    	// Integral result
      _integral += error * _dt;
      float output = _ki * _integral;

    	// Restrict to max/min
    	if( output > _max ){
    	    output = _max;
    	}
    	else if( output < _min ){
    	    output = _min;
    	}

    	//_prev_error = error;
      // actuating->act();
      return output;
    }

    // @params sensor, actuating, setpoint, min, max, _kd, dt
    float static D(float sensor, float actuating, float _setpoint, float _min, float _max, float _kd, float _dt) {
    	db<Controller>(WRN) << "Controller::I("  << _kd << ")" <<endl;

      // calculate error
      float error = _setpoint - sensor;
      // Derivative term
      // FIXME: use _prev_error
      double derivative;// = (error - _prev_error) / _dt;
      derivative = error / _dt;
      double output = _kd * derivative;

      // Restrict to max/min
      if( output > _max ){
          output = _max;
      }
      else if( output < _min ){
          output = _min;
      }

      // _prev_error = error;
      // actuating->act();
      return output;
    }
    // // @params _kp, _kd
  	// void static PD(float _kp, float _kd) {
  	// 	db<Controller>(WRN) << "Controller::PD(" << _kp << "," << _kd << ")" <<endl;
  	// }
    // // @params _kp, _ki, integral
    // void static PI(float _kp, float _ki, float _integral) {
    // 	db<Controller>(WRN) << "Controller::PI(" << _kp << ","  << _ki << "," << _integral << ")" <<endl;
    // }
    // // @params _kp, _ki, _kd, integral
    // void static PID(float _kp, float _ki, float _kd, float _integral) {
    // 	db<Controller>(WRN) << "Controller::PID(" << _kp << "," << _ki << "," << _kd << "," << _integral << ")" <<endl;
    // }
};

__END_SYS

#endif /* _CONTROLLER_H_ */
