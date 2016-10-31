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

      output = Controller::checkLimits(output, _min, _max);

      // FIXME: set _prev_error
      //_prev_error = error;
      // Controller::setPrevError(error);
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

    	output = Controller::checkLimits(output, _min, _max);

      // FIXME: set _prev_error
    	//_prev_error = error;
      // actuating->act();
      return output;
    }

    // @params sensor, actuating, setpoint, min, max, _kd, dt
    float static D(float sensor, float actuating, float _setpoint, float _min, float _max, float _kd, float _dt) {
    	db<Controller>(WRN) << "Controller::D("  << _kd << ")" <<endl;

      // calculate error
      float error = _setpoint - sensor;
      // Derivative term
      // FIXME: use _prev_error
      double derivative;// = (error - _prev_error) / _dt;
      derivative = error / _dt;
      double output = _kd * derivative;

      output = Controller::checkLimits(output, _min, _max);

      // FIXME: set _prev_error
      // _prev_error = error;
      // actuating->act();
      return output;
    }

    // @params sensor, actuating, _setpoint, _min, _max, _kp, _kd, _dt
  	float static PD(float sensor, float actuating, float _setpoint, float _min, float _max, float _kp, float _kd, float _dt) {
  		db<Controller>(WRN) << "Controller::PD(" << _kp << "," << _kd << ")" <<endl;

      float pOut = Controller::P(sensor, actuating, _setpoint, _min, _max, _kp);
      float dOut = Controller::D(sensor, actuating, _setpoint, _min, _max, _kd, _dt);

      float output = pOut + dOut;

      output = Controller::checkLimits(output, _min, _max);

      return output;
  	}

    // @params sensor, actuating, _setpoint, _min, _max, _kp, _ki, _dt, _integral
    float static PI(float sensor, float actuating, float _setpoint, float _min, float _max, float _kp, float _ki, float _dt, float _integral) {
    	db<Controller>(WRN) << "Controller::PI(" << _kp << ","  << _ki << "," << _integral << ")" <<endl;

      float pOut = Controller::P(sensor, actuating, _setpoint, _min, _max, _kp);
      float iOut = Controller::I(sensor, actuating, _setpoint, _min, _max, _ki, _dt, _integral);

      float output = pOut + iOut;

      output = Controller::checkLimits(output, _min, _max);

      return output;
    }

    // @params sensor, actuating, _setpoint, _min, _max, _kp, _ki, _kd, _dt, _integral
    float static PID(float sensor, float actuating, float _setpoint, float _min, float _max, float _kp, float _ki, float _kd, float _dt, float _integral) {
    	db<Controller>(WRN) << "Controller::PID(" << _kp << "," << _ki << "," << _kd << "," << _integral << ")" <<endl;

      float pOut = Controller::P(sensor, actuating, _setpoint, _min, _max, _kp);
      float iOut = Controller::I(sensor, actuating, _setpoint, _min, _max, _ki, _dt, _integral);
      float dOut = Controller::D(sensor, actuating, _setpoint, _min, _max, _kd, _dt);

      float output = pOut + iOut + dOut;

      output = Controller::checkLimits(output, _min, _max);

      return output;
    }

  private:
    // void static setPrevError(float error){
    //   Controller::_prev_error = error;
    // }
    //
    // float static getPrevError(){
    //   return Controller::_prev_error;
    // }

    float static checkLimits(float value, float min, float max){
      // Restrict to max/min
      if( value > max ){
          value = max;
      }
      else if( value < min ){
          value = min;
      }
      return value;
    }
};

__END_SYS

#endif /* _CONTROLLER_H_ */
