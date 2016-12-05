#ifndef __I_CONTROLLER_H_
#define __I_CONTROLLER_H_

#include <controller2.h>

__BEGIN_SYS

class I_controller : public Controller2 {

    public:
    	template<typename ... Args>
    	I_controller(Sensor* sensor, Actuating* actuating,
        float max, float min, float setpoint, float dt,
    			   float (* control)(float, float, float, float, Args... args), Args... args)
      {
        _sensor = sensor;
        _actuating = actuating;
        _max = max;
        _min = min;
        _setpoint = setpoint;
        _dt = dt;
        _integral = 0;
        _error = 0;
        _prev_error = 0;

        Run(control, args...);
    	}

    public:
      // @params _ki
    	float static I(float error, float prev_error, float dt, float integral, float ki) {
    		db<I_controller>(WRN) << "I_controller::I(error=" << error
    				<< ",prev_error=" << prev_error
    				<< ",dt=" << dt
    				<< ",integral=" << integral
    				<< ",ki=" << ki << ")" <<endl;

    		return CalculateI(error, dt, ki, integral);
    	}

      //@params: float error, float dt, float ki, float integral
    	float static CalculateI(float error, float dt, float ki, float integral) {
    		db<I_controller>(WRN) << "CalculateI(error=" << error << ",ki=" << ki << ",integral=" << integral << endl;

    		float iOut = ki * integral;

    		return iOut;
    	}
};

__END_SYS

#endif /* _I_controller_H_ */
