#ifndef __PI_controller_H_
#define __PI_controller_H_

#include <controller2.h>
#include <p_controller.h>
#include <i_controller.h>

__BEGIN_SYS

class PI_controller : public Controller2 {
// class PI_controller : public P_controller, public I_controller {

    public:
    	template<typename ... Args>
    	PI_controller(Sensor* sensor, Actuating* actuating,
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
      // @params _kp, _ki
    	float static PI(float error, float prev_error, float dt, float integral, float kp, float ki) {
    		db<PI_controller>(TRC) << "PI_controller::PI(error=" << error
    				<< ",prev_error=" << prev_error
    				<< ",dt=" << dt
    				<< ",integral=" << integral
    				<< ",kp=" << kp
    				<< ",ki=" << ki << ")" <<endl;

    		float pOut = P_controller::CalculateP(error, kp);
    		float iOut = I_controller::CalculateI(error, dt, ki, integral);

    		return pOut + iOut;
    	}

};

__END_SYS

#endif /* _PI_controller_H_ */
