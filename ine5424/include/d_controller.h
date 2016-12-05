#ifndef __D_controller_H_
#define __D_controller_H_

#include <controller2.h>

__BEGIN_SYS

class D_controller : public Controller2 {

    public:
    	template<typename ... Args>
    	D_controller(Sensor* sensor, Actuating* actuating,
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
      // @params _kd
    	float static D(float error, float prev_error, float dt, float integral, float kd) {
    		db<D_controller>(TRC) << "D_controller::D(error=" << error
    				<< ",prev_error=" << prev_error
    				<< ",dt=" << dt
    				<< ",integral=" << integral
    				<< ",kd=" << kd << ")" <<endl;

    		return CalculateD(error, dt, prev_error, kd);
    	}


      //@params: float error, float dt, float prev_error, float kd
    	float static CalculateD(float error, float dt, float prev_error, float kd) {
    		db<D_controller>(TRC) << "CalculateD(error=" << error << ",prev_error=" << prev_error << ",kd=" << kd << ")"<< endl;

    		// Derivative term
    		float derivative = (error - prev_error) / dt;
    		float dOut = kd * derivative;

    		return dOut;
    	}
};

__END_SYS

#endif /* _D_controller_H_ */
