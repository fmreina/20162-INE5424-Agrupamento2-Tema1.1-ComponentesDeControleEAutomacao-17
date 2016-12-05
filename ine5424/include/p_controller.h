#ifndef __P_CONTROLLER_H_
#define __P_CONTROLLER_H_

#include <controller2.h>

__BEGIN_SYS

class P_controller : public Controller2 {

    public:
    	template<typename ... Args>
    	P_controller(Sensor* sensor, Actuating* actuating,
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
      // @params _kp
    	float static P(float error, float prev_error, float dt, float integral, float kp) {
    		db<P_controller>(TRC) << "P_controller::P(error=" << error
    				<< ",prev_error=" << prev_error
    				<< ",dt=" << dt
    				<< ",integral=" << integral
    				<< ",kp=" << kp << ")" <<endl;

    		return CalculateP(error, kp);
    	}

    // protected:
      //	@params: float error, float kp
      	float static CalculateP(float error, float kp) {
      		db<P_controller>(TRC) << "CalculateP(error=" << error << ",kp=" << kp << ")" << endl;

      		// proportional result
      		float pOut = kp * error;

      		return pOut;
      	}
};

__END_SYS

#endif /* _P_CONTROLLER_H_ */
