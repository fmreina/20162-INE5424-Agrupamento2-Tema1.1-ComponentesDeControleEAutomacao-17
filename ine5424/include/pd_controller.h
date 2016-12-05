#ifndef __PD_controller_H_
#define __PD_controller_H_

#include <controller2.h>
#include <p_controller.h>
#include <d_controller.h>

__BEGIN_SYS

class PD_controller : public Controller2 {
// class PD_controller : public P_controller, public D_controller {

    public:
    	template<typename ... Args>
    	PD_controller(Sensor* sensor, Actuating* actuating,
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
      // @params _kp, _kd
    	float static PD(float error, float prev_error, float dt, float integral, float kp, float kd) {
    		db<PD_controller>(TRC) << "PD_controller::PD(error=" << error
    				<< ",prev_error=" << prev_error
    				<< ",dt=" << dt
    				<< ",integral=" << integral
    				<< ",kp=" << kp
    				<< ",kd=" << kd << ")" <<endl;

    		float pOut = P_controller::CalculateP(error, kp);
    		float dOut = D_controller::CalculateD(error, dt, prev_error, kd);

    		return pOut + dOut;
    	}

    protected:

};

__END_SYS

#endif /* _PD_controller_H_ */
