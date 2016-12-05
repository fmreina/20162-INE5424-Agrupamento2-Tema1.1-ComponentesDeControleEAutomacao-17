#ifndef __PID_controller_H_
#define __PID_controller_H_

#include <controller2.h>
#include <p_controller.h>
#include <i_controller.h>
#include <d_controller.h>

__BEGIN_SYS

class PID_controller : public Controller2 {
// class PD_controller : public P_controller, public I_controller, public D_controller {

    public:
    	template<typename ... Args>
    	PID_controller(Sensor* sensor, Actuating* actuating,
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
      // @params _kp, _ki, _kd
    	float static PID(float error, float prev_error, float dt, float integral, float kp, float ki, float kd) {
    		db<PID_controller>(TRC) << "PID_controller::PID(error=" << error
    				<< ",prev_error=" << prev_error
    				<< ",dt=" << dt
    				<< ",integral=" << integral
    				<< ",kp=" << kp
    				<< ",ki=" << ki
    				<< ",kd=" << kd << ")" <<endl;

    		float pOut = P_controller::CalculateP(error, kp);
    		float iOut = I_controller::CalculateI(error, dt, ki, integral);
    		float dOut = D_controller::CalculateD(error, dt, prev_error, kd);

    		return pOut + iOut + dOut;
    	}


    protected:

};

__END_SYS

#endif /* _PID_controller_H_ */
