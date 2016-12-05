#ifndef CLOSED_LOOP_H_
#define CLOSED_LOOP_H_

#include <sensor.h>
#include <actuating.h>
#include <controller.h>
#include <periodic_thread.h>

__BEGIN_SYS

class Closed_Loop
{
public:
	Closed_Loop(Sensor* _sensor, Actuating* _actuating, Controller* _controller, float _dt) :
		sensor(_sensor),
		actuating(_actuating),
		controller(_controller)
	{

	    Periodic_Thread p_thread(RTConf(_dt * 1000, 100), &run);
		p_thread.join();
	}

	virtual ~Closed_Loop();

	Sensor* sensor;
	Actuating* actuating;
	Controller* controller;
	float pv, result;

	int run(void) {
		pv = sensor->read();
		controller->setPointView(pv);
		result = controller->calculate();
		actuating->act(result);
		return 0;
	}
};

__END_SYS

#endif /* _CONTROLLER_H_ */
