#ifndef CLOSED_LOOP_H_
#define CLOSED_LOOP_H_

#include <sensor.h>
#include <actuating.h>
#include <controller.h>
#include <periodic_thread.h>

__BEGIN_SYS

class Closed_Loop
{
private:
	const static int iterations = 100;

public:
	Closed_Loop(Sensor* _sensor, Actuating* _actuating, Controller* _controller, float _dt) :
		sensor(_sensor),
		actuating(_actuating),
		controller(_controller),
		dt(_dt) { }

	virtual ~Closed_Loop();

	Sensor* sensor;
	Actuating* actuating;
	Controller* controller;
	float pv, result, dt;

	void startLoop() {
	    Periodic_Thread p_thread(RTConf(dt * 1000, iterations), &Closed_Loop::run());
		p_thread.join();
	}

	int run() {
		db<Closed_Loop>(TRC) << "Closed_Loop::run()" << endl;
		for(int i = 0; i < iterations; i++) {
			Periodic_Thread::wait_next();
			pv = sensor->read();
			controller->setPointView(pv);
			result = controller->calculate();
			actuating->act(result);
		}
		return 0;
	}
};

__END_SYS

#endif /* _CONTROLLER_H_ */
