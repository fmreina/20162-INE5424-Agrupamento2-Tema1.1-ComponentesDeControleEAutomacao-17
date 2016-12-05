// EPOS PID Controller Test Program

#include <utility/ostream.h>
#include <utility/math.h>
#include <controller.h>
#include <sensor.h>
#include <actuating.h>
#include <closed_loop.h>

using namespace EPOS;

OStream cout;

int main()
{
	cout << ".:Proportional Integrative Derivative Controller Test:." << endl;

	float max = 100;
	float min = -100;
	float kd = 0.2;
	float kp = 1.6;
	float dt = 0.5;
	float setpoint = 1.0f;
	float pointView = 0.0f;

	Sensor* sensor = new Sensor(0);
	Actuating* actuating = new Actuating();

	cout << "Initializing PD controller!" << endl;
	Controller* pd = new PD(setpoint, pointView, dt, kp, kd);

	cout << "The end!" << endl;

	Closed_Loop* cl = new Closed_Loop(sensor, actuating, pd, dt);


	return 0;
}
