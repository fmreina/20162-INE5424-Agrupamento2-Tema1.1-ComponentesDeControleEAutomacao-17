// EPOS PID Controller Test Program

#include <utility/ostream.h>
#include <utility/math.h>
#include <controller.h>
#include <sensor.h>
#include <actuating.h>

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
	float setpoint = 1;

	Sensor* sensor = new Sensor(0);
	Actuating* actuating = new Actuating();

	cout << "Initializing PD controller!" << endl;
	Controller* pctrl = new Controller(sensor, actuating, max, min, setpoint, dt, &Controller::PD, kp, kd);

	cout << "The end!" << endl;

	return 0;
}
