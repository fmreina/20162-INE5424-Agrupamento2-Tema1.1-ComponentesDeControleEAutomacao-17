// EPOS Scheduler Test Program

#include <utility/ostream.h>
#include <controller.h>
#include <sensor.h>
#include <actuating.h>

using namespace EPOS;

 OStream cout;

int main()
{
    cout << ".:Proportional Integrative Derivative Controller Test:." << endl;

    float max = 2;
    float min = -1;

    float kd = 1.65;
    float ki = 1.65;
    float kp = 1.16;
    float integral = 0.55;
    float dt = 0.5;
    float setpoint = 1;
    Sensor* sensor = new Sensor(10);
    Actuating* actuating = new Actuating();

    cout << "Setting values "
         << "\nkp=" << kp
         << "\nki=" << ki
         << "\nkd=" << kd
         << "\ndt=" << dt
         << "\nst=" << setpoint
    << endl;

	cout << "Initializing P controller!" << endl;
	Controller* pctrl = new Controller(sensor, actuating, max, min, setpoint, dt, &Controller::P, kp);

	cout << "Initializing I controller!" << endl;
	Controller* ictrl = new Controller(sensor, actuating, max, min, setpoint, dt, &Controller::I, ki);

	cout << "Initializing D controller!" << endl;
	Controller* dctrl = new Controller(sensor, actuating, max, min, setpoint, dt, &Controller::D, kd);

	cout << "Initializing PD controller!" << endl;
	Controller* pdctrl = new Controller(sensor, actuating, max, min, setpoint, dt, &Controller::PD, kp, kd);

	cout << "Initializing PI controller!" << endl;
	Controller* pictrl = new Controller(sensor, actuating, max, min, setpoint, dt, &Controller::PI, kp, ki);

	cout << "Initializing PID controller!" << endl;
	Controller* pidctrl = new Controller(sensor, actuating, max, min, setpoint, dt, &Controller::PID, kp, ki, kd);

    cout << "The end!" << endl;

    return 0;
}
