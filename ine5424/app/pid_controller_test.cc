// EPOS PID Controller Test Program

#include <utility/ostream.h>
#include <utility/math.h>
#include <controller.h>
#include <sensor.h>
#include <actuating.h>

using namespace EPOS;

OStream cout;

// compare two floats with the floating precision of epsilon
bool compare_floats(float A, float B, float epsilon = 0.001f) {
	return (abs(A - B) < epsilon);
}

void runPDTest() {
	float max = 100;
	float min = -100;
	float kd = 0.2;
	float kp = 1.6;
	float dt = 0.5;
	float setpoint = 1;

	Sensor* sensor = new Sensor(0);
	Actuating* actuating = new Actuating();

	cout << "Setting values "
			<< "\nkp=" << kp
			<< "\nkd=" << kd
			<< "\ndt=" << dt
			<< "\nst=" << setpoint
			<< endl;

	cout << "Initializing PD controller!" << endl;
	Controller* pctrl = new Controller(sensor, actuating, max, min, setpoint, dt, &Controller::PD, kp, kd);

	float position = 0;
	int i = 1;
	while(!compare_floats(position, setpoint, 0.001f)) {
		position += actuating->read() * 0.24;
		sensor->set(position);
		cout << "POSITION="<< position << ";\nIteration=" << i << endl;
		pctrl->Run(&Controller::PD, kp, kd);
		i++;
	}
}

void runPITest() {
	float max = 100;
	float min = -100;
	float ki = 1.2;
	float kp = 1.6;
	float dt = 0.5;
	float setpoint = 1;

	Sensor* sensor = new Sensor(0);
	Actuating* actuating = new Actuating();

	cout << "Setting values "
				<< "\nkp=" << kp
				<< "\nki=" << ki
				<< "\ndt=" << dt
				<< "\nst=" << setpoint
				<< endl;

	cout << "Initializing PI controller!" << endl;
	Controller* pctrl = new Controller(sensor, actuating, max, min, setpoint, dt, &Controller::PI, kp, ki);

	float position = 0;
	int i = 1;
	while(!compare_floats(position, setpoint, 0.001f)) {
		position += actuating->read() * 0.24;
		sensor->set(position);
		cout << "POSITION="<< position << ";\nITERATION=" << i << endl;
		pctrl->Run(&Controller::PI, kp, ki);
		i++;
	}
}

int main()
{
	cout << ".:Proportional Integrative Derivative Controller Test:." << endl;

//	runPDTest();
//	runPITest();

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

//	for(int i = 0; i < 100; i++)
//		cout << "Doing other stuff" << endl;

	cout << "The end!" << endl;

	return 0;
}
