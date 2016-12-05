// EPOS PID Controller2 Test Program

#include <utility/ostream.h>
#include <utility/math.h>
#include <controller2.h>
#include <p_controller.h>
#include <i_controller.h>
#include <d_controller.h>
#include <pi_controller.h>
#include <pd_controller.h>
#include <pid_controller.h>
#include <sensor.h>
#include <actuating.h>

using namespace EPOS;

OStream cout;

// compare two floats with the floating precision of epsilon
bool compare_floats(float A, float B, float epsilon = 0.001f) {
	return (abs(A - B) < epsilon);
}

void runPTest() {
	float max = 100;
	float min = -100;
	float kp = 1.6;
	float dt = 0.5;
	float setpoint = 1;

	Sensor* sensor = new Sensor(0);
	Actuating* actuating = new Actuating();

	cout << "Setting values "
	<< "\nkp=" << kp
	<< "\ndt=" << dt
	<< "\nst=" << setpoint
	<< endl;

	cout << "Initializing P controller!" << endl;
	Controller2* pctrl = new P_controller(sensor, actuating, max, min, setpoint, dt, &P_controller::P, kp);

	float position = 0;
	int i = 1;
	while(!compare_floats(position, setpoint, 0.001f)) {
		position += actuating->read() * 0.24;
		sensor->set(position);
		cout << "POSITION="<< position << ";\nITERATION=" << i << endl;
		pctrl->Run(&P_controller::P, kp);
		i++;
	}
	cout << endl;
}

void runITest() {
	float max = 100;
	float min = -100;
	float ki = 1.2;
	float dt = 0.5;
	float setpoint = 1;

	Sensor* sensor = new Sensor(0);
	Actuating* actuating = new Actuating();

	cout << "Setting values "
	<< "\nki=" << ki
	<< "\ndt=" << dt
	<< "\nst=" << setpoint
	<< endl;

	cout << "Initializing I controller!" << endl;
	Controller2* ictrl = new I_controller(sensor, actuating, max, min, setpoint, dt, &I_controller::I, ki);

	float position = 0;
	int i = 1;
	// while(!compare_floats(position, setpoint, 0.001f)) {
	for(;i<51;){
		position += actuating->read() * 0.24;
		sensor->set(position);
		cout << "POSITION="<< position << ";\nITERATION=" << i << endl;
		ictrl->Run(&I_controller::I, ki);
		i++;
	}
	cout << endl;
}

void runDTest() {
	float max = 100;
	float min = -100;
	float kd = 0.2;
	float dt = 0.5;
	float setpoint = 1;

	Sensor* sensor = new Sensor(0);
	Actuating* actuating = new Actuating();

	cout << "Setting values "
	<< "\nkd=" << kd
	<< "\ndt=" << dt
	<< "\nst=" << setpoint
	<< endl;

	cout << "Initializing D controller!" << endl;
	Controller2* dctrl = new D_controller(sensor, actuating, max, min, setpoint, dt, &D_controller::D, kd);

	float position = 0;
	int i = 1;
	// while(!compare_floats(position, setpoint, 0.001f)) {
	for(;i<51;){
		position += actuating->read() * 0.24;
		sensor->set(position);
		cout << "POSITION="<< position << ";\nITERATION=" << i << endl;
		dctrl->Run(&D_controller::D, kd);
		i++;
	}
	cout << endl;
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
	Controller2* pictrl = new PI_controller(sensor, actuating, max, min, setpoint, dt, &PI_controller::PI, kp, ki);

	float position = 0;
	int i = 1;
	while(!compare_floats(position, setpoint, 0.001f)) {
		position += actuating->read() * 0.24;
		sensor->set(position);
		cout << "POSITION="<< position << ";\nITERATION=" << i << endl;
		pictrl->Run(&PI_controller::PI, kp, ki);
		i++;
	}
	cout << endl;
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
	Controller2* pdctrl = new PD_controller(sensor, actuating, max, min, setpoint, dt, &PD_controller::PD, kp, kd);

	float position = 0;
	int i = 1;
	while(!compare_floats(position, setpoint, 0.001f)) {
		position += actuating->read() * 0.24;
		sensor->set(position);
		cout << "POSITION="<< position << ";\nIteration=" << i << endl;
		pdctrl->Run(&PD_controller::PD, kp, kd);
		i++;
	}
	cout << endl;
}


void runPIDTest() {
	float max = 100;
	float min = -100;
	float ki = 1.2;
	float kp = 1.6;
	float kd = 0.2;
	float dt = 0.5;
	float setpoint = 1;

	Sensor* sensor = new Sensor(0);
	Actuating* actuating = new Actuating();

	cout << "Setting values "
				<< "\nkp=" << kp
				<< "\nki=" << ki
				<< "\nkd=" << kd
				<< "\ndt=" << dt
				<< "\nst=" << setpoint
				<< endl;

	cout << "Initializing PID controller!" << endl;
	Controller2* pctrl = new PID_controller(sensor, actuating, max, min, setpoint, dt, &PID_controller::PID, kp, ki, kd);

	float position = 0;
	int i = 1;
	while(!compare_floats(position, setpoint, 0.001f)) {
		position += actuating->read() * 0.24;
		sensor->set(position);
		cout << "POSITION="<< position << ";\nITERATION=" << i << endl;
		pctrl->Run(&PID_controller::PID, kp, ki, kd);
		i++;
	}
	cout << endl;
}

int main()
{
	cout << ".:Proportional Integrative Derivative Controller2 Test:." << endl;

	runPTest();
	// runITest();
	// runDTest();
	// runPITest();
	// runPDTest();
	// runPIDTest();

	cout << "The end!" << endl;

	return 0;
}
