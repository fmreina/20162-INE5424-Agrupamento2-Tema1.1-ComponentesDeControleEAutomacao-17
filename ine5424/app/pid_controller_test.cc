// EPOS Scheduler Test Program

#include <utility/ostream.h>
#include <controller.h>
// #include <controller_interface.h>
// #include <sensor_interface.h>
// #include <actuating_interface.h>

using namespace EPOS;

 OStream cout;

int main()
{
    cout << ".:Proportional Integrative Derivative Controller Test:." << endl;

    float max = 70;
    float min = -70;
    float dOutput, iOutput, pOutput, output;

    float kd = 1.65;
    float ki = 1.65;
    float kp = 1.65;
    float integral = 0.55;
    float dt = 0.5;
    float pv = 20;
    float setpoint = 50;

    cout << "Setting values "
         << "\nkp=" << kp
         << "\nki=" << ki
         << "\nkd=" << kd
         << "\ndt=" << dt
         << "\npv=" << pv
         << "\nst=" << setpoint
    << endl;

  cout << "Initializing Sensor!" << endl;
  // Sensor_Interface* sensor = new Sensor_Interface(10);

  // Actuating_Interface* actuating = new Actuating_Interface();

	cout << "Initializing P controller!" << endl;
  float sensor = 10;
  float actuating = 10;
	Controller* pctrl = new Controller(&Controller::P, sensor, actuating, kp);
  // Controller_Interface* _controller = new Controller_Interface(sensor, actuating, dt, &Controller_Interface::P, kp);

	// cout << "Initializing I controller!" << endl;
	// Controller* ictrl = new Controller(sensor, 10, dt, &Controller::I, ki, integral);
  //
	// cout << "Initializing D controller!" << endl;
	// Controller* dctrl = new Controller(sensor, 10, dt, &Controller::D, kd);
  //
	// cout << "Initializing PD controller!" << endl;
	// Controller* pdctrl = new Controller(sensor, 10, dt, &Controller::PD, kp, kd);
  //
	// cout << "Initializing PI controller!" << endl;
	// Controller* pictrl = new Controller(sensor, 10, dt, &Controller::PI, kp, ki, integral);
  //
	// cout << "Initializing PID controller!" << endl;
	// Controller* pidctrl = new Controller(sensor, 10, dt, &Controller::PID, kp, ki, kd, integral);

    cout << "output = " << output << endl;

    cout << "The end!" << endl;

    return 0;
}
