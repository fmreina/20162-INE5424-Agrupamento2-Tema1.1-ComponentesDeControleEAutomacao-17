// EPOS Scheduler Test Program

#include <utility/ostream.h>
#include <controller2.h>
#include <p_controller.h>

using namespace EPOS;

 OStream cout;

Controller2 * pctrl;

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

  float sensor = 10;
  float actuating = 10;

  cout << "Initializing P controller!" << endl;
  pctrl = new P::P(sensor, actuating, setpoint, min, max, kp);

  // cout << "Initializing P controller!" << endl;
	// Controller* pctrl = new Controller(&Controller::P, sensor, actuating, setpoint, min, max, kp);
  //
	// cout << "Initializing I controller!" << endl;
	// Controller* ictrl = new Controller(&Controller::I, sensor, actuating, setpoint, min, max, ki, dt, integral);
  //
	// cout << "Initializing D controller!" << endl;
	// Controller* dctrl = new Controller(&Controller::D, sensor, actuating, setpoint, min, max, kd, dt);
  //
	// cout << "Initializing PD controller!" << endl;
	// Controller* pdctrl = new Controller(&Controller::PD, sensor, actuating, setpoint, min, max, kp, kd, dt);
  //
	// cout << "Initializing PI controller!" << endl;
	// Controller* pictrl = new Controller(&Controller::PI, sensor, actuating, setpoint, min, max, kp, ki, dt, integral);
  //
	// cout << "Initializing PID controller!" << endl;
	// Controller* pidctrl = new Controller(&Controller::PID, sensor, actuating, setpoint, min, max, kp, ki, kd, dt, integral);

    cout << "output = " << output << endl;

    cout << "The end!" << endl;

    return 0;
}
