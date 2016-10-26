// EPOS Scheduler Test Program

#include <utility/ostream.h>
#include <d_controller.h>
#include <i_controller.h>
#include <p_controller.h>

using namespace EPOS;

 OStream cout;

int main()
{
    float output = 0;
    float max = 2;
    float min = -2;
    cout << "setting values for variables kp, ki, kd, pv and setpoint" << endl;
    float kp = 1.6;
    float ki = 1;
    float kd = 1;
    float dt = 0.02;
    float pv = 0;
    float setpoint = 1;

    cout << "Initializing P, I and D controller" << endl;
    P* pctrl = new P(kp, max, min);
    I* ictrl = new I(ki, dt, max, min);
    D* dctrl = new D(kd, dt, max, min);

    cout << "Calculating PID output" << endl;
    output = pctrl->calculate(setpoint, pv);
    output += ictrl->calculate(setpoint, pv);
    output += dctrl->calculate(setpoint, pv);

    cout << "output = " << output << endl;

    cout << "The end!" << endl;

    return 0;
}
