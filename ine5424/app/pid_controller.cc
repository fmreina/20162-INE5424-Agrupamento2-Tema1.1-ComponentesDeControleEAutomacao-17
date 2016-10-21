// EPOS Scheduler Test Program

#include <utility/ostream.h>
#include <d_controller.h>
#include <i_controller.h>
#include <p_controller.h>

using namespace EPOS;

 OStream cout;

int main()
{
    double output = 0;
    double max = 70;
    double min = -70;
    cout << "setting values for variables kd, pv and setpoint" << endl;
    double kd = 1.65;
    double dt = 0.5;
    double pv = 20;
    double setpoint = 50;

    cout << "Initializing D controller" << endl;
    P* pctrl = new P(kd, max, min);
    I* ictrl = new I(kd, dt, max, min);
    D* dctrl = new D(kd, dt, max, min);

    cout << "Calculating D output" << endl;
    output += pctrl->calculate(setpoint, pv);
    output += ictrl->calculate(setpoint, pv);
    output += dctrl->calculate(setpoint, pv);

    cout << "Output calculated but I dont know how to show!" << endl;

    cout << "The end!" << endl;

    return 0;
}
