// EPOS Controller Unit Tests Program

#include <utility/ostream.h>
#include <utility/math.h>
#include <controller.h>
#include <sensor.h>
#include <actuating.h>

using namespace EPOS;

 OStream cout;

 // compare two floats with the floating precision of epsilon
 bool compare_floats(float A, float B, float epsilon = 0.005f) {
     return (abs(A - B) < epsilon);
 }

void p_unit_test(float max, float min, float pointView, float setpoint, float kp, float expected_result) {
  cout << "\n>>[Unit test : P controller]<<" << endl;
  cout << "- Entry parameters:" << endl;
  cout << "\tMax limit = " << max << endl;
  cout << "\tMin limit = " << min << endl;
  cout << "\tPointview = " << pointView << endl;
  cout << "\tSetpoint = " << setpoint << endl;
  cout << "\tkp = " << kp << endl;
  cout << "\tExpected output = " << expected_result << endl;

  cout << "Initializing P controller!" << endl;
	Controller* controller = new P(pointView, setpoint, kp);
  controller->setMax(max);
  controller->setMin(min);

  cout <<"\n!!! Assertion !!!"<< endl;
  assert(compare_floats(controller->calculate(), expected_result));
  cout << endl;

  if(compare_floats(controller->calculate(), expected_result))
  {
    cout << ".: Unit test : P controller : SUCCEED :." << endl;
  }
  else
  {
    cout << ".: Unit test : P controller : FAILED :." << endl;
    cout << "\tExpected (" << expected_result << ") but got (" << controller->calculate() << ")" <<endl;
  }

  cout << ">>[End of test]<<\n" << endl;
}

void i_unit_test(float max, float min, float pointView, float setpoint, float dt, float ki, float expected_result) {
  cout << "\n>>[Unit test : I controller]<<" << endl;
  cout << "- Entry parameters:" << endl;
  cout << "\tMax limit = " << max << endl;
  cout << "\tMin limit = " << min << endl;
  cout << "\tPointview = " << pointView << endl;
  cout << "\tSetpoint = " << setpoint << endl;
  cout << "\tdt = " << dt << endl;
  cout << "\tki = " << ki << endl;
  cout << "\tExpected output = " << expected_result << endl;

  cout << "Initializing I controller!" << endl;
	Controller* controller = new I(pointView, setpoint, dt, ki);
  controller->setMax(max);
  controller->setMin(min);

  cout <<"\n!!! Assertion !!!"<< endl;
  assert(compare_floats(controller->calculate(), expected_result));
  cout << endl;

  if(compare_floats(controller->calculate(), expected_result))
  {
    cout << ".: Unit test : I controller : SUCCEED :." << endl;
  }
  else
  {
    cout << ".: Unit test : I controller : FAILED :." << endl;
    cout << "\tExpected (" << expected_result << ") but got (" << controller->calculate() << ")" <<endl;
  }

  cout << ">>[End of test]<<\n" << endl;
}

void d_unit_test(float max, float min, float pointView, float setpoint, float dt, float kd, float expected_result) {
  cout << "\n>>[Unit test : D controller]<<" << endl;
  cout << "- Entry parameters:" << endl;
  cout << "\tMax limit = " << max << endl;
  cout << "\tMin limit = " << min << endl;
  cout << "\tPointview = " << pointView << endl;
  cout << "\tSetpoint = " << setpoint << endl;
  cout << "\tdt = " << dt << endl;
  cout << "\tkd = " << kd << endl;
  cout << "\tExpected output = " << expected_result << endl;

  cout << "Initializing D controller!" << endl;
	Controller* controller = new D(pointView, setpoint, dt, kd);
  controller->setMax(max);
  controller->setMin(min);

  cout <<"\n!!! Assertion !!!"<< endl;
  assert(compare_floats(controller->calculate(), expected_result));
  cout << endl;

  if(compare_floats(controller->calculate(), expected_result))
  {
    cout << ".: Unit test : D controller : SUCCEED :." << endl;
  }
  else
  {
    cout << ".: Unit test : D controller : FAILED :." << endl;
    cout << "\tExpected (" << expected_result << ") but got (" << controller->calculate() << ")" <<endl;
  }

  cout << ">>[End of test]<<\n" << endl;
}

void pi_unit_test(float max, float min, float pointView, float setpoint, float dt, float kp, float ki, float expected_result) {
  cout << "\n>>[Unit test : PI controller]<<" << endl;
  cout << "- Entry parameters:" << endl;
  cout << "\tMax limit = " << max << endl;
  cout << "\tMin limit = " << min << endl;
  cout << "\tPointview = " << pointView << endl;
  cout << "\tSetpoint = " << setpoint << endl;
  cout << "\tdt = " << dt << endl;
  cout << "\tkp = " << kp << endl;
  cout << "\tki = " << ki << endl;
  cout << "\tExpected output = " << expected_result << endl;

  cout << "Initializing PI controller!" << endl;
	Controller* controller = new PI(pointView, setpoint, dt, kp, ki);
  controller->setMax(max);
  controller->setMin(min);

  cout <<"\n!!! Assertion !!!"<< endl;
  assert(compare_floats(controller->calculate(), expected_result));
  cout << endl;

  if(compare_floats(controller->calculate(), expected_result))
  {
    cout << ".: Unit test : PI controller : SUCCEED :." << endl;
  }
  else
  {
    cout << ".: Unit test : PI controller : FAILED :." << endl;
    cout << "\tExpected (" << expected_result << ") but got (" << controller->calculate() << ")" <<endl;
  }

  cout << ">>[End of test]<<\n" << endl;
}

void pd_unit_test(float max, float min, float pointView, float setpoint, float dt, float kp, float kd, float expected_result) {
  cout << "\n>>[Unit test : PD controller]<<" << endl;
  cout << "- Entry parameters:" << endl;
  cout << "\tMax limit = " << max << endl;
  cout << "\tMin limit = " << min << endl;
  cout << "\tPointview = " << pointView << endl;
  cout << "\tSetpoint = " << setpoint << endl;
  cout << "\tdt = " << dt << endl;
  cout << "\tkp = " << kp << endl;
  cout << "\tkd = " << kd << endl;
  cout << "\tExpected output = " << expected_result << endl;

  cout << "Initializing PD controller!" << endl;
	Controller* controller = new PD(pointView, setpoint, dt, kp, kd);
  controller->setMax(max);
  controller->setMin(min);

  cout <<"\n!!! Assertion !!!"<< endl;
  assert(compare_floats(controller->calculate(), expected_result));
  cout << endl;

  if(compare_floats(controller->calculate(), expected_result))
  {
    cout << ".: Unit test : PD controller : SUCCEED :." << endl;
  }
  else
  {
    cout << ".: Unit test : PD controller : FAILED :." << endl;
    cout << "\tExpected (" << expected_result << ") but got (" << controller->calculate() << ")" <<endl;
  }

  cout << ">>[End of test]<<\n" << endl;
}

void pid_unit_test(float max, float min, float pointView, float setpoint, float dt, float kp, float ki, float kd, float expected_result) {
  cout << "\n>>[Unit test : PID controller]<<" << endl;
  cout << "- Entry parameters:" << endl;
  cout << "\tMax limit = " << max << endl;
  cout << "\tMin limit = " << min << endl;
  cout << "\tPointview = " << pointView << endl;
  cout << "\tSetpoint = " << setpoint << endl;
  cout << "\tdt = " << dt << endl;
  cout << "\tkp = " << kp << endl;
  cout << "\tki = " << ki << endl;
  cout << "\tkd = " << kd << endl;
  cout << "\tExpected output = " << expected_result << endl;

  cout << "Initializing PID controller!" << endl;
	Controller* controller = new PID(pointView, setpoint, dt, kp, ki, kd);
  controller->setMax(max);
  controller->setMin(min);

  cout <<"\n!!! Assertion !!!"<< endl;
  assert(compare_floats(controller->calculate(), expected_result));
  cout << endl;

  if(compare_floats(controller->calculate(), expected_result))
  {
    cout << ".: Unit test : PID controller : SUCCEED :." << endl;
  }
  else
  {
    cout << ".: Unit test : PID controller : FAILED :." << endl;
    cout << "\tExpected (" << expected_result << ") but got (" << controller->calculate() << ")" <<endl;
  }

  cout << ">>[End of test]<<\n" << endl;
}

int main()
{
  cout << "\n.: Unit Tests :." << endl;

  float max = 100;
  float min = -100;
  float pointView = 10.0f;
  float setpoint = 1;
  float dt = 0.5;

  float kp = 1.16;
  float expected_result = -10.439;
  p_unit_test(max, min, pointView, setpoint, kp, expected_result);

  float ki = 1.65;
  expected_result = -7.424;
  i_unit_test(max, min, pointView, setpoint, dt, ki, expected_result);

  float kd = 1.65;
  expected_result = -29.699;
  d_unit_test(max, min, pointView, setpoint, dt, kd, expected_result);

  kp = 1.16;
  ki = 1.65;
  expected_result = -17.864;
  pi_unit_test(max, min, pointView, setpoint, dt, kp, ki, expected_result);

  kp = 1.16;
  kd = 1.65;
  expected_result = -40.139;
  pd_unit_test(max, min, pointView, setpoint, dt, kp, kd, expected_result);

  kp = 1.16;
  ki = 1.65;
  kd = 1.65;
  expected_result = -47.564;
  pid_unit_test(max, min, pointView, setpoint, dt, kp, ki, kd, expected_result);

  cout << "The end!" << endl;

  return 0;
}
