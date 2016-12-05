#ifndef __CONTROLLER_H_
#define __CONTROLLER_H_

#include <periodic_thread.h>
#include <sensor.h>
#include <actuating.h>

__BEGIN_SYS

class Controller
{

public:
	Controller();
	virtual ~Controller();

	void setSetPoint(float _st) { setpoint = _st; }
	void setPointView(float _pv) { pointView = _pv; }
	void setMax(float _max) { max = _max; }
	void setMin(float _min) { min = _min; }

	float setpoint, pointView, error, dt, max, min;

	virtual float calculate();

};


class P : public Controller
{
public:
	P(float _pv, float _st, float _kp) :
		kp(_kp)
	{
		pointView = _pv;
		setpoint = _st;
		error = 0;
	};
	~P();

	float kp;

	float calculate() {
		db<P>(TRC) << "P::calculate()" << endl;

		error = setpoint - pointView;
		return kp * error;
	}
};

class I : public Controller
{
public:
	I(float _pv, float _st, float _dt, float _ki) :
		ki(_ki),
		integral(0)
	{
		pointView = _pv;
		setpoint = _st;
		dt = _dt;
		error = 0;
	};
	~I();

	float ki, integral;

	float calculate() {
		db<I>(TRC) << "I::calculate()" << endl;

		error = setpoint - pointView;
		integral += error * dt;

		return ki * error;
	}
};

class D : public Controller
{
public:
	D(float _pv, float _st, float _dt, float _kd) :
		kd(_kd),
		derivative(0),
		prevError(0)
	{
		pointView = _pv;
		setpoint = _st;
		dt = _dt;
		error = 0;
	};
	~D();

	float kd, derivative, prevError;

	float calculate() {
		db<D>(TRC) << "P::calculate()" << endl;

		error = setpoint - pointView;
		derivative = (error - prevError) / dt;
		prevError = error;

		return kd * error;
	}
};

class PI : public Controller
{
public:
	PI(float _st, float _pv, float _dt, float _kp, float _ki) :
		kp(_kp),
		ki(_ki),
		integral(0)
	{
		pointView = _pv;
		setpoint = _st;
		dt = _dt;
		error = 0;
	};
	~PI();

	float kp, ki, integral;

	float calculate() {
		db<PI>(TRC) << "PI::calculate()" << endl;

		error = setpoint - pointView;
		integral += error * dt;

		return error * (kp + ki);
	}
};

class PD : public Controller
{
public:
	PD(float _st, float _pv, float _dt, float _kp, float _kd) :
		kp(_kp),
		kd(_kd),
		derivative(0),
		prevError(0)
	{
		pointView = _pv;
		setpoint = _st;
		dt = _dt;
		error = 0;
	};
	~PD();

	float kp, kd, derivative, prevError;

	float calculate() {
		db<PD>(TRC) << "PD::calculate()" << endl;

		error = setpoint - pointView;
		derivative = (error - prevError) / dt;
		prevError = error;

		return error * (kp + kd);
	}
};

class PID : public Controller
{
public:
	PID(float _st, float _pv, float _dt, float _kp, float _ki, float _kd) :
		kp(_kp),
		ki(_ki),
		kd(_kd),
		integral(0),
		derivative(0),
		prevError(0)
	{
		pointView = _pv;
		setpoint = _st;
		dt = _dt;
		error = 0;
	};
	~PID();

	float kp, ki, kd, integral, derivative, prevError;

	float calculate() {
		db<D>(TRC) << "P::calculate()" << endl;

		error = setpoint - pointView;
		integral += error * dt;
		derivative = (error - prevError) / dt;
		prevError = error;

		return error * (kp + ki + kd);
	}
};

__END_SYS

#endif /* _CONTROLLER_H_ */
