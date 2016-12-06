#ifndef __CONTROLLER_H_
#define __CONTROLLER_H_


__BEGIN_SYS

class Controller
{

public:
	Controller() {};
	virtual ~Controller() {};

	void setSetPoint(float _st) { setpoint = _st; }
	void setPointView(float _pv) { pointView = _pv; }
	void setMax(float _max) { max = _max; }
	void setMin(float _min) { min = _min; }
	float getSetPoint(){ return setpoint; }

	float setpoint, pointView, error, dt, max, min;

	virtual float calculate() = 0;

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
	virtual ~P() {};

	float kp;

	float calculate() {
		db<P>(TRC) << "P::calculate()" << endl;

		error = setpoint - pointView;

		float _pOut = kp * error;

		return _pOut > max ? max : _pOut < min ? min : _pOut;
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
	virtual ~I() {};

	float ki, integral;

	float calculate() {
		db<I>(TRC) << "I::calculate()" << endl;

		error = setpoint - pointView;
		integral += error * dt;

		float _iOut = ki * integral;

		return _iOut > max ? max : _iOut < min ? min : _iOut;
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
	virtual ~D() {};

	float kd, derivative, prevError;

	float calculate() {
		db<D>(TRC) << "P::calculate()" << endl;

		error = setpoint - pointView;
		derivative = (error - prevError) / dt;
		prevError = error;

		float _dOut = kd * derivative;

	return _dOut > max ? max : _dOut < min ? min : _dOut;
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
	virtual ~PI() {};

	float kp, ki, integral;

	float calculate() {
		db<PI>(TRC) << "PI::calculate()" << endl;

		error = setpoint - pointView;
		integral += error * dt;

		float _piOut = (kp * error) + (ki * integral);

		return _piOut > max ? max : _piOut < min ? min : _piOut;
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
	virtual ~PD() {};

	float kp, kd, derivative, prevError;

	float calculate() {
		db<PD>(TRC) << "PD::calculate()" << endl;

		error = setpoint - pointView;
		derivative = (error - prevError) / dt;
		prevError = error;

		float _pdOut = (kp * error) + (kd * derivative);

		return _pdOut > max ? max : _pdOut < min ? min : _pdOut;
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
	virtual ~PID() {};

	float kp, ki, kd, integral, derivative, prevError;

	float calculate() {
		db<D>(TRC) << "PID::calculate()" << endl;

		error = setpoint - pointView;
		integral += error * dt;
		derivative = (error - prevError) / dt;
		prevError = error;

		float _pidOut = (kp * error) + (ki * integral) + (kd * derivative);

		return _pidOut > max ? max : _pidOut < min ? min : _pidOut;
	}
};

__END_SYS

#endif /* _CONTROLLER_H_ */
