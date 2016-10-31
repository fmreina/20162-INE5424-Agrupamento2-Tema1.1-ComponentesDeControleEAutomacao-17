#ifndef __CONTROLLER_INTERFACE_H_
#define __CONTROLLER_INTERFACE_H_

#include <sensor_interface.h>
#include <actuating_interface.h>

__BEGIN_SYS

class Controller_Interface
{

  public:
		template<typename ... Args>
    Controller_Interface(float sensor, Actuating_Interface* actuating,
      float dt, void (* control)(Args... args), Args... args):
    	_sensor(sensor), _actuating(actuating), _dt(dt)
      {
        db<Controller_Interface>(WRN) << "Controller_Interface() => " << this << endl;
        Run(control, args...);
      }

    ~Controller_Interface(){
      db<Controller_Interface>(WRN) << "~Controller_Interface() => " << this << endl;
    };

    template<typename ... Args>
    void Run(void (* _control)(Args... args), Args... args)
    {
      db<Controller_Interface>(WRN) << "Controller_Interface::Run() " << endl;
      // _sensor_value = _sensor->read();
      _actuating->act(15);

      _control(args...);

    }

  protected:
    float _sensor;
    Actuating_Interface*  _actuating;

    float _dt;

  public:
    float _sensor_value;

    void P(float _kp, float setpoint, float min, float max) {
    	db<Controller_Interface>(WRN) << "Controller_Interface::P(" << _kp << ")" <<endl;
    }
};

__END_SYS

#endif /* _CONTROLLER_H_ */
