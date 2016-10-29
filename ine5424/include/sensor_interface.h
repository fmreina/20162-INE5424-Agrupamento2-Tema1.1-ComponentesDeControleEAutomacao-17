#ifndef _SENSOR_INTERFACE_H_
#define _SENSOR_INTERFACE_H_

__BEGIN_SYS

class Sensor_Interface
{
  public:
    Sensor_Interface(float value) {
      db<Sensor_Interface>(WRN) << "Sensor_Interface" << endl;
      _value = value;
    }

    ~Sensor_Interface();

    float read(){
      db<Sensor_Interface>(WRN) << "Sensor_Interface::read(" << _value << ")" << endl;
      return _value;
    }

  private:
    int _value;

  // public:
  //   Sensor_Interface(float value) : _value(value){
  //     db<Sensor_Interface>(WRN) << "Sensor_Interface" << endl;
  //   };
  //
  //   ~Sensor_Interface();
  //
  //   float read(){
  //     db<Sensor_Interface>(WRN) << "Sensor_Interface::read(" << _value << ")" << endl;
  //     return _value;
  //   };
};

__END_SYS

#endif /* _SENSOR_INTERFACE_H_ */
