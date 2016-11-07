#ifndef __SENSOR_H_
#define __SENSOR_H_

__BEGIN_SYS

class Sensor
{
  public:
    Sensor(float value)
    {
        db<Sensor>(TRC) << "Sensor()" << endl;
        _value = value;
    }

    ~Sensor()
    {
      db<Sensor>(TRC) << "~Sensor()" << endl;
    }

  private:
    float _value;

  public:
    float read(){
      db<Sensor>(TRC) << "Sensor::read("<< _value <<")" << endl;
      return _value;
    }

    void set(float value) {
        db<Sensor>(TRC) << "Sensor::set("<< value <<")" << endl;
    	_value = value;
    }
};

__END_SYS

#endif /* __SENSOR_H_ */
