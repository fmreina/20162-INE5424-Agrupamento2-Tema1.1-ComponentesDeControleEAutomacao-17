#ifndef __SENSOR_H_
#define __SENSOR_H_

__BEGIN_SYS

class Sensor
{
  public:
    Sensor(float value)
    {
        db<Sensor>(WRN) << "Sensor()" << endl;
        _value = value;
    }

    ~Sensor()
    {
      db<Sensor>(WRN) << "~Sensor()" << endl;
    }

  private:
    float _value;

  public:
    float read(){
      db<Sensor>(WRN) << "Sensor::read("<< _value <<")" << endl;
      return _value;
    }
};

__END_SYS

#endif /* __SENSOR_H_ */
