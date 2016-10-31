#ifndef __CONTROLLER2_H_
#define __CONTROLLER2_H_

extern "C" { void __exit(); };

__BEGIN_SYS

class Controller2
{
  Controller2(){}
  ~Controller2(){}
  protected:
    float _sensor;
    float _actuating;

    // max/min output
    float _max;
    float _min;
    // previous error
    static float _prev_error;
    // dt -  loop interval time
    float _dt;
    float _setpoint;

  public:

    float calculate( double setpoint );

  protected:
    float checkLimits(float value, float min, float max){
      // Restrict to max/min
      if( value > max ){
          value = max;
      }
      else if( value < min ){
          value = min;
      }
      return value;
    }
};

__END_SYS

#endif /* _CONTROLLER_H_ */
