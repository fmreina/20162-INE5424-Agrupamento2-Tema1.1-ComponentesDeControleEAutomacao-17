#ifndef __ACTUATING_H_
#define __ACTUATING_H_

__BEGIN_SYS

class Actuating
{
  public:
    Actuating()
    {
        db<Actuating>(WRN) << "Actuating()" << endl;
    }

    ~Actuating()
    {
      db<Actuating>(WRN) << "~Actuating()" << endl;
    }

  private:

  public:
    float act(float value){
      db<Actuating>(WRN) << "Actuating::act("<< value <<")" << endl;
      return value;
    }
};

__END_SYS

#endif /* __ACTUATING_H_ */
