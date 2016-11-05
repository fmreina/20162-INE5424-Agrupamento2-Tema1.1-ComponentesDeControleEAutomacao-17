#ifndef __ACTUATING_H_
#define __ACTUATING_H_

__BEGIN_SYS

class Actuating
{
  public:
    Actuating()
    {
        db<Actuating>(TRC) << "Actuating()" << endl;
    }

    ~Actuating()
    {
      db<Actuating>(TRC) << "~Actuating()" << endl;
    }

  private:
    float _value;
  public:
    float act(float value){
      db<Actuating>(TRC) << "Actuating::act("<< value <<")" << endl;
      _value = value;
      return value;
    }

    /*
     * read() only for testing pourpose. Not used in real implementation
     */
    float read(){
	  db<Actuating>(TRC) << "Actuating::read()" << endl;
	  return _value;
	}
};

__END_SYS

#endif /* __ACTUATING_H_ */
