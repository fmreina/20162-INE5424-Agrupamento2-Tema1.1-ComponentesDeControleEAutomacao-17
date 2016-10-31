#ifndef _ACTUATING_INTERFACE_H_
#define _ACTUATING_INTERFACE_H_

__BEGIN_SYS

class Actuating_Interface
{
  private:


  public:
    Actuating_Interface(){
      db<Actuating_Interface>(WRN) << "Actuating_Interface" << endl;
    };

    ~Actuating_Interface();

    void act(float value){
      db<Actuating_Interface>(WRN) << "Actuating_Interface::act(" << value << ")" << endl;
    };
};

__END_SYS

#endif /* _ACTUATING_INTERFACE_H_ */
