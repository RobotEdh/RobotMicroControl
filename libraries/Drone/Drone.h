#ifndef DRONE_h
#define DRONE_h

class DroneClass
{
  public:
    DroneClass();
  
    void Drone_init(void);
    void Drone_main(void);

  private:
    const double _Kp[2] = {0.6,0.6};
    const double _Ki[2] = {0.1,0.1};
    const double _Kd[2] = {0.3,0.3};
    const double _IMax  = 5; // Maximum Integral value
    const double _filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";f_cut = 20 Hz -> _filter =  7.9577e-3
};

#endif