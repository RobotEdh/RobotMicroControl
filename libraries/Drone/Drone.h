#ifndef DRONE_h
#define DRONE_h

class DroneClass
{
  public:
    DroneClass();
  
    void Drone_init(void);
    void Drone_main(void);

  private:
    const double Kp[2] = {0.6,0.6};
    const double Ki[2] = {0.1,0.1};
    const double Kd[2] = {0.3,0.3};
};

#endif