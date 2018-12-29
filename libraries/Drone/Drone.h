#ifndef DRONE_h
#define DRONE_h


#define ROLL     0
#define PITCH    1
#define YAW      2
#define THROTTLE 3


class DroneClass
{
  public:
    DroneClass();
  
    void Drone_init(void);
    void Drone_main(void);

  private:
    const double Kp[3] = {0.6,0.6,0.6};
    const double Ki[3] = {0.1,0.1,0.1};
    const double Kd[3] = {0.3,0.3,0.3};
};

#endif