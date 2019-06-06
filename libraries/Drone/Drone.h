#ifndef DRONE_h
#define DRONE_h

#include <CMPS12.h>     // Compas
#include <RC.h>         // Radio Command
#include <MotorESC.h>   // Motor ESC
#include <DS1307.h>     // RTC


#define PIDLOGFREQ 1 //record every 5 ticks ie 100 ms at 50Hz
  
class DroneClass
{
  public:
    DroneClass();
    
    uint8_t Yaw_init(void);
    void    Drone_init(void);
    void    Drone_pid(void);
    void    Drone_main(void);

  private:
    const uint32_t samplePeriod = 20;  // 20 ms (50Hz)
    const double _Kp[3] = {0.6,   0.6,   2.4};
    const double _Ki[3] = {0.02, 0.02, 0.01};  // values for 50hz (20 ms)
    const double _Kd[3] = {12.0,  12.0,  0.0}; // values for 50hz (20 ms)
    const double _IMax  = 5; // Maximum Integral value
    const double _filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";f_cut = 20 Hz -> _filter =  7.9577e-3
};

#endif