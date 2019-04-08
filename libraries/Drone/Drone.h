#ifndef DRONE_h
#define DRONE_h

#include <CMPS12.h>     // Compas
#include <RC.h>         // Radio Command
#include <MotorESC.h>   // Motor ESC
#include <DS1307.h>     // RTC


#define LOGDATASIZE 512 //multiple of block size 512/8=64
#define LOGFREQ 5 //log every 5 cycles

class DroneClass
{
  public:
    DroneClass();
  
    void Drone_init(void);
    void Drone_pid(void);
    void Drone_main(void);

  private:
    const uint32_t samplePeriod = 20;  // 20 ms (50Hz)
    const double _Kp[3] = {0.6,   0.6,   2.4};
    const double _Ki[3] = {0.008, 0.008, 0.004};  // values for 20hz (50 ms)
    const double _Kd[3] = {30.0,  30.0,  0.0};    // values for 20hz (50 ms)
    const double _IMax  = 5; // Maximum Integral value
    const double _filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";f_cut = 20 Hz -> _filter =  7.9577e-3
};

#endif